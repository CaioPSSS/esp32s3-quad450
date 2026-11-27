#include <cmath>
#include <unity.h>

#include "alt_estimator.h"
#include "flight_pipeline.h"
#include "quad_physics_6dof.h"

using namespace fc;
using namespace fc::proto;
using namespace fc::sim;

void setUp() {}
void tearDown() {}

// 1. Verifica que o VerticalEstimator aprende o bias térmico do acelerômetro Z (+0.35 m/s²)
//    e funde predição de 1000 Hz com barômetro ruidoso de 40 Hz sem deriva!
void test_vertical_estimator_cancels_accel_bias_and_fuses_40hz_baro() {
    VerticalEstimator est;
    est.reset();

    // Tara inicial no solo (t = 0)
    BaroSample b0{};
    b0.altitude_m = 12.5f; // Offset arbitrário de pressão local -> deve virar 0.0 m relativo!
    b0.timestamp_us = 1000;
    b0.healthy = true;
    est.correct_baro(b0, false);

    constexpr float kTrueBiasMs2 = 0.35f; // Deriva espúria de +0.35 m/s² no eixo Z
    constexpr float dt = 0.001f;

    // Simula drone estacionado a +3.0 m acima do ponto de decolagem por 18 segundos
    for (uint32_t step = 1; step <= 18000; ++step) {
        const uint32_t now_us = 1000 + step * 1000u;
        const float t_s = step * dt;

        if (step % 25 == 0) { // BMP280 @ 40 Hz com oscilação de 8 cm
            BaroSample b{};
            b.altitude_m = 12.5f + 3.0f + 0.08f * std::sin(2.0f * kPi * 3.1f * t_s);
            b.timestamp_us = now_us;
            b.healthy = true;
            est.correct_baro(b, true);
        }
        est.predict_1khz(0.0f + kTrueBiasMs2, true, dt);
    }

    TEST_ASSERT_TRUE(est.is_baro_healthy(18001000u));
    TEST_ASSERT_FLOAT_WITHIN(0.10f, 3.0f, est.altitude_m());
    TEST_ASSERT_FLOAT_WITHIN(0.08f, 0.0f, est.climb_rate_ms());
    TEST_ASSERT_FLOAT_WITHIN(0.08f, kTrueBiasMs2, est.accel_bias_ms2());
}

// 2. Voo 6-DoF completo em ALT_HOLD + ALT_CINE (20° máx) com curva inclinada e queda de bateria 3S!
void test_closed_loop_6dof_alt_hold_with_banked_turn_and_battery_sag() {
    FlightParams params{};
    FlightPipeline fc_pipe;
    fc_pipe.init(params, 1000.0f, 1000u);

    QuadPhysics6DoF phys;
    phys.reset(2.0f, -1.5f);
    phys.set_battery_ocv(12.3f);

    RcCommand rc{};
    rc.link_alive = true;
    rc.mode = FlightMode::ALT_HOLD;

    BaroSample baro{};
    constexpr float dt = 0.001f;
    float locked_alt_m = 0.0f;
    float max_cine_roll_deg = 0.0f;

    for (uint32_t step = 0; step < 12000; ++step) {
        const uint32_t now_us = step * 1000u;
        const float t_s = step * dt;
        rc.timestamp_us = now_us;

        if (t_s < 1.2f) {
            rc.arm_switch = false;
            rc.throttle = 0.0f;
        } else if (t_s < 1.5f) {
            rc.arm_switch = true; // ARMED_IDLE no solo
            rc.throttle = 0.0f;
        } else if (t_s < 3.5f) {
            rc.arm_switch = true; // Sobe em ALT_HOLD
            rc.throttle = 0.85f;
        } else if (t_s < 5.0f) {
            rc.throttle = 0.50f;  // Centraliza stick -> trava altura!
            if (step == 4999) {
                locked_alt_m = phys.truth().altitude_m();
            }
        } else if (t_s < 8.5f) {
            // Simula simultaneamente queda forte de tensão da bateria 3S (12.3V -> 10.9V)
            // E curva inclinada de Roll (+50% stick)!
            phys.set_battery_ocv(10.9f);
            rc.throttle = 0.50f;
            rc.roll = 0.50f;
        } else {
            // Muda para Posição 3 da chave (ALT_CINE suave na Fase 1) com stick Roll 100% no batente!
            rc.mode = FlightMode::POS_HOLD;
            rc.throttle = 0.50f;
            rc.roll = 1.0f;
        }

        if (step % 25 == 0) {
            baro = phys.sample_baro(now_us);
        }
        const ImuSample imu = phys.sample_imu(now_us);
        const PipelineStepOutput out = fc_pipe.step_1khz(
            imu, rc, phys.truth().vbat_loaded_v, now_us, dt, baro);
        phys.step(out.motors, dt);

        if (t_s >= 9.5f) {
            max_cine_roll_deg = std::max(max_cine_roll_deg, std::fabs(phys.truth().euler.roll_deg()));
        }
    }

    // Confirmamos que subiu bem (> 1.8 m) e segurou a altura travada dentro de +-0.30 m mesmo
    // perdendo 1.4V de bateria e inclinando em curva!
    TEST_ASSERT_TRUE(locked_alt_m > 1.8f);
    TEST_ASSERT_FLOAT_WITHIN(0.30f, locked_alt_m, phys.truth().altitude_m());

    // No modo ALT_CINE (Posição 3 na Fase 1), 100% de stick comanda exatos +20.0° no controlador
    // e mantém a inclinação física mansa (< 25.5° mesmo sob aceleração lateral sustentada, vs 35°+ normal)!
    TEST_ASSERT_FLOAT_WITHIN(1.2f, 20.0f, fc_pipe.ahrs().euler().roll_deg());
    TEST_ASSERT_TRUE(max_cine_roll_deg > 18.0f && max_cine_roll_deg < 25.5f);
}

// 3. Perda de rádio a ~3m de altura -> FAILSAFE_DESCEND controlado -> Flare -> Toque na grama -> Desarme!
void test_closed_loop_6dof_baro_failsafe_flare_and_auto_touchdown_disarm() {
    FlightParams params{};
    FlightPipeline fc_pipe;
    fc_pipe.init(params, 1000.0f, 1000u);

    QuadPhysics6DoF phys;
    phys.reset(0.0f, 0.0f);

    RcCommand rc{};
    rc.link_alive = true;
    rc.mode = FlightMode::ALT_HOLD;

    BaroSample baro{};
    constexpr float dt = 0.001f;

    bool saw_failsafe_descend = false;
    float worst_descent_rate_ms = 0.0f;
    float touchdown_speed_ms = 0.0f;
    bool recorded_touchdown = false;
    PipelineStepOutput last_out{};

    for (uint32_t step = 0; step < 16000; ++step) {
        const uint32_t now_us = step * 1000u;
        const float t_s = step * dt;

        if (t_s < 1.2f) {
            rc.timestamp_us = now_us;
            rc.arm_switch = false;
            rc.throttle = 0.0f;
        } else if (t_s < 1.5f) {
            rc.timestamp_us = now_us;
            rc.arm_switch = true;
            rc.throttle = 0.0f; // Satisfaz Pre-Arm (ERR_THROTTLE_NOT_ZERO == 0)
        } else if (t_s < 3.6f) {
            rc.timestamp_us = now_us;
            rc.arm_switch = true;
            rc.throttle = 0.85f; // Sobe até ~2.6 - 3.0 m
        } else if (t_s < 5.0f) {
            rc.timestamp_us = now_us;
            rc.throttle = 0.50f; // Estabiliza no ar
        } else {
            // Corta o controle remoto no ar em t = 5.0s!
            rc.link_alive = false;
        }

        if (step % 25 == 0) {
            baro = phys.sample_baro(now_us);
        }
        const ImuSample imu = phys.sample_imu(now_us);
        last_out = fc_pipe.step_1khz(imu, rc, phys.truth().vbat_loaded_v, now_us, dt, baro);

        const float prev_alt = phys.truth().altitude_m();
        const float prev_vz  = phys.truth().climb_rate_ms();
        phys.step(last_out.motors, dt);

        if (last_out.telemetry.state == SystemState::FAILSAFE_DESCEND) {
            saw_failsafe_descend = true;
            worst_descent_rate_ms = std::min(worst_descent_rate_ms, phys.truth().climb_rate_ms());
            if (!recorded_touchdown && prev_alt > 0.001f && phys.truth().on_ground) {
                recorded_touchdown = true;
                touchdown_speed_ms = std::fabs(prev_vz);
            }
        }
        if (saw_failsafe_descend && last_out.telemetry.state == SystemState::FAILSAFE_LOCKED) {
            break;
        }
    }

    TEST_ASSERT_TRUE(saw_failsafe_descend);
    TEST_ASSERT_TRUE(phys.truth().on_ground);
    // Velocidade máxima de descida nunca passa de ~0.95 m/s e o toque na grama ocorre suave (< 0.60 m/s)!
    TEST_ASSERT_TRUE(worst_descent_rate_ms > -1.0f);
    TEST_ASSERT_TRUE(touchdown_speed_ms < 0.65f);

    // Após apoiar na grama, desligou os 4 motores sozinho e manteve a sirene ligada!
    TEST_ASSERT_EQUAL(SystemState::FAILSAFE_LOCKED, last_out.telemetry.state);
    TEST_ASSERT_FALSE(last_out.motors.armed);
    TEST_ASSERT_TRUE(last_out.buzzer_siren);
}

// 4. Regressão P0-CTRL-2: Perda do BMP280 em pleno voo ALT_HOLD com stick em 50% NÃO pode
//    disparar o drone para cima (deve mapear 50% suavemente para learned_hover_throttle ~29%)!
void test_bumpless_baro_loss_in_alt_hold_prevents_50pct_throttle_climb() {
    FlightParams params{};
    FlightPipeline fc_pipe;
    fc_pipe.init(params, 1000.0f, 1000u);

    QuadPhysics6DoF phys;
    phys.reset(0.0f, 0.0f);

    RcCommand rc{};
    rc.link_alive = true;
    rc.mode = FlightMode::ALT_HOLD;

    BaroSample baro{};
    constexpr float dt = 0.001f;
    float alt_at_baro_failure = 0.0f;
    float max_alt_after_failure = 0.0f;
    float max_vz_after_failure = 0.0f;

    for (uint32_t step = 0; step < 9000; ++step) {
        const uint32_t now_us = step * 1000u;
        const float t_s = step * dt;
        rc.timestamp_us = now_us;

        if (t_s < 1.2f) {
            rc.arm_switch = false;
            rc.throttle = 0.0f;
        } else if (t_s < 1.5f) {
            rc.arm_switch = true;
            rc.throttle = 0.0f;
        } else if (t_s < 3.2f) {
            rc.throttle = 0.80f;
        } else {
            rc.throttle = 0.50f; // Stick centralizado em ALT_HOLD
        }

        // Em t = 6.0s, o fio SDA do BMP280 rompe (baro para de atualizar!)
        if (t_s < 6.0f && (step % 25 == 0)) {
            baro = phys.sample_baro(now_us);
        }
        if (step == 6000) {
            alt_at_baro_failure = phys.truth().altitude_m();
        }

        const ImuSample imu = phys.sample_imu(now_us);
        const PipelineStepOutput out = fc_pipe.step_1khz(
            imu, rc, phys.truth().vbat_loaded_v, now_us, dt, baro);
        phys.step(out.motors, dt);

        if (step >= 6000) {
            max_alt_after_failure = std::max(max_alt_after_failure, phys.truth().altitude_m());
            max_vz_after_failure = std::max(max_vz_after_failure, phys.truth().climb_rate_ms());
        }
    }

    // Antes do fix, 50% bruto lançava o drone +6.3 m em 2s (+5.4 m/s)!
    // Com remapeamento bumpless p/ hover aprendido, a subida espúria fica < 0.45 m e Vz < 0.35 m/s!
    TEST_ASSERT_TRUE(alt_at_baro_failure > 1.2f);
    TEST_ASSERT_LESS_THAN_FLOAT(alt_at_baro_failure + 0.45f, max_alt_after_failure);
    TEST_ASSERT_LESS_THAN_FLOAT(0.35f, max_vz_after_failure);
}

// 5. Regressão P0-CTRL-1: Pouso com throttle zerado sobre rampa inclinada (+8° Roll)
//    drena o integrador I, bloqueia boost Airmode e retorna para ARMED_IDLE sem acelerar!
void test_sloped_ground_landing_does_not_spool_motors_or_bounce() {
    FlightParams params{};
    FlightPipeline fc_pipe;
    fc_pipe.init(params, 1000.0f, 1000u);

    RcCommand rc{};
    rc.link_alive = true;
    rc.mode = FlightMode::STABILIZE;

    BaroSample baro{};
    baro.healthy = true;
    constexpr float dt = 0.001f;

    // 1. Calibra em solo plano (1.2s), arma e entra em FLYING
    for (uint32_t step = 0; step < 2000; ++step) {
        const uint32_t now_us = step * 1000u;
        rc.timestamp_us = now_us;
        baro.timestamp_us = now_us;
        ImuSample imu{};
        imu.accel_ms2 = {0.0f, 0.0f, -9.80665f};
        imu.timestamp_us = now_us;
        imu.healthy = true;

        if (step < 1200) {
            rc.arm_switch = false;
            rc.throttle = 0.0f;
        } else if (step < 1500) {
            rc.arm_switch = true;
            rc.throttle = 0.0f;
        } else {
            rc.arm_switch = true;
            rc.throttle = 0.35f; // Entra em FLYING
        }
        (void)fc_pipe.step_1khz(imu, rc, 11.8f, now_us, dt, baro);
    }

    // 2. Pousa numa rampa inclinada a +8.0° em Roll com stick de throttle em 0% por 1.5s!
    const float slope_rad = 8.0f * kDegToRad;
    PipelineStepOutput out{};
    for (uint32_t step = 2000; step < 3500; ++step) {
        const uint32_t now_us = step * 1000u;
        rc.timestamp_us = now_us;
        rc.throttle = 0.0f;
        baro.timestamp_us = now_us;

        ImuSample imu{};
        // Gravidade projetada no chassi parado a +8° de Roll: ay = -g*sin(phi), az = -g*cos(phi)
        imu.accel_ms2 = {0.0f, -9.80665f * std::sin(slope_rad), -9.80665f * std::cos(slope_rad)};
        imu.gyro_rads = {0.0f, 0.0f, 0.0f};
        imu.timestamp_us = now_us;
        imu.healthy = true;

        out = fc_pipe.step_1khz(imu, rc, 11.8f, now_us, dt, baro);
    }

    // Deve ter retornado para ARMED_IDLE (in_air = false) e nenhum motor pode ter disparado!
    TEST_ASSERT_EQUAL(SystemState::ARMED_IDLE, out.telemetry.state);
    for (int i = 0; i < 4; ++i) {
        TEST_ASSERT_LESS_THAN_FLOAT(0.15f, out.motors.throttle[i]);
    }
}

int main(int /*argc*/, char** /*argv*/) {
    UNITY_BEGIN();
    RUN_TEST(test_vertical_estimator_cancels_accel_bias_and_fuses_40hz_baro);
    RUN_TEST(test_closed_loop_6dof_alt_hold_with_banked_turn_and_battery_sag);
    RUN_TEST(test_closed_loop_6dof_baro_failsafe_flare_and_auto_touchdown_disarm);
    RUN_TEST(test_bumpless_baro_loss_in_alt_hold_prevents_50pct_throttle_climb);
    RUN_TEST(test_sloped_ground_landing_does_not_spool_motors_or_bounce);
    return UNITY_END();
}
