#include <cmath>
#include <unity.h>

#include "pid_cascade.h"
#include "quad_mixer.h"
#include "safety_fsm.h"

using namespace fc;
using namespace fc::proto;

namespace {
SafetyInputs make_healthy_disarmed_inputs(uint32_t now_us) {
    SafetyInputs in{};
    in.now_us = now_us;
    in.imu_healthy = true;
    in.last_imu_us = now_us;
    in.gyro_calibrated = true;
    in.tilt_cos = 1.0f;
    in.vbat_v = 11.8f;
    in.rc.throttle = 0.0f;
    in.rc.arm_switch = false;
    in.rc.link_alive = true;
    in.rc.timestamp_us = now_us;
    return in;
}
} // namespace

void setUp() {}
void tearDown() {}

// =============================================================================
// 1. VALIDAÇÃO DAS 6 DIREÇÕES FÍSICAS DE CONTROLE (PID + MIXER QUAD-X FRD)
// =============================================================================
void test_six_physical_perturbation_directions_in_quad_x() {
    FlightParams params{};
    CascadedPID pid;
    pid.configure_from_params(params, 1000.0f);

    QuadXMixer mixer;
    mixer.configure(params.motor_idle, params.vbat_nominal);

    // A) Perturbação: Asa direita caiu (+10° Roll), piloto quer nivelado (0°).
    //    Os motores da DIREITA (M1 FR [0], M2 RR [1]) DEVEM acelerar mais que a ESQUERDA (M3 [2], M4 [3])!
    {
        EulerAngles att{10.0f * kDegToRad, 0.0f, 0.0f};
        const TorqueCommand tq = pid.update_stabilize(0.0f, 0.0f, 0.0f, att, {}, 0.001f, true, false);
        const MotorOutput m = mixer.mix(true, true, 0.40f, tq, std::cos(10.0f * kDegToRad), 11.1f);
        TEST_ASSERT_GREATER_THAN_FLOAT(m.throttle[2], m.throttle[0]); // M1 (FR) > M3 (RL)
        TEST_ASSERT_GREATER_THAN_FLOAT(m.throttle[3], m.throttle[1]); // M2 (RR) > M4 (FL)
    }

    // B) Perturbação: Asa esquerda caiu (-10° Roll) -> ESQUERDA (M3, M4) acelera!
    pid.reset();
    {
        EulerAngles att{-10.0f * kDegToRad, 0.0f, 0.0f};
        const TorqueCommand tq = pid.update_stabilize(0.0f, 0.0f, 0.0f, att, {}, 0.001f, true, false);
        const MotorOutput m = mixer.mix(true, true, 0.40f, tq, std::cos(10.0f * kDegToRad), 11.1f);
        TEST_ASSERT_GREATER_THAN_FLOAT(m.throttle[0], m.throttle[2]); // M3 (RL) > M1 (FR)
        TEST_ASSERT_GREATER_THAN_FLOAT(m.throttle[1], m.throttle[3]); // M4 (FL) > M2 (RR)
    }

    // C) Perturbação: Nariz caiu p/ baixo (-10° Pitch) -> FRENTE (M1 [0], M4 [3]) acelera!
    pid.reset();
    {
        EulerAngles att{0.0f, -10.0f * kDegToRad, 0.0f};
        const TorqueCommand tq = pid.update_stabilize(0.0f, 0.0f, 0.0f, att, {}, 0.001f, true, false);
        const MotorOutput m = mixer.mix(true, true, 0.40f, tq, std::cos(10.0f * kDegToRad), 11.1f);
        TEST_ASSERT_GREATER_THAN_FLOAT(m.throttle[1], m.throttle[0]); // M1 (FR) > M2 (RR)
        TEST_ASSERT_GREATER_THAN_FLOAT(m.throttle[2], m.throttle[3]); // M4 (FL) > M3 (RL)
    }

    // D) Perturbação: Nariz subiu demais (+10° Pitch) -> TRASEIRA (M2 [1], M3 [2]) acelera!
    pid.reset();
    {
        EulerAngles att{0.0f, 10.0f * kDegToRad, 0.0f};
        const TorqueCommand tq = pid.update_stabilize(0.0f, 0.0f, 0.0f, att, {}, 0.001f, true, false);
        const MotorOutput m = mixer.mix(true, true, 0.40f, tq, std::cos(10.0f * kDegToRad), 11.1f);
        TEST_ASSERT_GREATER_THAN_FLOAT(m.throttle[0], m.throttle[1]); // M2 (RR) > M1 (FR)
        TEST_ASSERT_GREATER_THAN_FLOAT(m.throttle[3], m.throttle[2]); // M3 (RL) > M4 (FL)
    }

    // E) Perturbação: Nariz girou p/ esquerda (-1.0 rad/s Yaw) -> motores CCW (M1 [0], M3 [2]) aceleram
    //    para gerar torque de reação horário (+Z) no chassi!
    pid.reset();
    {
        const TorqueCommand tq = pid.update_stabilize(0.0f, 0.0f, 0.0f, {}, {0.0f, 0.0f, -1.0f}, 0.001f, true, false);
        const MotorOutput m = mixer.mix(true, true, 0.40f, tq, 1.0f, 11.1f);
        TEST_ASSERT_GREATER_THAN_FLOAT(m.throttle[1], m.throttle[0]); // M1 (CCW) > M2 (CW)
        TEST_ASSERT_GREATER_THAN_FLOAT(m.throttle[3], m.throttle[2]); // M3 (CCW) > M4 (CW)
    }

    // F) Perturbação: Nariz girou p/ direita (+1.0 rad/s Yaw) -> motores CW (M2 [1], M4 [3]) aceleram!
    pid.reset();
    {
        const TorqueCommand tq = pid.update_stabilize(0.0f, 0.0f, 0.0f, {}, {0.0f, 0.0f, +1.0f}, 0.001f, true, false);
        const MotorOutput m = mixer.mix(true, true, 0.40f, tq, 1.0f, 11.1f);
        TEST_ASSERT_GREATER_THAN_FLOAT(m.throttle[0], m.throttle[1]); // M2 (CW) > M1 (CCW)
        TEST_ASSERT_GREATER_THAN_FLOAT(m.throttle[2], m.throttle[3]); // M4 (CW) > M3 (CCW)
    }
}

// =============================================================================
// 2. D-ON-MEASUREMENT: DEGRAU BRUSCO NO STICK NÃO GERA COICE DERIVATIVO!
// =============================================================================
void test_d_on_measurement_zero_setpoint_kick() {
    RateAxisPID pid;
    pid.configure(0.15f, 0.0f, 0.01f, 1000.0f, 50.0f);

    // Giroscópio parado em 0.0 rad/s, mas piloto dá comando brusco de 0 -> +3.0 rad/s em 1 ms!
    (void)pid.update(0.0f, 0.0f, 0.001f, true, false);
    (void)pid.update(3.0f, 0.0f, 0.001f, true, false);

    // Como o giroscópio ainda não se moveu, o termo D DEVE ser exatamente 0.0f!
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, pid.last_d());

    // Agora o drone começa a girar rápido (+2.0 rad/s em 10 ms): D deve frear (sinal negativo!)
    for (int i = 0; i < 10; ++i) {
        (void)pid.update(3.0f, 0.2f * static_cast<float>(i + 1), 0.001f, true, false);
    }
    TEST_ASSERT_LESS_THAN_FLOAT(-0.01f, pid.last_d());
}

// =============================================================================
// 3. ANTI-WINDUP EM SOLO E EM SATURAÇÃO DO MIXER
// =============================================================================
void test_pid_ground_zeroing_and_saturation_anti_windup() {
    RateAxisPID pid;
    pid.configure(0.15f, 0.50f, 0.0f, 1000.0f, 50.0f, 0.30f, 1.0f);

    // Em solo (in_air = false), mesmo com erro por 2 segundos na grama torta, I deve ser ZERO!
    for (int i = 0; i < 2000; ++i) {
        (void)pid.update(0.5f, 0.0f, 0.001f, false, false);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, pid.last_i());

    // Em voo mas com mixer saturado na direção positiva: I NÃO pode acumular!
    for (int i = 0; i < 500; ++i) {
        (void)pid.update(1.0f, 0.0f, 0.001f, true, true);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, pid.last_i());
}

// =============================================================================
// 4. DESATURAÇÃO AIRMODE + COMPENSAÇÃO VBAT E INCLINAÇÃO NO MIXER
// =============================================================================
void test_mixer_airmode_desaturation_and_vbat_compensation() {
    QuadXMixer mixer;
    mixer.configure(0.06f, 11.1f);

    // Desarmado -> 1000 µs estritos nos 4 motores
    const MotorOutput disarmed = mixer.mix(false, false, 0.80f, {0.5f, 0.5f, 0.2f}, 1.0f, 11.1f);
    TEST_ASSERT_FALSE(disarmed.armed);
    for (int i = 0; i < 4; ++i) {
        TEST_ASSERT_EQUAL_UINT16(1000u, QuadXMixer::to_pwm_us(disarmed.throttle[i], disarmed.armed));
    }

    // Piloto a 90% de throttle pede forte correção de Roll (+0.25): sem Airmode cortaria em 1.0;
    // com Airmode, o mixer recua o throttle comum para preservar a diferença de 0.50 entre lados!
    const MotorOutput m = mixer.mix(true, true, 0.90f, {0.25f, 0.0f, 0.0f}, 1.0f, 11.1f);
    TEST_ASSERT_TRUE(mixer.was_saturated());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.00f, m.throttle[2]); // Esquerda no teto 1.00
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.50f, m.throttle[0]); // Direita recuada p/ 0.50 (diferença = 0.50!)

    // Compensação VBAT: bateria cansada em 10.0V deve gerar PWM maior que bateria cheia em 12.6V
    const MotorOutput m_full = mixer.mix(true, true, 0.40f, {}, 1.0f, 12.6f);
    const MotorOutput m_low  = mixer.mix(true, true, 0.40f, {}, 1.0f, 10.0f);
    TEST_ASSERT_GREATER_THAN_FLOAT(m_full.throttle[0], m_low.throttle[0]);
}

// =============================================================================
// 5. INTERTRAVAMENTOS DE PRÉ-ARMAMENTO (NUNCA ARMA LIGANDO COM CHAVE ON!)
// =============================================================================
void test_safety_prearm_blocks_boot_with_arm_switch_already_on() {
    SafetyStateMachine fsm;
    uint32_t t = 1000000u;

    // 1. Liga o drone já com a chave ARM do controle esquecida em ON!
    SafetyInputs in = make_healthy_disarmed_inputs(t);
    in.rc.arm_switch = true;

    SafetyDecision dec = fsm.update(in, 0.001f); // Sai de BOOT_CALIBRATING -> DISARMED
    t += 1000u;
    in.now_us = in.last_imu_us = in.rc.timestamp_us = t;
    dec = fsm.update(in, 0.001f);

    TEST_ASSERT_EQUAL(SystemState::DISARMED, dec.state);
    TEST_ASSERT_FALSE(dec.motors_armed);
    TEST_ASSERT_BITS_HIGH(ERR_ARM_SWITCH_BOOT_ON, dec.prearm_flags);

    // 2. Piloto desliga a chave ARM (OFF), mas está com Throttle em 20% -> ainda bloqueia!
    t += 1000u;
    in.now_us = in.last_imu_us = in.rc.timestamp_us = t;
    in.rc.arm_switch = false;
    in.rc.throttle = 0.20f;
    dec = fsm.update(in, 0.001f);
    TEST_ASSERT_BITS_LOW(ERR_ARM_SWITCH_BOOT_ON, dec.prearm_flags);
    TEST_ASSERT_BITS_HIGH(ERR_THROTTLE_NOT_ZERO, dec.prearm_flags);

    // 3. Piloto zera o Throttle e vira ARM OFF -> ON: agora ARMA limpo em ARMED_IDLE!
    t += 1000u;
    in.now_us = in.last_imu_us = in.rc.timestamp_us = t;
    in.rc.throttle = 0.0f;
    (void)fsm.update(in, 0.001f);

    t += 1000u;
    in.now_us = in.last_imu_us = in.rc.timestamp_us = t;
    in.rc.arm_switch = true;
    dec = fsm.update(in, 0.001f);

    TEST_ASSERT_EQUAL(SystemState::ARMED_IDLE, dec.state);
    TEST_ASSERT_TRUE(dec.motors_armed);
    TEST_ASSERT_FALSE(dec.in_air);
}

// =============================================================================
// 6. FAILSAFE EM 2 ESTÁGIOS COM RECUPERAÇÃO NO AR + DETECTOR DE CAPOTAGEM > 65°
// =============================================================================
void test_two_stage_failsafe_recovery_and_crash_flip_cutoff() {
    SafetyStateMachine fsm;
    uint32_t t = 1000000u;
    SafetyInputs in = make_healthy_disarmed_inputs(t);

    // Boot -> Disarmed -> Arm -> Liftoff (FLYING)
    (void)fsm.update(in, 0.001f);
    t += 1000u; in.now_us = in.last_imu_us = in.rc.timestamp_us = t;
    in.rc.arm_switch = true;
    (void)fsm.update(in, 0.001f);
    t += 1000u; in.now_us = in.last_imu_us = in.rc.timestamp_us = t;
    in.rc.throttle = 0.45f;
    in.rc.roll = 0.30f;
    SafetyDecision dec = fsm.update(in, 0.001f);
    TEST_ASSERT_EQUAL(SystemState::FLYING, dec.state);
    TEST_ASSERT_TRUE(dec.in_air);

    // Simula perda de pacotes ESP-NOW por 350 ms (> 300 ms): entra em FAILSAFE_DESCEND (Estágio 1)!
    t += 350000u;
    in.now_us = in.last_imu_us = t; // rc.timestamp_us ficou lá atrás!
    dec = fsm.update(in, 0.35f);

    TEST_ASSERT_EQUAL(SystemState::FAILSAFE_DESCEND, dec.state);
    TEST_ASSERT_TRUE(dec.motors_armed); // NÃO cai como pedra!
    TEST_ASSERT_TRUE(dec.buzzer_siren);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, dec.effective_rc.roll);  // Nivela Roll=0!
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, dec.effective_rc.pitch); // Nivela Pitch=0!

    // Pacote ESP-NOW volta após 0.8 s -> recupera voo normal sem cair!
    t += 800000u;
    in.now_us = in.last_imu_us = in.rc.timestamp_us = t;
    dec = fsm.update(in, 0.80f);
    TEST_ASSERT_EQUAL(SystemState::FLYING, dec.state);
    TEST_ASSERT_FALSE(dec.buzzer_siren);

    // Agora simula colisão num galho capotando a 70° (cos(70°) = 0.342 < cos(65°)):
    // DEVE cortar motores na hora em FAILSAFE_LOCKED e bloquear rearmamento automático!
    t += 1000u;
    in.now_us = in.last_imu_us = in.rc.timestamp_us = t;
    in.tilt_cos = std::cos(70.0f * kDegToRad);
    dec = fsm.update(in, 0.001f);

    TEST_ASSERT_EQUAL(SystemState::FAILSAFE_LOCKED, dec.state);
    TEST_ASSERT_FALSE(dec.motors_armed);
    TEST_ASSERT_TRUE(dec.buzzer_siren);

    // Mesmo endireitando o drone no chão, continua travado até o piloto desligar a chave ARM!
    t += 1000u;
    in.now_us = in.last_imu_us = in.rc.timestamp_us = t;
    in.tilt_cos = 1.0f;
    dec = fsm.update(in, 0.001f);
    TEST_ASSERT_EQUAL(SystemState::FAILSAFE_LOCKED, dec.state);

    t += 1000u;
    in.now_us = in.last_imu_us = in.rc.timestamp_us = t;
    in.rc.arm_switch = false;
    dec = fsm.update(in, 0.001f);
    TEST_ASSERT_EQUAL(SystemState::DISARMED, dec.state);
}

int main(int /*argc*/, char** /*argv*/) {
    UNITY_BEGIN();
    RUN_TEST(test_six_physical_perturbation_directions_in_quad_x);
    RUN_TEST(test_d_on_measurement_zero_setpoint_kick);
    RUN_TEST(test_pid_ground_zeroing_and_saturation_anti_windup);
    RUN_TEST(test_mixer_airmode_desaturation_and_vbat_compensation);
    RUN_TEST(test_safety_prearm_blocks_boot_with_arm_switch_already_on);
    RUN_TEST(test_two_stage_failsafe_recovery_and_crash_flip_cutoff);
    return UNITY_END();
}
