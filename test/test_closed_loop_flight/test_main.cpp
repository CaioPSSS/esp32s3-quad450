#include <cmath>
#include <cstring>
#include <unity.h>

#include "flight_pipeline.h"
#include "mavlink_v2_min.h"
#include "quad_physics_6dof.h"

using namespace fc;
using namespace fc::mav;
using namespace fc::proto;
using namespace fc::sim;

void setUp() {}
void tearDown() {}

// 1. Valida codificação MAVLink v2 (com zero-trimming) + parser byte-a-byte
void test_mavlink_v2_encode_zero_trim_and_parse_roundtrip() {
    uint8_t seq = 7;
    uint8_t wire[280] = {};

    MsgAttitude att{};
    att.time_boot_ms = 12345;
    att.roll = 0.25f;
    att.pitch = -0.10f;
    att.yaw = 0.0f; // Campos finais = 0.0f -> devem ser truncados no fio pelo MAVLink v2!

    const size_t n = encode_mavlink_v2(MSG_ID_ATTITUDE, 1, 1, seq, att, wire);
    TEST_ASSERT_GREATER_THAN_UINT32(12u, n);
    TEST_ASSERT_LESS_THAN_UINT32(12u + sizeof(MsgAttitude), n); // Prova que truncou zeros!

    MavlinkV2Parser parser;
    ParsedMavMessage rx{};
    bool got = false;
    for (size_t i = 0; i < n; ++i) {
        if (parser.push_byte(wire[i], rx)) {
            got = true;
        }
    }
    TEST_ASSERT_TRUE(got);
    TEST_ASSERT_EQUAL_UINT32(MSG_ID_ATTITUDE, rx.msgid);
    const auto decoded = rx.unpack<MsgAttitude>();
    TEST_ASSERT_EQUAL_UINT32(12345u, decoded.time_boot_ms);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.25f, decoded.roll);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.10f, decoded.pitch);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, decoded.yawspeed);
}

// 2. Voo completo 6-DoF em malha fechada (14 segundos simulados @ 1000 Hz):
//    Calibração em rampa -> Decolagem -> Resposta ao degrau de +20° Roll ->
//    Rejeição de rajada lateral de vento -> Giro de Yaw -> Perda de rádio (Failsafe Estágio 1)!
void test_full_closed_loop_6dof_flight_mission() {
    FlightParams params{};
    FlightPipeline pipeline;
    pipeline.init(params, 1000.0f, 1000); // 1.0s calibração

    QuadPhysics6DoF sim;
    sim.reset(3.0f, -2.0f); // Inicia pousado em rampa de +3° Roll e -2° Pitch!

    RcCommand rc{};
    rc.link_alive = true;

    constexpr float dt = 0.001f;
    auto run_for_seconds = [&](float duration_s, uint32_t& io_step) {
        const int steps = static_cast<int>(std::lround(duration_s / dt));
        PipelineStepOutput last{};
        for (int k = 0; k < steps; ++k, ++io_step) {
            const uint32_t now_us = io_step * 1000u;
            if (rc.link_alive) {
                rc.timestamp_us = now_us;
            }
            const ImuSample imu = sim.sample_imu(now_us);
            last = pipeline.step_1khz(imu, rc, sim.truth().vbat_loaded_v, now_us, dt);
            sim.step(last.motors, dt);
        }
        return last;
    };

    uint32_t step = 0;

    // Fase A (1.2 s): Calibração no solo inclinado -> deve transitar p/ DISARMED alinhado à rampa!
    PipelineStepOutput out = run_for_seconds(1.2f, step);
    TEST_ASSERT_EQUAL(SystemState::DISARMED, out.telemetry.state);
    TEST_ASSERT_FLOAT_WITHIN(0.25f,  3.0f, out.telemetry.attitude.roll_deg());
    TEST_ASSERT_FLOAT_WITHIN(0.25f, -2.0f, out.telemetry.attitude.pitch_deg());

    // Fase B (0.2 s): Arma em ARMED_IDLE
    rc.arm_switch = true;
    rc.throttle = 0.0f;
    out = run_for_seconds(0.2f, step);
    TEST_ASSERT_EQUAL(SystemState::ARMED_IDLE, out.telemetry.state);
    TEST_ASSERT_TRUE(out.motors.armed);

    // Fase C (2.5 s): Decola de rampa torta com stick centralizado -> deve subir e nivelar a 0.0°!
    rc.throttle = 0.52f;
    out = run_for_seconds(2.5f, step);
    TEST_ASSERT_EQUAL(SystemState::FLYING, out.telemetry.state);
    TEST_ASSERT_GREATER_THAN_FLOAT(2.0f, sim.truth().altitude_m());
    TEST_ASSERT_FLOAT_WITHIN(0.4f, 0.0f, sim.truth().euler.roll_deg());
    TEST_ASSERT_FLOAT_WITHIN(0.4f, 0.0f, sim.truth().euler.pitch_deg());

    // Fase D (1.8 s): Comando de Roll (+0.60 stick -> ~+19.3° após deadband/expo)
    rc.throttle = 0.40f;
    rc.roll = 0.60f;
    out = run_for_seconds(1.8f, step);
    TEST_ASSERT_GREATER_THAN_FLOAT(14.0f, sim.truth().euler.roll_deg());
    TEST_ASSERT_LESS_THAN_FLOAT(25.0f, sim.truth().euler.roll_deg());

    // Fase E (2.5 s): Solta o stick (Roll=0) + aplica rajada constante de torque externo (0.20 N.m)!
    //                 O integrador do CascadedPID deve vencer a rajada e cravar 0.0°!
    rc.roll = 0.0f;
    sim.set_external_torque_body({0.20f, -0.15f, 0.0f});
    out = run_for_seconds(3.0f, step);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, sim.truth().euler.roll_deg());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, sim.truth().euler.pitch_deg());
    sim.set_external_torque_body({0.0f, 0.0f, 0.0f});

    // Fase F (1.5 s): Giro coordenado de Yaw (+0.50 stick) mantendo Roll/Pitch nivelados
    const float yaw_before = sim.truth().euler.yaw_deg();
    rc.yaw = 0.50f;
    out = run_for_seconds(1.5f, step);
    rc.yaw = 0.0f;
    TEST_ASSERT_GREATER_THAN_FLOAT(yaw_before + 25.0f, sim.truth().euler.yaw_deg());
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 0.0f, sim.truth().euler.roll_deg());

    // Fase G (1.5 s): Corta link ESP-NOW em voo com drone inclinado -> FAILSAFE_DESCEND nivela e desce!
    rc.roll = 0.50f;
    (void)run_for_seconds(0.5f, step);
    const float alt_at_loss = sim.truth().altitude_m();
    rc.link_alive = false;
    out = run_for_seconds(1.5f, step);

    TEST_ASSERT_EQUAL(SystemState::FAILSAFE_DESCEND, out.telemetry.state);
    TEST_ASSERT_TRUE(out.buzzer_siren);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, sim.truth().euler.roll_deg()); // Nivelou sozinho!
    TEST_ASSERT_LESS_THAN_FLOAT(alt_at_loss, sim.truth().altitude_m()); // Descendo controlado!
}

int main(int /*argc*/, char** /*argv*/) {
    UNITY_BEGIN();
    RUN_TEST(test_mavlink_v2_encode_zero_trim_and_parse_roundtrip);
    RUN_TEST(test_full_closed_loop_6dof_flight_mission);
    return UNITY_END();
}
