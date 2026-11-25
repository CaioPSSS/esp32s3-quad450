#include <cmath>
#include <limits>
#include <unity.h>

#include "gyro_calibrator.h"
#include "mahony_ahrs.h"

using namespace fc;

namespace {
// Gera a força específica exata medida por um acelerômetro FRD parado com Roll e Pitch dados
Vec3f synthetic_stationary_accel_frd(float roll_deg, float pitch_deg) {
    const float r = roll_deg * kDegToRad;
    const float p = pitch_deg * kDegToRad;
    // g_b = g * [-sin(p), sin(r)*cos(p), cos(r)*cos(p)], e f_b = -g_b
    return {
         kGravityMs2 * std::sin(p),
        -kGravityMs2 * std::sin(r) * std::cos(p),
        -kGravityMs2 * std::cos(r) * std::cos(p)
    };
}
} // namespace

void setUp() {}
void tearDown() {}

// 1. Repouso plano na mesa: Roll=0, Pitch=0, tilt_cos=1 e aceleração vertical = 0 m/s²
void test_stationary_flat_equilibrium() {
    MahonyAHRS ahrs(2.0f, 0.05f);
    const Vec3f flat_acc = synthetic_stationary_accel_frd(0.0f, 0.0f);

    for (int i = 0; i < 1000; ++i) {
        ahrs.update({0.0f, 0.0f, 0.0f}, flat_acc, 0.001f);
    }

    TEST_ASSERT_TRUE(ahrs.is_accel_gate_open());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ahrs.euler().roll_deg());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ahrs.euler().pitch_deg());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, ahrs.tilt_cos());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, ahrs.earth_accel_up_ms2());
}

// 2. Convergência exata de sinais em Roll (+30°) e Pitch (-20°) partindo de q=[1,0,0,0]
void test_roll_and_pitch_static_convergence() {
    MahonyAHRS ahrs(2.5f, 0.0f);
    const Vec3f tilted_acc = synthetic_stationary_accel_frd(30.0f, -20.0f);

    for (int i = 0; i < 4000; ++i) {
        ahrs.update({0.0f, 0.0f, 0.0f}, tilted_acc, 0.001f);
    }

    TEST_ASSERT_FLOAT_WITHIN(0.05f,  30.0f, ahrs.euler().roll_deg());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, -20.0f, ahrs.euler().pitch_deg());
    // Mesmo inclinado em 30°/-20°, como está parado, a aceleração cinemática vertical deve ser 0!
    TEST_ASSERT_FLOAT_WITHIN(0.02f,   0.0f, ahrs.earth_accel_up_ms2());
}

// 3. Alinhamento instantâneo no boot em grama inclinada (+12° Roll, +8° Pitch, -45° Yaw)
void test_instant_align_to_gravity_on_sloped_ground() {
    MahonyAHRS ahrs;
    const Vec3f sloped_acc = synthetic_stationary_accel_frd(12.0f, 8.0f);

    TEST_ASSERT_TRUE(ahrs.align_to_gravity(sloped_acc, -45.0f * kDegToRad));
    TEST_ASSERT_FLOAT_WITHIN(0.02f,  12.0f, ahrs.euler().roll_deg());
    TEST_ASSERT_FLOAT_WITHIN(0.02f,   8.0f, ahrs.euler().pitch_deg());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, -45.0f, ahrs.euler().yaw_deg());
    TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.0f, ahrs.earth_accel_up_ms2());
}

// 4. Ausência de Gimbal Lock perto de +89° de Pitch em integração de Quaternions
void test_near_vertical_pitch_no_gimbal_lock() {
    MahonyAHRS ahrs(0.0f, 0.0f); // Pura integração de giroscópio
    const float pitch_rate_rads = 89.0f * kDegToRad; // 89 deg/s por 1.0 segundo

    for (int i = 0; i < 1000; ++i) {
        ahrs.update({0.0f, pitch_rate_rads, 0.0f}, {0.0f, 0.0f, 0.0f}, 0.001f);
    }

    const Quatf& q = ahrs.quaternion();
    const float q_norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, q_norm);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 89.0f, ahrs.euler().pitch_deg());
    TEST_ASSERT_FLOAT_WITHIN(0.02f, std::cos(89.0f * kDegToRad), ahrs.tilt_cos());
}

// 5. Rejeição de puxada forte de 2.0g + projeção vertical correta p/ Altitude Hold
void test_2g_climb_rejects_false_horizon_and_projects_upward_accel() {
    MahonyAHRS ahrs(3.0f, 0.05f);
    const Vec3f level_acc = synthetic_stationary_accel_frd(0.0f, 0.0f);
    ahrs.align_to_gravity(level_acc);

    // Simula subida acelerada a +1g cinemático (+9.80665 m/s² p/ cima => f_z = -2g) com leve ruído lateral
    const Vec3f climb_2g_acc{2.5f, -2.5f, -2.0f * kGravityMs2};
    for (int i = 0; i < 200; ++i) {
        ahrs.update({0.0f, 0.0f, 0.0f}, climb_2g_acc, 0.001f);
    }

    // Porta de gravidade deve fechar (||a|| > 1.2g), mantendo Roll e Pitch intactos em 0.0°!
    TEST_ASSERT_FALSE(ahrs.is_accel_gate_open());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ahrs.euler().roll_deg());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ahrs.euler().pitch_deg());
    // Aceleração vertical na Terra deve acusar +1g (+9.80665 m/s² subindo)!
    TEST_ASSERT_FLOAT_WITHIN(0.05f, kGravityMs2, ahrs.earth_accel_up_ms2());
}

// 6. Termo Integral Ki aprende e cancela deriva térmica constante do giroscópio
void test_integral_term_cancels_constant_gyro_drift() {
    // Polo lento ~ Ki/Kp = 0.5 / 2.5 = 0.2 rad/s (tau ~= 5 s) -> 30 s = 6 constantes de tempo
    MahonyAHRS ahrs(2.5f, 0.50f);
    const Vec3f flat_acc = synthetic_stationary_accel_frd(0.0f, 0.0f);
    ahrs.align_to_gravity(flat_acc);

    // Giroscópio com erro térmico constante de +0.06 rad/s (~3.4 deg/s) no Roll, mas drone parado
    for (int i = 0; i < 30000; ++i) {
        ahrs.update({0.06f, -0.04f, 0.0f}, flat_acc, 0.001f);
    }

    TEST_ASSERT_FLOAT_WITHIN(0.02f,  0.0f, ahrs.euler().roll_deg());
    TEST_ASSERT_FLOAT_WITHIN(0.02f,  0.0f, ahrs.euler().pitch_deg());
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.06f, ahrs.estimated_gyro_bias_rads().x);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, -0.04f, ahrs.estimated_gyro_bias_rads().y);
}

// 7. Fusão 9-DoF de proa (Fase 2 Bússola) corrige Yaw sem perturbar Roll/Pitch
void test_heading_fusion_aligns_yaw_cleanly() {
    MahonyAHRS ahrs(2.0f, 0.05f);
    const Vec3f tilted_acc = synthetic_stationary_accel_frd(15.0f, -10.0f);
    ahrs.align_to_gravity(tilted_acc, 0.0f);

    const float target_yaw = 60.0f * kDegToRad;
    for (int i = 0; i < 3000; ++i) {
        ahrs.update_with_heading({0.0f, 0.0f, 0.0f}, tilted_acc, target_yaw, 2.0f, 0.001f);
    }

    TEST_ASSERT_FLOAT_WITHIN(0.15f,  15.0f, ahrs.euler().roll_deg());
    TEST_ASSERT_FLOAT_WITHIN(0.15f, -10.0f, ahrs.euler().pitch_deg());
    TEST_ASSERT_FLOAT_WITHIN(0.25f,  60.0f, ahrs.euler().yaw_deg());
}

// 8. Blindagem contra NaN na IMU
void test_nan_packet_does_not_corrupt_quaternion() {
    MahonyAHRS ahrs(2.0f, 0.05f);
    const Vec3f flat_acc = synthetic_stationary_accel_frd(5.0f, 0.0f);
    ahrs.align_to_gravity(flat_acc);

    const float nan_v = std::numeric_limits<float>::quiet_NaN();
    ahrs.update({nan_v, 0.0f, 0.0f}, flat_acc, 0.001f);
    ahrs.update({0.0f, 0.0f, 0.0f}, {0.0f, nan_v, -9.81f}, 0.001f);

    TEST_ASSERT_TRUE(std::isfinite(ahrs.euler().roll_deg()));
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 5.0f, ahrs.euler().roll_deg());
}

// 9. GyroCalibrator rejeita balanço da mão ao plugar XT60 e depois calibra com precisão
void test_gyro_calibrator_rejects_hand_shake_then_converges() {
    GyroCalibrator cal(1000, 0.035f); // 1.0 s parado @ 1 kHz
    const Vec3f true_bias{-0.022f, 0.031f, -0.014f};
    const Vec3f flat_acc = synthetic_stationary_accel_frd(3.0f, -2.0f);

    // Primeiros 400 ms: piloto soltando o cabo XT60 (oscilação de 0.30 rad/s)
    for (int i = 0; i < 400; ++i) {
        const float shake = 0.30f * std::sin(0.25f * static_cast<float>(i));
        const bool done = cal.push_sample(
            {true_bias.x + shake, true_bias.y - shake, true_bias.z},
            flat_acc);
        TEST_ASSERT_FALSE(done);
    }
    TEST_ASSERT_GREATER_THAN_UINT32(0u, cal.motion_resets());

    // Drone agora parado na grama por 1000 amostras (com ruído branco realista pequeno)
    bool finished = false;
    for (int i = 0; i < 1050 && !finished; ++i) {
        const float micro_noise = 0.003f * std::sin(1.7f * static_cast<float>(i));
        finished = cal.push_sample(
            {true_bias.x + micro_noise, true_bias.y - micro_noise, true_bias.z + micro_noise},
            flat_acc);
    }

    TEST_ASSERT_TRUE(finished);
    TEST_ASSERT_TRUE(cal.is_calibrated());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, true_bias.x, cal.gyro_bias_rads().x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, true_bias.y, cal.gyro_bias_rads().y);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, true_bias.z, cal.gyro_bias_rads().z);
}

int main(int /*argc*/, char** /*argv*/) {
    UNITY_BEGIN();
    RUN_TEST(test_stationary_flat_equilibrium);
    RUN_TEST(test_roll_and_pitch_static_convergence);
    RUN_TEST(test_instant_align_to_gravity_on_sloped_ground);
    RUN_TEST(test_near_vertical_pitch_no_gimbal_lock);
    RUN_TEST(test_2g_climb_rejects_false_horizon_and_projects_upward_accel);
    RUN_TEST(test_integral_term_cancels_constant_gyro_drift);
    RUN_TEST(test_heading_fusion_aligns_yaw_cleanly);
    RUN_TEST(test_nan_packet_does_not_corrupt_quaternion);
    RUN_TEST(test_gyro_calibrator_rejects_hand_shake_then_converges);
    return UNITY_END();
}
