#include <cmath>
#include <limits>
#include <unity.h>

#include "biquad_lpf.h"

using namespace fc;

void setUp() {}
void tearDown() {}

// 1. Guarda de segurança: frequências inválidas ou acima de Nyquist viram passthrough
void test_passthrough_guard_on_invalid_cutoffs() {
    BiquadLPF f;
    f.configure(1000.0f, 0.0f);
    TEST_ASSERT_TRUE(f.is_passthrough());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 12.34f, f.apply(12.34f));

    f.configure(1000.0f, 495.0f); // >= 0.49 * fs
    TEST_ASSERT_TRUE(f.is_passthrough());
}

// 2. Ganho DC unitário: gravidade constante 9.80665 m/s² deve convergir sem erro de escala!
void test_dc_unity_gain_gravity() {
    BiquadLPF accel_z_filter(1000.0f, 30.0f);
    TEST_ASSERT_FALSE(accel_z_filter.is_passthrough());

    float out = 0.0f;
    for (int i = 0; i < 500; ++i) {
        out = accel_z_filter.apply(kGravityMs2);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, kGravityMs2, out);
}

// 3. Ponto exato de -3 dB do Butterworth: senoide na frequência de corte (80 Hz @ 1000 Hz)
//    deve ter amplitude de saída = 1 / sqrt(2) ~= 0.7071!
void test_minus_3db_at_cutoff_frequency() {
    constexpr float fs = 1000.0f;
    constexpr float fc_hz = 80.0f;
    BiquadLPF gyro_filter(fs, fc_hz);

    float peak = 0.0f;
    for (int n = 0; n < 500; ++n) {
        const float t = static_cast<float>(n) / fs;
        const float x = std::sin(2.0f * kPi * fc_hz * t);
        const float y = gyro_filter.apply(x);
        if (n >= 200 && std::fabs(y) > peak) {
            peak = std::fabs(y);
        }
    }

    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.7071f, peak);
}

// 4. Caso real do drone 450 mm: comando lento de Roll (2 Hz, amp 1.5 rad/s) poluído por
//    vibração severa de hélice/motor A2212 (200 Hz, amp 4.0 rad/s).
void test_propeller_vibration_rejection() {
    constexpr float fs = 1000.0f;
    BiquadLPF gyro_filter(fs, 60.0f);

    float max_tracking_error = 0.0f;
    for (int n = 0; n < 1000; ++n) {
        const float t = static_cast<float>(n) / fs;
        const float clean_roll_rate = 1.5f * std::sin(2.0f * kPi * 2.0f * t);
        const float motor_vib_noise = 4.0f * std::sin(2.0f * kPi * 200.0f * t);
        const float filtered = gyro_filter.apply(clean_roll_rate + motor_vib_noise);

        if (n >= 300) {
            const float err = std::fabs(filtered - clean_roll_rate);
            if (err > max_tracking_error) {
                max_tracking_error = err;
            }
        }
    }

    TEST_ASSERT_LESS_THAN_FLOAT(0.45f, max_tracking_error);
}

// 5. Reset limpa memória interna imediatamente
void test_reset_clears_delay_registers() {
    BiquadLPF f(1000.0f, 50.0f);
    for (int i = 0; i < 100; ++i) {
        (void)f.apply(100.0f);
    }
    f.reset();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, f.apply(0.0f));
}

// 6. Wrapper 3D BiquadVec3 filtra X, Y e Z sem acoplamento cruzado
void test_biquad_vec3_axes_independent() {
    BiquadVec3 fv;
    fv.configure(1000.0f, 40.0f);

    Vec3f out{};
    for (int i = 0; i < 400; ++i) {
        out = fv.apply({1.0f, -2.5f, 9.81f});
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, out.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -2.5f, out.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 9.81f, out.z);
}

// 7. Imunidade a glitch SPI (NaN / Inf não pode envenenar registradores IIR!)
void test_nan_and_inf_immunity() {
    BiquadLPF f(1000.0f, 80.0f);
    for (int i = 0; i < 200; ++i) {
        (void)f.apply(1.25f);
    }
    const float out_on_nan = f.apply(std::numeric_limits<float>::quiet_NaN());
    const float out_on_inf = f.apply(std::numeric_limits<float>::infinity());
    const float out_next   = f.apply(1.25f);

    TEST_ASSERT_TRUE(std::isfinite(out_on_nan));
    TEST_ASSERT_TRUE(std::isfinite(out_on_inf));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.25f, out_next);
}

// 8. Reconfiguração suave em pleno voo (MAVLink PARAM_SET sem degrau/solavanco)
void test_reconfigure_bumpless_in_flight() {
    BiquadLPF f(1000.0f, 90.0f);
    f.reset(kGravityMs2);

    // Troca corte de 90 Hz para 25 Hz em pleno voo com aceleração estacionária de 1g:
    f.reconfigure_bumpless(1000.0f, 25.0f, f.last_output());
    const float immediate_next = f.apply(kGravityMs2);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, kGravityMs2, immediate_next);
}

int main(int /*argc*/, char** /*argv*/) {
    UNITY_BEGIN();
    RUN_TEST(test_passthrough_guard_on_invalid_cutoffs);
    RUN_TEST(test_dc_unity_gain_gravity);
    RUN_TEST(test_minus_3db_at_cutoff_frequency);
    RUN_TEST(test_propeller_vibration_rejection);
    RUN_TEST(test_reset_clears_delay_registers);
    RUN_TEST(test_biquad_vec3_axes_independent);
    RUN_TEST(test_nan_and_inf_immunity);
    RUN_TEST(test_reconfigure_bumpless_in_flight);
    return UNITY_END();
}
