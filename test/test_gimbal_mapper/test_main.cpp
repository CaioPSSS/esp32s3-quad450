#include <unity.h>

#include "gimbal_mapper.h"

using namespace fc;

void setUp() {}
void tearDown() {}

// 1. Curso parcial real de gimbal RC (ex.: 780..3220) mapeia 0..1000 c/ zona morta segura p/ Pre-Arm
void test_partial_throw_rc_gimbal_normalization() {
    GimbalCalibrationData cal{};
    cal.throttle = {780, 780, 3220, false};
    cal.roll     = {820, 1950, 3180, false}; // Centro assimétrico realista!
    GimbalMapper mapper(cal);

    // Levemente acima do batente inferior (ex.: 810 raw) ainda DEVE cravar 0 permil p/ Pre-Arm!
    const auto s_zero = mapper.map_raw(810, 2048, 2048, 1960);
    TEST_ASSERT_EQUAL_UINT16(0u, s_zero.thr_permil);
    TEST_ASSERT_EQUAL_INT16(0, s_zero.roll_permil); // Dentro da deadzone central!

    // Batente superior crava 1000
    const auto s_full = mapper.map_raw(3220, 2048, 2048, 3180);
    TEST_ASSERT_EQUAL_UINT16(1000u, s_full.thr_permil);
    TEST_ASSERT_EQUAL_INT16(1000, s_full.roll_permil);

    // Batente esquerdo crava -1000 mesmo com centro assimétrico
    const auto s_left = mapper.map_raw(780, 2048, 2048, 820);
    TEST_ASSERT_EQUAL_INT16(-1000, s_left.roll_permil);
}

// 2. Assistente de calibração em 2 etapas detecta automaticamente potenciômetro de Throttle invertido!
void test_interactive_calibration_wizard_auto_detects_inverted_throttle() {
    GimbalMapper mapper;
    mapper.begin_calibration();

    // Se o piloto não mexeu os sticks e apertou finalizar -> DEVE recusar!
    mapper.record_min_max_sample(2000, 2000, 2000, 2000);
    TEST_ASSERT_FALSE(mapper.finalize_calibration(2000, 2000, 2000, 2000));

    // Agora move todos os 4 eixos nos extremos (com Throttle invertido: zero físico = 3500, full = 600!)
    mapper.begin_calibration();
    mapper.record_min_max_sample(3500, 700,  750,  800);
    mapper.record_min_max_sample(600,  3300, 3250, 3400);

    // Etapa 2: solta sticks no centro e deixa Throttle abaixado (raw = 3490 perto de 3500!)
    TEST_ASSERT_TRUE(mapper.finalize_calibration(3490, 2010, 1990, 2100));
    TEST_ASSERT_TRUE(mapper.calibration().throttle.inverted);

    // Verifica que Throttle no batente inferior (3500) dá 0 permil e em cima (600) dá 1000 permil!
    TEST_ASSERT_EQUAL_UINT16(0u, mapper.map_raw(3495, 2010, 1990, 2100).thr_permil);
    TEST_ASSERT_EQUAL_UINT16(1000u, mapper.map_raw(600, 2010, 1990, 2100).thr_permil);
}

int main(int /*argc*/, char** /*argv*/) {
    UNITY_BEGIN();
    RUN_TEST(test_partial_throw_rc_gimbal_normalization);
    RUN_TEST(test_interactive_calibration_wizard_auto_detects_inverted_throttle);
    return UNITY_END();
}
