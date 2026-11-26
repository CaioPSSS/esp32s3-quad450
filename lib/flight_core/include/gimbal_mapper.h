#pragma once

#include <cstdint>

namespace fc {

struct AxisCalibration {
    uint16_t min_raw    = 400;   // Típico de gimbal RC (~60° de curso num pot de 270°)
    uint16_t center_raw = 2048;
    uint16_t max_raw    = 3700;
    bool     inverted   = false;
};

struct GimbalCalibrationData {
    uint32_t magic = 0x4743414Cu; // "GCAL"
    AxisCalibration throttle{400, 400, 3700, false};
    AxisCalibration yaw     {400, 2048, 3700, false};
    AxisCalibration pitch   {400, 2048, 3700, false};
    AxisCalibration roll    {400, 2048, 3700, false};
};

struct NormalizedSticks {
    uint16_t thr_permil   = 0; // [0 .. 1000]
    int16_t  yaw_permil   = 0; // [-1000 .. +1000]
    int16_t  pitch_permil = 0; // [-1000 .. +1000]
    int16_t  roll_permil  = 0; // [-1000 .. +1000]
};

/**
 * Mapeador e Calibrador de Gimbals Potenciométricos RC (12-bit ADC: 0..4095).
 *
 * Trata automaticamente:
 * - Curso parcial típico de gimbals RC (ex.: 750..3250 em vez de 0..4095)
 * - Assimetria entre metade inferior (min..center) e superior (center..max)
 * - Zona morta de segurança na base do Throttle (garante 0 permil firme p/ Pre-Arm!)
 * - Zona morta central nos 3 eixos com mola (Roll, Pitch, Yaw)
 * - Assistente de calibração interativo em 2 passos (extremos + centros)
 */
class GimbalMapper {
public:
    GimbalMapper() = default;
    explicit GimbalMapper(const GimbalCalibrationData& cal) : cal_(cal) {}

    void set_calibration(const GimbalCalibrationData& cal) noexcept { cal_ = cal; }
    const GimbalCalibrationData& calibration() const noexcept { return cal_; }

    NormalizedSticks map_raw(uint16_t raw_thr,
                             uint16_t raw_yaw,
                             uint16_t raw_pitch,
                             uint16_t raw_roll) const noexcept;

    // --- Assistente de Calibração em 2 Etapas ---
    void begin_calibration() noexcept;
    void record_min_max_sample(uint16_t raw_thr, uint16_t raw_yaw,
                               uint16_t raw_pitch, uint16_t raw_roll) noexcept;
    bool finalize_calibration(uint16_t rest_thr_zero, uint16_t center_yaw,
                              uint16_t center_pitch, uint16_t center_roll) noexcept;

    static uint16_t map_unipolar_permil(uint16_t raw, const AxisCalibration& ax,
                                        uint16_t bottom_deadzone_permil = 25) noexcept;
    static int16_t  map_bipolar_permil(uint16_t raw, const AxisCalibration& ax,
                                       int16_t center_deadzone_permil = 20) noexcept;

private:
    GimbalCalibrationData cal_{};
    uint16_t min_seen_[4] = {4095, 4095, 4095, 4095};
    uint16_t max_seen_[4] = {0, 0, 0, 0};
};

} // namespace fc
