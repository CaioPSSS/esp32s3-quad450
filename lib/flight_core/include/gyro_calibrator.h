#pragma once

#include <cstdint>
#include "fc_types.h"

namespace fc {

/**
 * Calibrador Estacionário de Giroscópio e Gravidade Inicial com Detecção de Movimento
 * (Algoritmo Online de Welford em O(1) de memória).
 *
 * Resolve um problema clássico real: quando o piloto pluga o conector XT60 da bateria,
 * o chassi balança na mão por 0.5 a 1.5 s. Se o calibrador perceber desvio-padrão
 * acima de `max_gyro_stddev_rads`, ele descarta a janela suja e recomeça sozinho!
 */
class GyroCalibrator {
public:
    explicit GyroCalibrator(uint32_t required_samples = 2000,
                            float max_gyro_stddev_rads = 0.035f) noexcept
        : required_samples_(required_samples),
          max_stddev_sq_(max_gyro_stddev_rads * max_gyro_stddev_rads) {}

    void reset() noexcept;

    /**
     * Alimenta uma amostra de IMU (tipicamente a 1000 Hz -> 2000 amostras = 2.0 s).
     * Retorna `true` assim que a calibração estacionária terminar com sucesso.
     */
    bool push_sample(const Vec3f& gyro_rads, const Vec3f& accel_ms2) noexcept;

    bool is_calibrated() const noexcept { return calibrated_; }
    uint32_t progress_samples() const noexcept { return count_; }
    uint32_t required_samples() const noexcept { return required_samples_; }
    uint32_t motion_resets() const noexcept { return motion_resets_; }

    const Vec3f& gyro_bias_rads() const noexcept { return gyro_bias_; }
    const Vec3f& mean_accel_ms2() const noexcept { return accel_mean_; }

private:
    void clear_window() noexcept;

    uint32_t required_samples_ = 2000;
    float max_stddev_sq_ = 0.035f * 0.035f;

    uint32_t count_ = 0;
    uint32_t motion_resets_ = 0;
    Vec3f gyro_mean_{};
    Vec3f gyro_m2_{};
    Vec3f accel_mean_{};

    Vec3f gyro_bias_{};
    bool calibrated_ = false;
};

} // namespace fc
