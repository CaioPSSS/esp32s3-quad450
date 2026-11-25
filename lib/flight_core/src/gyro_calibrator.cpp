#include "gyro_calibrator.h"

#include <cmath>

namespace fc {

namespace {
constexpr uint32_t kCheckWindowSubstep = 100; // Checa movimento a cada 100 ms (@ 1 kHz)
constexpr float kMaxReasonableBiasRads = 0.35f; // ~20 deg/s: acima disso a IMU está com defeito

inline bool finite3(const Vec3f& v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
} // namespace

void GyroCalibrator::reset() noexcept {
    clear_window();
    motion_resets_ = 0;
    gyro_bias_ = {};
    calibrated_ = false;
}

void GyroCalibrator::clear_window() noexcept {
    count_ = 0;
    gyro_mean_ = {};
    gyro_m2_ = {};
    accel_mean_ = {};
}

bool GyroCalibrator::push_sample(const Vec3f& gyro_rads, const Vec3f& accel_ms2) noexcept {
    if (calibrated_) {
        return true;
    }
    if (!finite3(gyro_rads) || !finite3(accel_ms2)) {
        clear_window();
        return false;
    }

    ++count_;
    const float inv_n = 1.0f / static_cast<float>(count_);

    // Atualização numericamente estável de Welford para média e variância do giroscópio
    const Vec3f delta = gyro_rads - gyro_mean_;
    gyro_mean_ = gyro_mean_ + (delta * inv_n);
    const Vec3f delta2 = gyro_rads - gyro_mean_;
    gyro_m2_.x += delta.x * delta2.x;
    gyro_m2_.y += delta.y * delta2.y;
    gyro_m2_.z += delta.z * delta2.z;

    // Média online do acelerômetro para alinhamento inicial de Roll/Pitch
    accel_mean_ = accel_mean_ + ((accel_ms2 - accel_mean_) * inv_n);

    // Verifica periodicamente se o drone balançou (ex.: mão soltando o conector XT60)
    if (count_ >= kCheckWindowSubstep && (count_ % kCheckWindowSubstep == 0u || count_ >= required_samples_)) {
        const float denom = static_cast<float>(count_ - 1u);
        const float var_x = gyro_m2_.x / denom;
        const float var_y = gyro_m2_.y / denom;
        const float var_z = gyro_m2_.z / denom;

        if (var_x > max_stddev_sq_ || var_y > max_stddev_sq_ || var_z > max_stddev_sq_) {
            ++motion_resets_;
            clear_window();
            return false;
        }
    }

    if (count_ >= required_samples_) {
        if (std::fabs(gyro_mean_.x) > kMaxReasonableBiasRads ||
            std::fabs(gyro_mean_.y) > kMaxReasonableBiasRads ||
            std::fabs(gyro_mean_.z) > kMaxReasonableBiasRads) {
            ++motion_resets_;
            clear_window();
            return false;
        }
        gyro_bias_ = gyro_mean_;
        calibrated_ = true;
        return true;
    }

    return false;
}

} // namespace fc
