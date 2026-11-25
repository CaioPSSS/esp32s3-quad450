#include "mahony_ahrs.h"

#include <algorithm>
#include <cmath>

namespace fc {

namespace {
constexpr float kMinAccelGateG   = 0.80f;
constexpr float kMaxAccelGateG   = 1.20f;
constexpr float kMaxIntegralBias = 0.20f; // ~11.5 deg/s anti-windup por eixo
constexpr float kMaxDtSeconds    = 0.05f; // Proteção contra saltos grandes de tempo

inline bool is_finite_vec3(const Vec3f& v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline float wrap_pi(float angle_rad) noexcept {
    while (angle_rad > kPi) {
        angle_rad -= 2.0f * kPi;
    }
    while (angle_rad < -kPi) {
        angle_rad += 2.0f * kPi;
    }
    return angle_rad;
}

inline Quatf euler_to_quat(float roll, float pitch, float yaw) noexcept {
    const float cr = std::cos(0.5f * roll);
    const float sr = std::sin(0.5f * roll);
    const float cp = std::cos(0.5f * pitch);
    const float sp = std::sin(0.5f * pitch);
    const float cy = std::cos(0.5f * yaw);
    const float sy = std::sin(0.5f * yaw);

    Quatf q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}
} // namespace

void MahonyAHRS::configure(float kp, float ki) noexcept {
    if (std::isfinite(kp) && kp >= 0.0f) {
        kp_ = kp;
    }
    if (std::isfinite(ki) && ki >= 0.0f) {
        ki_ = ki;
    }
}

void MahonyAHRS::reset() noexcept {
    q_ = {1.0f, 0.0f, 0.0f, 0.0f};
    integral_fb_ = {0.0f, 0.0f, 0.0f};
    euler_ = {};
    earth_accel_up_ms2_ = 0.0f;
    tilt_cos_ = 1.0f;
    accel_gate_open_ = false;
}

bool MahonyAHRS::align_to_gravity(const Vec3f& accel_ms2, float initial_yaw_rad) noexcept {
    if (!is_finite_vec3(accel_ms2) || !std::isfinite(initial_yaw_rad)) {
        return false;
    }
    const float norm = accel_ms2.norm();
    if (norm < 0.5f * kGravityMs2 || norm > 1.5f * kGravityMs2) {
        return false;
    }

    // Em repouso FRD: f_b = -g_b => g_unit = -accel / ||accel|| = [-sin(pitch), sin(roll)*cos(pitch), cos(roll)*cos(pitch)]
    const Vec3f g_unit = accel_ms2 * (-1.0f / norm);
    const float sin_pitch = std::clamp(-g_unit.x, -1.0f, 1.0f);
    const float pitch = std::asin(sin_pitch);
    const float roll  = std::atan2(g_unit.y, g_unit.z);

    q_ = euler_to_quat(roll, pitch, wrap_pi(initial_yaw_rad));
    normalize_quaternion();
    integral_fb_ = {0.0f, 0.0f, 0.0f};
    update_derived_quantities(accel_ms2);
    return true;
}

void MahonyAHRS::update(const Vec3f& gyro_rads, const Vec3f& accel_ms2, float dt_s) noexcept {
    update_with_heading(gyro_rads, accel_ms2, 0.0f, 0.0f, dt_s);
}

void MahonyAHRS::update_with_heading(const Vec3f& gyro_rads,
                                     const Vec3f& accel_ms2,
                                     float measured_yaw_rad,
                                     float kp_yaw,
                                     float dt_s) noexcept {
    if (!is_finite_vec3(gyro_rads) || !is_finite_vec3(accel_ms2) ||
        !std::isfinite(dt_s) || dt_s <= 0.0f) {
        return;
    }
    const float dt = std::min(dt_s, kMaxDtSeconds);

    // 1. Avalia porta de gravidade (0.80g .. 1.20g)
    const float acc_norm = accel_ms2.norm();
    const float min_acc = kMinAccelGateG * kGravityMs2;
    const float max_acc = kMaxAccelGateG * kGravityMs2;
    accel_gate_open_ = (acc_norm >= min_acc && acc_norm <= max_acc);

    Vec3f err_total{0.0f, 0.0f, 0.0f};

    if (accel_gate_open_) {
        // Direção unitária medida da gravidade (+Z para baixo no corpo FRD): g_meas = -f_b / ||f_b||
        const Vec3f g_meas = accel_ms2 * (-1.0f / acc_norm);

        // Direção unitária estimada da gravidade a partir da 3ª linha de R_b_to_e(q)
        const Vec3f g_est{
            2.0f * (q_.x * q_.z - q_.w * q_.y),
            2.0f * (q_.y * q_.z + q_.w * q_.x),
            1.0f - 2.0f * (q_.x * q_.x + q_.y * q_.y)
        };

        // Produto vetorial g_meas x g_est gera rotação corretiva exata no corpo
        const Vec3f err_grav = g_meas.cross(g_est);
        err_total = err_total + (err_grav * kp_);

        // O termo integral aprende apenas quando a gravidade é confiável
        if (ki_ > 0.0f) {
            integral_fb_.x = std::clamp(integral_fb_.x + ki_ * err_grav.x * dt, -kMaxIntegralBias, kMaxIntegralBias);
            integral_fb_.y = std::clamp(integral_fb_.y + ki_ * err_grav.y * dt, -kMaxIntegralBias, kMaxIntegralBias);
            integral_fb_.z = std::clamp(integral_fb_.z + ki_ * err_grav.z * dt, -kMaxIntegralBias, kMaxIntegralBias);
        }
    }

    // 2. Fusão opcional de proa (Fase 2: Bússola QMC5883L / GPS Course)
    if (kp_yaw > 0.0f && std::isfinite(measured_yaw_rad) && std::isfinite(kp_yaw)) {
        const float current_yaw = std::atan2(2.0f * (q_.w * q_.z + q_.x * q_.y),
                                             1.0f - 2.0f * (q_.y * q_.y + q_.z * q_.z));
        const float yaw_err_ned = wrap_pi(measured_yaw_rad - current_yaw);

        // Projeta vetor de erro vertical da Terra [0, 0, yaw_err_ned] para o corpo via R_b_to_e^T
        const Vec3f down_in_body{
            2.0f * (q_.x * q_.z - q_.w * q_.y),
            2.0f * (q_.y * q_.z + q_.w * q_.x),
            1.0f - 2.0f * (q_.x * q_.x + q_.y * q_.y)
        };
        err_total = err_total + (down_in_body * (kp_yaw * yaw_err_ned));
    }

    integrate_and_update_outputs(gyro_rads, accel_ms2, err_total, accel_gate_open_, dt);
}

void MahonyAHRS::integrate_and_update_outputs(const Vec3f& gyro_rads,
                                              const Vec3f& accel_ms2,
                                              const Vec3f& err_body,
                                              bool /*allow_integral*/,
                                              float dt_s) noexcept {
    // Taxa angular corrigida: omega = gyro + integral_fb + err_proporcional
    const Vec3f omega = gyro_rads + integral_fb_ + err_body;

    // Cinemática de quaternion de Hamilton: q_dot = 0.5 * q (x) [0, omega]
    const float half_dt = 0.5f * dt_s;
    const Quatf q_prev = q_;

    q_.w += (-q_prev.x * omega.x - q_prev.y * omega.y - q_prev.z * omega.z) * half_dt;
    q_.x += ( q_prev.w * omega.x + q_prev.y * omega.z - q_prev.z * omega.y) * half_dt;
    q_.y += ( q_prev.w * omega.y - q_prev.x * omega.z + q_prev.z * omega.x) * half_dt;
    q_.z += ( q_prev.w * omega.z + q_prev.x * omega.y - q_prev.y * omega.x) * half_dt;

    normalize_quaternion();
    update_derived_quantities(accel_ms2);
}

void MahonyAHRS::normalize_quaternion() noexcept {
    const float norm_sq = q_.w * q_.w + q_.x * q_.x + q_.y * q_.y + q_.z * q_.z;
    if (!std::isfinite(norm_sq) || norm_sq < 1e-12f) {
        q_ = {1.0f, 0.0f, 0.0f, 0.0f};
        return;
    }
    const float inv_norm = 1.0f / std::sqrt(norm_sq);
    q_.w *= inv_norm;
    q_.x *= inv_norm;
    q_.y *= inv_norm;
    q_.z *= inv_norm;

    // Convenção canônica de hemisfério escalar positivo (w >= 0)
    if (q_.w < 0.0f) {
        q_.w = -q_.w;
        q_.x = -q_.x;
        q_.y = -q_.y;
        q_.z = -q_.z;
    }
}

void MahonyAHRS::update_derived_quantities(const Vec3f& accel_ms2) noexcept {
    // 3ª linha de R_b_to_e(q)
    const float r31 = 2.0f * (q_.x * q_.z - q_.w * q_.y);
    const float r32 = 2.0f * (q_.y * q_.z + q_.w * q_.x);
    const float r33 = 1.0f - 2.0f * (q_.x * q_.x + q_.y * q_.y);

    tilt_cos_ = std::clamp(r33, -1.0f, 1.0f);

    // Extração ZYX Euler segura contra erro numérico em +-90 deg
    const float sin_pitch = std::clamp(-r31, -1.0f, 1.0f);
    euler_.pitch_rad = std::asin(sin_pitch);
    euler_.roll_rad  = std::atan2(r32, r33);
    euler_.yaw_rad   = std::atan2(2.0f * (q_.w * q_.z + q_.x * q_.y),
                                  1.0f - 2.0f * (q_.y * q_.y + q_.z * q_.z));

    // Projeção vertical da força específica na Terra (eixo Down NED = linha 3 de R_b_to_e . f_b):
    //   a_kin_down = (R_b_to_e * f_b)_z + g
    //   earth_accel_up = -a_kin_down = -(r31*fx + r32*fy + r33*fz + g)
    const float f_down_ned = r31 * accel_ms2.x + r32 * accel_ms2.y + r33 * accel_ms2.z;
    earth_accel_up_ms2_ = -(f_down_ned + kGravityMs2);
}

float MahonyAHRS::tilt_angle_rad() const noexcept {
    return std::acos(std::clamp(tilt_cos_, -1.0f, 1.0f));
}

} // namespace fc
