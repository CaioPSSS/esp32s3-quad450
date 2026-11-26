#include "pid_cascade.h"

#include <algorithm>
#include <cmath>

namespace fc {

namespace {
constexpr float kMinDtSeconds = 1e-5f;
constexpr float kMaxDtSeconds = 0.05f;
} // namespace

void RateAxisPID::configure(float kp, float ki, float kd,
                            float sample_rate_hz, float dterm_lpf_hz,
                            float integral_limit, float output_limit) noexcept {
    kp_ = (std::isfinite(kp) && kp >= 0.0f) ? kp : 0.0f;
    ki_ = (std::isfinite(ki) && ki >= 0.0f) ? ki : 0.0f;
    kd_ = (std::isfinite(kd) && kd >= 0.0f) ? kd : 0.0f;
    i_limit_ = (std::isfinite(integral_limit) && integral_limit >= 0.0f) ? integral_limit : 0.35f;
    out_limit_ = (std::isfinite(output_limit) && output_limit > 0.0f) ? output_limit : 1.0f;
    d_lpf_.configure(sample_rate_hz, dterm_lpf_hz);
    reset();
}

void RateAxisPID::update_gains_bumpless(float kp, float ki, float kd,
                                        float sample_rate_hz, float dterm_lpf_hz) noexcept {
    if (std::isfinite(kp) && kp >= 0.0f) kp_ = kp;
    if (std::isfinite(ki) && ki >= 0.0f) ki_ = ki;
    if (std::isfinite(kd) && kd >= 0.0f) kd_ = kd;
    d_lpf_.reconfigure_bumpless(sample_rate_hz, dterm_lpf_hz, d_lpf_.last_output());
    integral_ = std::clamp(integral_, -i_limit_, i_limit_);
}

void RateAxisPID::reset() noexcept {
    integral_ = 0.0f;
    prev_gyro_ = 0.0f;
    has_prev_gyro_ = false;
    d_lpf_.reset(0.0f);
    last_sp_ = 0.0f;
    last_p_ = 0.0f;
    last_i_ = 0.0f;
    last_d_ = 0.0f;
}

float RateAxisPID::update(float rate_setpoint_rads,
                          float gyro_measured_rads,
                          float dt_s,
                          bool allow_integration,
                          bool mixer_saturated) noexcept {
    if (!std::isfinite(rate_setpoint_rads) || !std::isfinite(gyro_measured_rads) ||
        !std::isfinite(dt_s) || dt_s < kMinDtSeconds) {
        return 0.0f;
    }
    const float dt = std::min(dt_s, kMaxDtSeconds);
    last_sp_ = rate_setpoint_rads;

    const float error = rate_setpoint_rads - gyro_measured_rads;
    last_p_ = kp_ * error;

    // Derivativo sobre a medição (-Kd * d(gyro)/dt) com filtro Biquad dedicado:
    // mudanças bruscas no stick do piloto NÃO causam pico derivativo!
    float raw_gyro_deriv = 0.0f;
    if (has_prev_gyro_) {
        raw_gyro_deriv = (gyro_measured_rads - prev_gyro_) / dt;
    } else {
        has_prev_gyro_ = true;
    }
    prev_gyro_ = gyro_measured_rads;
    const float filt_gyro_deriv = d_lpf_.apply(raw_gyro_deriv);
    last_d_ = -kd_ * filt_gyro_deriv;

    // Anti-windup condicional + zeramento em solo
    if (!allow_integration || ki_ <= 0.0f) {
        integral_ = 0.0f;
    } else {
        const float pre_out = last_p_ + integral_ + last_d_;
        const bool pushing_further_into_sat =
            (mixer_saturated || std::fabs(pre_out) >= out_limit_) &&
            ((pre_out > 0.0f && error > 0.0f) || (pre_out < 0.0f && error < 0.0f));

        if (!pushing_further_into_sat) {
            integral_ = std::clamp(integral_ + ki_ * error * dt, -i_limit_, i_limit_);
        }
    }
    last_i_ = integral_;

    return std::clamp(last_p_ + last_i_ + last_d_, -out_limit_, out_limit_);
}

void CascadedPID::configure_from_params(const proto::FlightParams& params,
                                        float sample_rate_hz) noexcept {
    ang_rp_kp_ = params.ang_rp_kp;
    max_tilt_rad_ = params.max_tilt_deg * kDegToRad;

    roll_pid_.configure(params.rate_rp_kp, params.rate_rp_ki, params.rate_rp_kd,
                        sample_rate_hz, params.dterm_lpf_hz, 0.35f, 1.0f);
    pitch_pid_.configure(params.rate_rp_kp, params.rate_rp_ki, params.rate_rp_kd,
                         sample_rate_hz, params.dterm_lpf_hz, 0.35f, 1.0f);
    // Yaw não usa termo D em quadricópteros (alta inércia rotacional + ruído de torque de hélice)
    yaw_pid_.configure(params.rate_yaw_kp, params.rate_yaw_ki, 0.0f,
                       sample_rate_hz, params.dterm_lpf_hz, 0.30f, 0.50f);
}

void CascadedPID::update_params_bumpless(const proto::FlightParams& params,
                                         float sample_rate_hz) noexcept {
    ang_rp_kp_ = params.ang_rp_kp;
    max_tilt_rad_ = params.max_tilt_deg * kDegToRad;

    roll_pid_.update_gains_bumpless(params.rate_rp_kp, params.rate_rp_ki, params.rate_rp_kd,
                                    sample_rate_hz, params.dterm_lpf_hz);
    pitch_pid_.update_gains_bumpless(params.rate_rp_kp, params.rate_rp_ki, params.rate_rp_kd,
                                     sample_rate_hz, params.dterm_lpf_hz);
    yaw_pid_.update_gains_bumpless(params.rate_yaw_kp, params.rate_yaw_ki, 0.0f,
                                   sample_rate_hz, params.dterm_lpf_hz);
}

void CascadedPID::reset() noexcept {
    roll_pid_.reset();
    pitch_pid_.reset();
    yaw_pid_.reset();
}

TorqueCommand CascadedPID::update_stabilize(float roll_sp_rad,
                                            float pitch_sp_rad,
                                            float yaw_rate_sp_rads,
                                            const EulerAngles& attitude,
                                            const Vec3f& gyro_filt_rads,
                                            float dt_s,
                                            bool in_air,
                                            bool mixer_saturated) noexcept {
    const float clamped_roll_sp  = std::clamp(roll_sp_rad,  -max_tilt_rad_, max_tilt_rad_);
    const float clamped_pitch_sp = std::clamp(pitch_sp_rad, -max_tilt_rad_, max_tilt_rad_);
    const float clamped_yaw_rate = std::clamp(yaw_rate_sp_rads, -max_rate_yaw_rads_, max_rate_yaw_rads_);

    // Malha externa P (Ângulo -> Velocidade Angular Desejada)
    const float roll_rate_sp = std::clamp(
        ang_rp_kp_ * (clamped_roll_sp - attitude.roll_rad),
        -max_rate_rp_rads_, max_rate_rp_rads_);

    const float pitch_rate_sp = std::clamp(
        ang_rp_kp_ * (clamped_pitch_sp - attitude.pitch_rad),
        -max_rate_rp_rads_, max_rate_rp_rads_);

    // Malha interna PID (Velocidade Angular -> Momento Normalizado FRD)
    TorqueCommand out{};
    out.roll  = roll_pid_.update(roll_rate_sp,  gyro_filt_rads.x, dt_s, in_air, mixer_saturated);
    out.pitch = pitch_pid_.update(pitch_rate_sp, gyro_filt_rads.y, dt_s, in_air, mixer_saturated);
    out.yaw   = yaw_pid_.update(clamped_yaw_rate, gyro_filt_rads.z, dt_s, in_air, mixer_saturated);
    return out;
}

} // namespace fc
