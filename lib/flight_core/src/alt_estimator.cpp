#include "alt_estimator.h"

#include <algorithm>
#include <cmath>

namespace fc {

namespace {
// Ganhos do observador complementar de 3 estados (polos amortecidos em wn ~= 1.6 rad/s):
//   kz = 3 * zeta * wn ~= 3.3,  kv = 1 + 2*zeta^2 * wn^2 ~= 2.6,  kb = 0.25
constexpr float kObsGainZ    = 3.20f;
constexpr float kObsGainVel  = 2.40f;
constexpr float kObsGainBias = 0.22f;
constexpr float kMaxBaroResidM   = 4.0f;  // Limita rajadas súbitas de pressão
constexpr float kMaxAccelBiasMs2 = 1.50f; // +-0.15g máx de deriva térmica estimada

constexpr float kStickCenterLow  = 0.42f;
constexpr float kStickCenterHigh = 0.58f;
constexpr float kMinAltHoldThr   = 0.12f;
constexpr float kMaxAltHoldThr   = 0.90f;
constexpr float kMaxVelIntegral  = 0.24f;
} // namespace

// ============================================================================
// VerticalEstimator
// ============================================================================

void VerticalEstimator::reset() noexcept {
    alt_m_ = 0.0f;
    vel_up_ms_ = 0.0f;
    accel_bias_ms2_ = 0.0f;
    last_unbiased_acc_ms2_ = 0.0f;
    ground_baro_zero_m_ = 0.0f;
    last_baro_us_ = 0;
    initialized_ = false;
}

void VerticalEstimator::predict_1khz(float earth_accel_up_ms2, bool motors_armed, float dt_s) noexcept {
    if (!std::isfinite(earth_accel_up_ms2) || !std::isfinite(dt_s) || dt_s <= 0.0f) {
        return;
    }
    const float dt = std::min(dt_s, 0.02f);

    if (!initialized_) {
        last_unbiased_acc_ms2_ = 0.0f;
        return;
    }

    const float acc_corr = std::clamp(earth_accel_up_ms2 - accel_bias_ms2_, -25.0f, 25.0f);
    last_unbiased_acc_ms2_ = acc_corr;

    alt_m_    += vel_up_ms_ * dt + 0.5f * acc_corr * dt * dt;
    vel_up_ms_ = std::clamp(vel_up_ms_ + acc_corr * dt, -15.0f, 15.0f);

    // Enquanto desarmado no chão, amortece velocidade residual p/ cravar 0.00 m antes de decolar
    if (!motors_armed) {
        alt_m_ *= 0.98f;
        vel_up_ms_ *= 0.95f;
    }
}

void VerticalEstimator::correct_baro(const BaroSample& baro, bool motors_armed) noexcept {
    if (!baro.healthy || !std::isfinite(baro.altitude_m)) {
        return;
    }
    if (initialized_ && baro.timestamp_us == last_baro_us_) {
        return; // Mesma amostra já consumida num ciclo anterior de 1 kHz
    }

    if (!initialized_) {
        ground_baro_zero_m_ = baro.altitude_m;
        alt_m_ = 0.0f;
        vel_up_ms_ = 0.0f;
        accel_bias_ms2_ = 0.0f;
        last_baro_us_ = baro.timestamp_us;
        initialized_ = true;
        return;
    }

    const uint32_t raw_dt_us = (baro.timestamp_us >= last_baro_us_)
                                   ? (baro.timestamp_us - last_baro_us_)
                                   : 25000u;
    last_baro_us_ = baro.timestamp_us;
    const float dt_baro = std::clamp(static_cast<float>(raw_dt_us) * 1.0e-6f, 0.005f, 0.25f);

    // Enquanto desarmado no solo, acompanha lentamente a pressão local p/ manter Home = 0.00 m
    if (!motors_armed) {
        ground_baro_zero_m_ += 0.08f * (baro.altitude_m - ground_baro_zero_m_);
    }

    const float z_meas = baro.altitude_m - ground_baro_zero_m_;
    const float err_z  = std::clamp(z_meas - alt_m_, -kMaxBaroResidM, kMaxBaroResidM);

    alt_m_     += kObsGainZ * err_z * dt_baro;
    vel_up_ms_  = std::clamp(vel_up_ms_ + kObsGainVel * err_z * dt_baro, -15.0f, 15.0f);
    accel_bias_ms2_ = std::clamp(accel_bias_ms2_ - kObsGainBias * err_z * dt_baro,
                                 -kMaxAccelBiasMs2, kMaxAccelBiasMs2);
}

bool VerticalEstimator::is_baro_healthy(uint32_t now_us, uint32_t timeout_us) const noexcept {
    if (!initialized_) {
        return false;
    }
    const uint32_t age_us = (now_us >= last_baro_us_) ? (now_us - last_baro_us_) : 0u;
    return age_us <= timeout_us;
}

// ============================================================================
// AltHoldController
// ============================================================================

void AltHoldController::configure(const proto::FlightParams& params) noexcept {
    kp_alt_ = params.alt_hold_kp;
    kp_vz_  = params.vel_z_kp;
    ki_vz_  = params.vel_z_ki;
    kd_vz_  = params.vel_z_kd;
    base_hover_thr_ = std::clamp(params.fs_hover_thr, 0.22f, 0.55f);
    if (!active_hold_ && !active_failsafe_) {
        hover_thr_est_ = std::clamp(hover_thr_est_, 0.22f, 0.55f);
    }
}

void AltHoldController::reset(float base_hover_thr) noexcept {
    base_hover_thr_     = std::clamp(base_hover_thr, 0.22f, 0.55f);
    hover_thr_est_      = base_hover_thr_;
    target_alt_m_       = 0.0f;
    vel_i_              = 0.0f;
    active_hold_        = false;
    active_failsafe_    = false;
    touchdown_timer_s_  = 0.0f;
    touchdown_detected_ = false;
}

void AltHoldController::observe_manual_flight(float manual_throttle,
                                              bool in_air,
                                              float tilt_cos,
                                              const VerticalEstimator& est,
                                              float dt_s) noexcept {
    active_hold_ = false;
    active_failsafe_ = false;
    touchdown_timer_s_ = 0.0f;
    touchdown_detected_ = false;

    target_alt_m_ = est.altitude_m();

    // Aprende suavemente o throttle real de pairado quando quase nivelado e sem acelerar verticalmente
    if (in_air && tilt_cos > 0.94f &&
        std::fabs(est.climb_rate_ms()) < 0.35f &&
        std::fabs(est.unbiased_accel_up_ms2()) < 1.2f &&
        manual_throttle > 0.20f && manual_throttle < 0.65f) {
        const float alpha = std::clamp(dt_s * 0.8f, 0.0f, 0.05f);
        hover_thr_est_ = std::clamp(hover_thr_est_ + alpha * (manual_throttle - hover_thr_est_),
                                    0.22f, 0.58f);
    }

    if (in_air) {
        // Pré-carrega o integrador em voo p/ transição STABILIZE -> ALT_HOLD sem degrau!
        vel_i_ = std::clamp(manual_throttle - hover_thr_est_, -kMaxVelIntegral, kMaxVelIntegral);
    } else {
        // No solo (DISARMED / ARMED_IDLE), mantém integrador zerado p/ decolagem imediata e limpa!
        vel_i_ = 0.0f;
    }
}

float AltHoldController::compute_velocity_loop(float target_vz_ms,
                                               const VerticalEstimator& est,
                                               float dt_s) noexcept {
    const float err_vz = target_vz_ms - est.climb_rate_ms();
    const float p_term = kp_vz_ * err_vz;
    const float d_term = -kd_vz_ * est.unbiased_accel_up_ms2();

    const float pre_cmd = hover_thr_est_ + p_term + vel_i_ + d_term;
    const bool sat_hi = (pre_cmd >= kMaxAltHoldThr && err_vz > 0.0f);
    const bool sat_lo = (pre_cmd <= kMinAltHoldThr && err_vz < 0.0f);
    if (!sat_hi && !sat_lo && ki_vz_ > 0.0f) {
        vel_i_ = std::clamp(vel_i_ + ki_vz_ * err_vz * dt_s, -kMaxVelIntegral, kMaxVelIntegral);
    }

    return std::clamp(hover_thr_est_ + p_term + vel_i_ + d_term, kMinAltHoldThr, kMaxAltHoldThr);
}

float AltHoldController::update_alt_hold(float stick_thr_0to1,
                                         float max_climb_rate_ms,
                                         const VerticalEstimator& est,
                                         float dt_s) noexcept {
    active_failsafe_ = false;
    touchdown_timer_s_ = 0.0f;
    touchdown_detected_ = false;

    if (!active_hold_) {
        active_hold_ = true;
        target_alt_m_ = est.altitude_m();
    }

    const float stick = std::clamp(stick_thr_0to1, 0.0f, 1.0f);
    const float max_vz = std::clamp(max_climb_rate_ms, 0.4f, 3.0f);
    float desired_vz_ms = 0.0f;

    if (stick > kStickCenterHigh) {
        const float norm_up = (stick - kStickCenterHigh) / (1.0f - kStickCenterHigh);
        desired_vz_ms = norm_up * max_vz;
        // Avança o alvo com folga limitada ("leash") p/ travar macio ao soltar o stick
        target_alt_m_ = std::clamp(target_alt_m_ + desired_vz_ms * dt_s,
                                   est.altitude_m() - 0.5f,
                                   est.altitude_m() + 1.2f);
    } else if (stick < kStickCenterLow) {
        const float norm_dn = (kStickCenterLow - stick) / kStickCenterLow;
        desired_vz_ms = -norm_dn * std::min(max_vz, 1.6f);
        target_alt_m_ = std::clamp(target_alt_m_ + desired_vz_ms * dt_s,
                                   est.altitude_m() - 1.2f,
                                   est.altitude_m() + 0.5f);
    } else {
        // Stick estacionado na zona central [42% .. 58%]: trava altitude exata!
        desired_vz_ms = std::clamp(kp_alt_ * (target_alt_m_ - est.altitude_m()),
                                   -1.4f, +1.4f);
    }

    return compute_velocity_loop(desired_vz_ms, est, dt_s);
}

float AltHoldController::update_failsafe_descent(const VerticalEstimator& est,
                                                 float failsafe_elapsed_s,
                                                 float dt_s) noexcept {
    active_hold_ = false;
    if (!active_failsafe_) {
        active_failsafe_ = true;
        touchdown_timer_s_ = 0.0f;
        touchdown_detected_ = false;
    }

    // Descida controlada: -0.65 m/s em altitude e *flare* macio (-0.35 m/s) abaixo de 1.5 m AGL!
    const float target_descent_ms = (est.altitude_m() > 1.5f) ? -0.65f : -0.35f;
    const float cmd_thr = compute_velocity_loop(target_descent_ms, est, dt_s);

    // Detector físico de toque no solo:
    // Quando o drone apoia na grama tentando descer a -0.35 m/s, `vel_up_ms_` fica ~0.0 m/s
    // e o integrador `vel_i_` desce o comando de motor bem abaixo do hover!
    const bool thr_dropped = (cmd_thr <= hover_thr_est_ * 0.72f);
    const bool vertical_still = (std::fabs(est.climb_rate_ms()) < 0.20f) &&
                                (std::fabs(est.unbiased_accel_up_ms2()) < 1.5f);

    if (failsafe_elapsed_s > 0.8f && thr_dropped && vertical_still) {
        touchdown_timer_s_ += dt_s;
        if (touchdown_timer_s_ >= 0.9f) {
            touchdown_detected_ = true;
        }
    } else {
        touchdown_timer_s_ = std::max(0.0f, touchdown_timer_s_ - 2.0f * dt_s);
    }

    return cmd_thr;
}

} // namespace fc
