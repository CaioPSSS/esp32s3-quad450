#include "safety_fsm.h"

#include <algorithm>
#include <cmath>

namespace fc {

void SafetyStateMachine::configure(const SafetyConfig& cfg) noexcept {
    cfg_ = cfg;
    max_arm_tilt_cos_ = std::cos(cfg_.max_arm_tilt_deg * kDegToRad);
    crash_flip_cos_   = std::cos(cfg_.crash_flip_deg * kDegToRad);
}

void SafetyStateMachine::reset() noexcept {
    state_ = SystemState::BOOT_CALIBRATING;
    prearm_flags_ = PREARM_OK;
    observed_switch_disarmed_ = false;
    prev_arm_switch_ = false;
    state_elapsed_s_ = 0.0f;
    landed_idle_timer_s_ = 0.0f;
    last_seen_rc_us_ = 0;
    fs_recovery_pkt_cnt_ = 0;
}

void SafetyStateMachine::set_bench_diagnostic_mode(bool active) noexcept {
    if (active) {
        if (state_ == SystemState::DISARMED || state_ == SystemState::BOOT_CALIBRATING) {
            state_ = SystemState::BENCH_DIAGNOSTIC;
            state_elapsed_s_ = 0.0f;
        }
    } else if (state_ == SystemState::BENCH_DIAGNOSTIC) {
        state_ = SystemState::DISARMED;
        observed_switch_disarmed_ = false;
        state_elapsed_s_ = 0.0f;
    }
}

uint16_t SafetyStateMachine::evaluate_prearm_flags(const SafetyInputs& in,
                                                   bool rc_fresh_prearm,
                                                   bool imu_fresh) const noexcept {
    uint16_t flags = PREARM_OK;

    if (!imu_fresh) {
        flags |= ERR_IMU_UNHEALTHY;
    }
    if (!in.gyro_calibrated) {
        flags |= ERR_GYRO_UNCALIBRATED;
    }
    if (!rc_fresh_prearm) {
        flags |= ERR_RC_LINK_LOST;
    }
    if (!observed_switch_disarmed_) {
        flags |= ERR_ARM_SWITCH_BOOT_ON;
    }
    if (in.rc.throttle > cfg_.max_arm_throttle) {
        flags |= ERR_THROTTLE_NOT_ZERO;
    }
    if (in.tilt_cos < max_arm_tilt_cos_) {
        flags |= ERR_TILT_TOO_HIGH;
    }
    if (!in.allow_no_battery_bench && in.vbat_v < cfg_.min_prearm_vbat_v) {
        flags |= ERR_BATTERY_LOW;
    }
    if (state_ == SystemState::BENCH_DIAGNOSTIC) {
        flags |= ERR_BENCH_MODE_ACTIVE;
    }

    return flags;
}

SafetyDecision SafetyStateMachine::update(const SafetyInputs& in, float dt_s) noexcept {
    const float dt = (std::isfinite(dt_s) && dt_s > 0.0f) ? std::min(dt_s, 0.1f) : 0.0f;
    state_elapsed_s_ += dt;

    const uint32_t imu_age_us = (in.now_us >= in.last_imu_us) ? (in.now_us - in.last_imu_us) : 0u;
    const bool imu_fresh = in.imu_healthy && (imu_age_us <= cfg_.imu_timeout_us);

    const uint32_t rc_age_us = (in.now_us >= in.rc.timestamp_us) ? (in.now_us - in.rc.timestamp_us) : cfg_.flight_rc_timeout_us + 1u;
    const bool rc_fresh_prearm = in.rc.link_alive && (rc_age_us <= cfg_.prearm_rc_max_age_us);
    const bool rc_fresh_flight = in.rc.link_alive && (rc_age_us <= cfg_.flight_rc_timeout_us);

    const bool is_new_rc_pkt = in.rc.link_alive && (in.rc.timestamp_us != last_seen_rc_us_);
    const uint32_t prev_pkt_gap_us = (last_seen_rc_us_ != 0u && in.rc.timestamp_us >= last_seen_rc_us_)
                                         ? (in.rc.timestamp_us - last_seen_rc_us_)
                                         : (cfg_.prearm_rc_max_age_us + 1u);
    if (is_new_rc_pkt) {
        if (prev_pkt_gap_us <= cfg_.prearm_rc_max_age_us) {
            if (fs_recovery_pkt_cnt_ < 255u) {
                ++fs_recovery_pkt_cnt_;
            }
        } else {
            fs_recovery_pkt_cnt_ = 1u;
        }
        last_seen_rc_us_ = in.rc.timestamp_us;
    } else if (!rc_fresh_prearm) {
        fs_recovery_pkt_cnt_ = 0u;
    }

    // Registra assim que enxergamos o controle vivo com a chave ARM desligada (OFF)
    if (rc_fresh_prearm && !in.rc.arm_switch) {
        observed_switch_disarmed_ = true;
    }

    const bool arm_rising_edge = (rc_fresh_prearm && in.rc.arm_switch && !prev_arm_switch_);
    prev_arm_switch_ = rc_fresh_prearm ? in.rc.arm_switch : false;

    prearm_flags_ = evaluate_prearm_flags(in, rc_fresh_prearm, imu_fresh);

    auto transition_to = [&](SystemState next) {
        if (state_ != next) {
            state_ = next;
            state_elapsed_s_ = 0.0f;
            landed_idle_timer_s_ = 0.0f;
        }
    };

    switch (state_) {
        case SystemState::BOOT_CALIBRATING:
            if (in.gyro_calibrated && imu_fresh) {
                transition_to(SystemState::DISARMED);
            }
            break;

        case SystemState::BENCH_DIAGNOSTIC:
            // Permanece em modo de bancada até sair explicitamente via CLI
            break;

        case SystemState::DISARMED:
            if (!in.gyro_calibrated) {
                transition_to(SystemState::BOOT_CALIBRATING);
            } else if (arm_rising_edge && prearm_flags_ == PREARM_OK) {
                transition_to(SystemState::ARMED_IDLE);
            }
            break;

        case SystemState::ARMED_IDLE:
            // Kill-Switch imediato ou perda de rádio ainda no chão -> desarma sem susto
            if (!imu_fresh || in.tilt_cos < crash_flip_cos_) {
                observed_switch_disarmed_ = false;
                transition_to(SystemState::FAILSAFE_LOCKED);
            } else if (!rc_fresh_flight || !in.rc.arm_switch ||
                       state_elapsed_s_ >= cfg_.armed_idle_timeout_s) {
                transition_to(SystemState::DISARMED);
            } else if (in.rc.throttle > cfg_.liftoff_throttle) {
                transition_to(SystemState::FLYING);
            }
            break;

        case SystemState::FLYING: {
            const bool resting_zero_stick =
                (in.rc.throttle <= cfg_.max_arm_throttle) &&
                (in.gyro_norm_rads < 0.25f) &&
                (in.abs_climb_rate_ms < 0.25f);
            if (resting_zero_stick) {
                landed_idle_timer_s_ += dt;
            } else {
                landed_idle_timer_s_ = 0.0f;
            }

            // Prioridade 1: Kill-Switch soberano do piloto -> corta motores na hora!
            if (rc_fresh_flight && !in.rc.arm_switch) {
                transition_to(SystemState::DISARMED);
            }
            // Prioridade 2: Capotagem (> 65°) ou morte da IMU -> corte + sirene!
            else if (!imu_fresh || in.tilt_cos < crash_flip_cos_) {
                observed_switch_disarmed_ = false;
                transition_to(SystemState::FAILSAFE_LOCKED);
            }
            // Prioridade 3: Perda de pacotes ESP-NOW > 300 ms -> Estágio 1 (Descida Nivelada)
            else if (!rc_fresh_flight) {
                fs_recovery_pkt_cnt_ = 0u;
                transition_to(SystemState::FAILSAFE_DESCEND);
            }
            // Prioridade 4: Pouso manual confirmado c/ stick no zero -> volta p/ ARMED_IDLE
            else if (cfg_.ground_return_s > 0.0f && landed_idle_timer_s_ >= cfg_.ground_return_s) {
                transition_to(SystemState::ARMED_IDLE);
            }
            break;
        }

        case SystemState::FAILSAFE_DESCEND: {
            const float max_desc_s = in.baro_healthy
                                         ? cfg_.failsafe_max_baro_descend_s
                                         : cfg_.failsafe_max_descend_s;
            const uint8_t req_pkts = std::max<uint8_t>(cfg_.failsafe_recovery_packets, 1u);
            if (!imu_fresh || in.tilt_cos < crash_flip_cos_) {
                observed_switch_disarmed_ = false;
                transition_to(SystemState::FAILSAFE_LOCKED);
            } else if (rc_fresh_flight && !in.rc.arm_switch) {
                // Kill-Switch continua soberano em 1 único pacote recebido!
                transition_to(SystemState::DISARMED);
            } else if (rc_fresh_prearm && fs_recovery_pkt_cnt_ >= req_pkts) {
                // Link de rádio voltou de forma consistente (sem flapping de pacote isolado)!
                transition_to(SystemState::FLYING);
            } else if (in.touchdown_detected ||
                       state_elapsed_s_ >= max_desc_s ||
                       (!in.allow_no_battery_bench && in.vbat_v > 5.0f &&
                        in.vbat_v < cfg_.critical_flight_vbat_v)) {
                observed_switch_disarmed_ = false;
                transition_to(SystemState::FAILSAFE_LOCKED);
            }
            break;
        }

        case SystemState::FAILSAFE_LOCKED:
            // Exige que o piloto coloque explicitamente a chave física ARM em OFF para destravar
            if (rc_fresh_prearm && !in.rc.arm_switch) {
                observed_switch_disarmed_ = true;
                transition_to(SystemState::DISARMED);
            }
            break;
    }

    SafetyDecision dec{};
    dec.state = state_;
    dec.prearm_flags = prearm_flags_;
    dec.motors_armed = (state_ == SystemState::ARMED_IDLE ||
                        state_ == SystemState::FLYING ||
                        state_ == SystemState::FAILSAFE_DESCEND);
    dec.in_air = (state_ == SystemState::FLYING ||
                  state_ == SystemState::FAILSAFE_DESCEND);
    dec.buzzer_siren = (state_ == SystemState::FAILSAFE_DESCEND ||
                        state_ == SystemState::FAILSAFE_LOCKED);

    if (state_ == SystemState::FAILSAFE_DESCEND) {
        // Estágio 1: nivela Roll=0, Pitch=0, Yaw=0 e reduz throttle em rampa suave
        const float progress = std::clamp(state_elapsed_s_ / std::max(cfg_.failsafe_max_descend_s, 0.5f),
                                          0.0f, 1.0f);
        dec.effective_rc.throttle = cfg_.failsafe_hover_thr -
                                    progress * (cfg_.failsafe_hover_thr - cfg_.failsafe_min_thr);
        dec.effective_rc.roll = 0.0f;
        dec.effective_rc.pitch = 0.0f;
        dec.effective_rc.yaw = 0.0f;
        dec.effective_rc.arm_switch = true;
        dec.effective_rc.mode = FlightMode::STABILIZE;
        dec.effective_rc.link_alive = false;
    } else if (dec.motors_armed) {
        dec.effective_rc = in.rc;
    }

    return dec;
}

} // namespace fc
