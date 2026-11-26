#include "flight_pipeline.h"

#include <algorithm>
#include <cmath>

namespace fc {

void FlightPipeline::init(const proto::FlightParams& params,
                          float sample_rate_hz,
                          uint32_t cal_samples) noexcept {
    params_ = params;
    sample_rate_hz_ = (sample_rate_hz > 100.0f) ? sample_rate_hz : 1000.0f;

    calibrator_ = GyroCalibrator(cal_samples, 0.035f);
    gyro_lpf_.configure(sample_rate_hz_, params_.gyro_lpf_hz);
    accel_lpf_.configure(sample_rate_hz_, params_.accel_lpf_hz);
    vbat_lpf_.configure(sample_rate_hz_, 5.0f);
    vbat_lpf_.reset(params_.vbat_nominal);

    ahrs_.reset();
    ahrs_.configure(params_.mahony_kp, params_.mahony_ki);

    SafetyConfig scfg{};
    scfg.failsafe_hover_thr = params_.fs_hover_thr;
    safety_.configure(scfg);
    safety_.reset();

    pid_.configure_from_params(params_, sample_rate_hz_);
    mixer_.configure(params_.motor_idle, params_.vbat_nominal);

    was_calibrated_ = false;
}

void FlightPipeline::apply_params_live(const proto::FlightParams& params) noexcept {
    params_ = params;
    gyro_lpf_.reconfigure_bumpless(sample_rate_hz_, params_.gyro_lpf_hz);
    accel_lpf_.reconfigure_bumpless(sample_rate_hz_, params_.accel_lpf_hz);
    ahrs_.configure(params_.mahony_kp, params_.mahony_ki);

    SafetyConfig scfg{};
    scfg.failsafe_hover_thr = params_.fs_hover_thr;
    safety_.configure(scfg);

    pid_.update_params_bumpless(params_, sample_rate_hz_);
    mixer_.configure(params_.motor_idle, params_.vbat_nominal);
}

bool FlightPipeline::request_gyro_recalibration() noexcept {
    if (safety_.state() == SystemState::ARMED_IDLE ||
        safety_.state() == SystemState::FLYING ||
        safety_.state() == SystemState::FAILSAFE_DESCEND) {
        return false; // Jamais recalibra em voo!
    }
    calibrator_.reset();
    was_calibrated_ = false;
    return true;
}

float FlightPipeline::shape_stick(float v, float deadband, float expo) noexcept {
    if (!std::isfinite(v)) {
        return 0.0f;
    }
    const float clamped = std::clamp(v, -1.0f, 1.0f);
    const float abs_v = std::fabs(clamped);
    if (abs_v <= deadband) {
        return 0.0f;
    }
    const float rescaled = (abs_v - deadband) / (1.0f - deadband);
    const float shaped = (1.0f - expo) * rescaled + expo * rescaled * rescaled * rescaled;
    return (clamped >= 0.0f) ? shaped : -shaped;
}

PipelineStepOutput FlightPipeline::step_1khz(const ImuSample& imu,
                                             const RcCommand& rc,
                                             float vbat_v,
                                             uint32_t now_us,
                                             float dt_s) noexcept {
    PipelineStepOutput out{};
    const float dt = (std::isfinite(dt_s) && dt_s > 0.0f) ? std::min(dt_s, 0.02f) : (1.0f / sample_rate_hz_);

    // 1. Filtra leitura da bateria 3S
    const float vbat_filt = (std::isfinite(vbat_v) && vbat_v > 3.0f)
                                ? vbat_lpf_.apply(vbat_v)
                                : vbat_v;

    // 2. Calibração estacionária no boot
    if (imu.healthy && !calibrator_.is_calibrated()) {
        const bool just_done = calibrator_.push_sample(imu.gyro_rads, imu.accel_ms2);
        if (just_done && !was_calibrated_) {
            was_calibrated_ = true;
            accel_lpf_.reset(calibrator_.mean_accel_ms2());
            gyro_lpf_.reset({0.0f, 0.0f, 0.0f});
            ahrs_.align_to_gravity(calibrator_.mean_accel_ms2(), 0.0f);
        }
    }

    // 3. Compensação de bias + Filtragem Biquad 2ª ordem + Mahony Quaternions
    Vec3f gyro_filt{};
    Vec3f accel_filt = imu.accel_ms2;
    if (imu.healthy && calibrator_.is_calibrated()) {
        const Vec3f gyro_unbiased = imu.gyro_rads - calibrator_.gyro_bias_rads();
        gyro_filt  = gyro_lpf_.apply(gyro_unbiased);
        accel_filt = accel_lpf_.apply(imu.accel_ms2);
        ahrs_.update(gyro_filt, accel_filt, dt);
    }

    // 4. Máquina de Estados de Segurança e Failsafe
    SafetyInputs sin{};
    sin.now_us = now_us;
    sin.imu_healthy = imu.healthy;
    sin.last_imu_us = imu.timestamp_us;
    sin.gyro_calibrated = calibrator_.is_calibrated();
    sin.rc = rc;
    sin.tilt_cos = ahrs_.tilt_cos();
    sin.vbat_v = vbat_filt;
    sin.allow_no_battery_bench = allow_no_bat_;

    const SafetyDecision sdec = safety_.update(sin, dt);
    out.buzzer_siren = sdec.buzzer_siren;

    // 5. Controle de Atitude em Cascata + Mixer Quad-X
    TorqueCommand torque{};
    if (!sdec.motors_armed) {
        pid_.reset();
    } else {
        const float max_tilt_rad = params_.max_tilt_deg * kDegToRad;
        const float max_yaw_rads = 150.0f * kDegToRad;

        const float roll_sp_rad  = shape_stick(sdec.effective_rc.roll)  * max_tilt_rad;
        const float pitch_sp_rad = shape_stick(sdec.effective_rc.pitch) * max_tilt_rad;
        const float yaw_sp_rads  = shape_stick(sdec.effective_rc.yaw)   * max_yaw_rads;

        torque = pid_.update_stabilize(roll_sp_rad,
                                       pitch_sp_rad,
                                       yaw_sp_rads,
                                       ahrs_.euler(),
                                       gyro_filt,
                                       dt,
                                       sdec.in_air,
                                       mixer_.was_saturated());
    }

    out.motors = mixer_.mix(sdec.motors_armed,
                            sdec.in_air,
                            sdec.effective_rc.throttle,
                            torque,
                            ahrs_.tilt_cos(),
                            vbat_filt);

    // 6. Preenche Snapshot de Telemetria (Core 0 / MAVLink / OLED)
    out.telemetry.state = sdec.state;
    out.telemetry.mode = sdec.effective_rc.mode;
    out.telemetry.prearm_flags = sdec.prearm_flags;
    out.telemetry.attitude = ahrs_.euler();
    out.telemetry.vbat_v = vbat_filt;
    out.telemetry.motors = out.motors;
    out.telemetry.loop_dt_us = static_cast<uint16_t>(std::clamp(dt * 1e6f, 0.0f, 65535.0f));

    // 7. Preenche Frame compacto de 64 bytes para a Caixa-Preta PSRAM
    auto clamp_i16 = [](float v) -> int16_t {
        return static_cast<int16_t>(std::clamp(std::lround(v), -32767L, 32767L));
    };
    out.blackbox.timestamp_us = now_us;
    out.blackbox.dt_us = out.telemetry.loop_dt_us;
    out.blackbox.state = static_cast<uint8_t>(sdec.state);
    out.blackbox.flags = static_cast<uint8_t>((sdec.motors_armed ? 0x01u : 0u) |
                                              (sdec.in_air ? 0x02u : 0u) |
                                              (mixer_.was_saturated() ? 0x04u : 0u) |
                                              (ahrs_.is_accel_gate_open() ? 0x08u : 0u));
    out.blackbox.gyro_raw_rads[0]  = imu.gyro_rads.x;
    out.blackbox.gyro_raw_rads[1]  = imu.gyro_rads.y;
    out.blackbox.gyro_raw_rads[2]  = imu.gyro_rads.z;
    out.blackbox.gyro_filt_rads[0] = gyro_filt.x;
    out.blackbox.gyro_filt_rads[1] = gyro_filt.y;
    out.blackbox.gyro_filt_rads[2] = gyro_filt.z;

    out.blackbox.rate_sp_mrads[0] = clamp_i16(pid_.roll_rate_pid().last_setpoint_rads() * 1000.0f);
    out.blackbox.rate_sp_mrads[1] = clamp_i16(pid_.pitch_rate_pid().last_setpoint_rads() * 1000.0f);
    out.blackbox.pid_p_permil[0]  = clamp_i16(pid_.roll_rate_pid().last_p() * 1000.0f);
    out.blackbox.pid_p_permil[1]  = clamp_i16(pid_.pitch_rate_pid().last_p() * 1000.0f);
    out.blackbox.pid_i_permil[0]  = clamp_i16(pid_.roll_rate_pid().last_i() * 1000.0f);
    out.blackbox.pid_i_permil[1]  = clamp_i16(pid_.pitch_rate_pid().last_i() * 1000.0f);
    out.blackbox.pid_d_permil[0]  = clamp_i16(pid_.roll_rate_pid().last_d() * 1000.0f);
    out.blackbox.pid_d_permil[1]  = clamp_i16(pid_.pitch_rate_pid().last_d() * 1000.0f);

    out.blackbox.euler_deg100[0] = clamp_i16(ahrs_.euler().roll_deg() * 100.0f);
    out.blackbox.euler_deg100[1] = clamp_i16(ahrs_.euler().pitch_deg() * 100.0f);
    out.blackbox.euler_deg100[2] = clamp_i16(ahrs_.euler().yaw_deg() * 100.0f);

    for (int i = 0; i < 4; ++i) {
        out.blackbox.motor_permil[i] = static_cast<uint16_t>(
            std::clamp(std::lround(out.motors.throttle[i] * 1000.0f), 0L, 1000L));
    }
    out.blackbox.vbat_mv = static_cast<uint16_t>(
        std::clamp(std::lround(std::max(vbat_filt, 0.0f) * 1000.0f), 0L, 65535L));

    return out;
}

} // namespace fc
