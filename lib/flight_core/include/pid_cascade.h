#pragma once

#include "biquad_lpf.h"
#include "fc_types.h"
#include "param_registry.h"

namespace fc {

struct TorqueCommand {
    float roll  = 0.0f; // [-1.0 .. +1.0] Momento desejado em +X (FRD)
    float pitch = 0.0f; // [-1.0 .. +1.0] Momento desejado em +Y (FRD)
    float yaw   = 0.0f; // [-1.0 .. +1.0] Momento desejado em +Z (FRD)
};

/**
 * Controlador PID de Taxa Angular de 1 Eixo (Malha Interna @ 1000 Hz) com:
 * - Derivativo sobre a Medição (-Kd * d(gyro)/dt filtrado via BiquadLPF) -> zero derivative kick!
 * - Anti-Windup Condicional (congela integrador se os atuadores saturarem na mesma direção)
 * - Zeramento de integrador em solo (evita tombar ao decolar de piso desnivelado)
 */
class RateAxisPID {
public:
    void configure(float kp, float ki, float kd,
                   float sample_rate_hz, float dterm_lpf_hz,
                   float integral_limit = 0.35f,
                   float output_limit = 1.0f) noexcept;

    void update_gains_bumpless(float kp, float ki, float kd,
                               float sample_rate_hz, float dterm_lpf_hz) noexcept;

    void reset() noexcept;

    float update(float rate_setpoint_rads,
                 float gyro_measured_rads,
                 float dt_s,
                 bool allow_integration,
                 bool mixer_saturated) noexcept;

    float last_p() const noexcept { return last_p_; }
    float last_i() const noexcept { return last_i_; }
    float last_d() const noexcept { return last_d_; }
    float last_setpoint_rads() const noexcept { return last_sp_; }

private:
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    float i_limit_ = 0.35f;
    float out_limit_ = 1.0f;

    float integral_ = 0.0f;
    float prev_gyro_ = 0.0f;
    bool has_prev_gyro_ = false;
    BiquadLPF d_lpf_{};

    float last_sp_ = 0.0f;
    float last_p_ = 0.0f;
    float last_i_ = 0.0f;
    float last_d_ = 0.0f;
};

/**
 * Controlador de Atitude em Cascata 3-Eixos (Ângulo -> Taxa Angular -> Torque FRD).
 *
 * - Roll e Pitch: Malha P externa de ângulo gera taxa desejada limitada a +-max_rate_rp_rads,
 *   seguida da malha interna RateAxisPID a 1000 Hz.
 * - Yaw: Comando direto de taxa angular (stick * max_rate_yaw_rads) na malha RateAxisPID.
 */
class CascadedPID {
public:
    void configure_from_params(const proto::FlightParams& params,
                               float sample_rate_hz = 1000.0f) noexcept;

    void update_params_bumpless(const proto::FlightParams& params,
                                float sample_rate_hz = 1000.0f) noexcept;

    void reset() noexcept;

    TorqueCommand update_stabilize(float roll_sp_rad,
                                   float pitch_sp_rad,
                                   float yaw_rate_sp_rads,
                                   const EulerAngles& attitude,
                                   const Vec3f& gyro_filt_rads,
                                   float dt_s,
                                   bool in_air,
                                   bool mixer_saturated) noexcept;

    const RateAxisPID& roll_rate_pid() const noexcept { return roll_pid_; }
    const RateAxisPID& pitch_rate_pid() const noexcept { return pitch_pid_; }
    const RateAxisPID& yaw_rate_pid() const noexcept { return yaw_pid_; }

private:
    float ang_rp_kp_ = 4.5f;
    float max_tilt_rad_ = 35.0f * kDegToRad;
    float max_rate_rp_rads_ = 220.0f * kDegToRad;
    float max_rate_yaw_rads_ = 150.0f * kDegToRad;

    RateAxisPID roll_pid_{};
    RateAxisPID pitch_pid_{};
    RateAxisPID yaw_pid_{};
};

} // namespace fc
