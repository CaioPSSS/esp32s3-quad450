#pragma once

#include "fc_types.h"
#include "pid_cascade.h"

namespace fc {

/**
 * Mixer Quad-X 450 mm com Desaturação Airmode, Compensação de Queda de Bateria (VBAT)
 * e Compensação Automática de Inclinação (1 / cos(tilt)).
 *
 * Matriz Canônica FRD:
 *   M1 [0] (FR, CCW): T - Roll + Pitch + Yaw
 *   M2 [1] (RR, CW ): T - Roll - Pitch - Yaw
 *   M3 [2] (RL, CCW): T + Roll - Pitch + Yaw
 *   M4 [3] (FL, CW ): T + Roll + Pitch - Yaw
 */
class QuadXMixer {
public:
    void configure(float motor_idle, float vbat_nominal_v) noexcept;

    MotorOutput mix(bool armed,
                    bool in_air,
                    float throttle_cmd,
                    const TorqueCommand& torque,
                    float tilt_cos,
                    float vbat_measured_v,
                    bool allow_airmode = true) noexcept;

    bool was_saturated() const noexcept { return saturated_; }

    /**
     * Converte valor normalizado [0.0 .. 1.0] em largura de pulso PWM em ponto flutuante
     * (1000.0 .. 2000.0 µs) preservando 100% da resolução de 14 bits do LEDC (~0.15 µs).
     */
    static float to_pwm_us_f(float motor_norm, bool armed) noexcept;

    /**
     * Converte valor normalizado [0.0 .. 1.0] em largura de pulso PWM inteira (1000 .. 2000 µs).
     */
    static uint16_t to_pwm_us(float motor_norm, bool armed) noexcept;

private:
    float motor_idle_ = 0.06f;
    float vbat_nominal_v_ = 11.1f;
    bool saturated_ = false;
};

} // namespace fc
