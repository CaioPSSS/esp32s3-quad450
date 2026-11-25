#pragma once

#include "fc_types.h"

namespace fc {

/**
 * Estimador de Atitude Mahony 6-DoF / 9-DoF em Quaternions (Convenção FRD / NED).
 *
 * Convenção física da IMU em FRD:
 *   - Parado e nivelado na mesa: accel_ms2 = [0, 0, -9.80665] m/s² (força normal p/ cima = -Z).
 *   - Asa direita inclinada p/ baixo (+Roll):  accel_ms2.y < 0 (gravidade aponta em +Y).
 *   - Nariz inclinado p/ cima (+Pitch):        accel_ms2.x > 0 (gravidade aponta em -X).
 *
 * Características de voo:
 *   - Livre de Gimbal Lock (integração em Quaternions de Hamilton).
 *   - Porta de rejeição de aceleração linear: só corrige horizonte com o acelerômetro
 *     quando 0.80g <= ||accel|| <= 1.20g (evita entortar o horizonte em subidas fortes).
 *   - Estimação integral de bias residual do giroscópio com anti-windup.
 *   - Calcula aceleração cinemática vertical na Terra (earth_accel_up_ms2, positivo p/ cima)
 *     e tilt_cos() em O(1) para o Altitude Hold e para o detector de capotagem (> 65°).
 */
class MahonyAHRS {
public:
    MahonyAHRS() = default;
    MahonyAHRS(float kp, float ki) : kp_(kp), ki_(ki) {}

    void configure(float kp, float ki) noexcept;
    void reset() noexcept;

    /**
     * Inicializa instantaneamente o quaternion de Roll/Pitch a partir do vetor acelerômetro
     * com o drone parado no solo (útil ao final da calibração de boot em terreno inclinado).
     */
    bool align_to_gravity(const Vec3f& accel_ms2, float initial_yaw_rad = 0.0f) noexcept;

    /**
     * Atualização principal 6-DoF (Giroscópio + Acelerômetro) a 1000 Hz.
     */
    void update(const Vec3f& gyro_rads, const Vec3f& accel_ms2, float dt_s) noexcept;

    /**
     * Atualização 9-DoF opcional (adiciona proa magnética compensada em inclinação na Fase 2).
     */
    void update_with_heading(const Vec3f& gyro_rads,
                             const Vec3f& accel_ms2,
                             float measured_yaw_rad,
                             float kp_yaw,
                             float dt_s) noexcept;

    const Quatf& quaternion() const noexcept { return q_; }
    const EulerAngles& euler() const noexcept { return euler_; }

    /**
     * Aceleração cinemática vertical pura no referencial da Terra (m/s², positivo = subindo),
     * já descontando a gravidade 1g independente da inclinação do drone.
     */
    float earth_accel_up_ms2() const noexcept { return earth_accel_up_ms2_; }

    /**
     * Cosseno do ângulo total de inclinação em relação à vertical (1.0 = plano, 0.0 = 90°).
     */
    float tilt_cos() const noexcept { return tilt_cos_; }
    float tilt_angle_rad() const noexcept;

    /**
     * Bias residual aprendido pelo termo integral (-integral_fb_ equivale ao offset do gyro).
     */
    Vec3f estimated_gyro_bias_rads() const noexcept {
        return {-integral_fb_.x, -integral_fb_.y, -integral_fb_.z};
    }

    bool is_accel_gate_open() const noexcept { return accel_gate_open_; }

private:
    void integrate_and_update_outputs(const Vec3f& gyro_rads,
                                      const Vec3f& accel_ms2,
                                      const Vec3f& err_body,
                                      bool allow_integral,
                                      float dt_s) noexcept;
    void normalize_quaternion() noexcept;
    void update_derived_quantities(const Vec3f& accel_ms2) noexcept;

    float kp_ = 2.0f;
    float ki_ = 0.05f;

    Quatf q_{1.0f, 0.0f, 0.0f, 0.0f};
    Vec3f integral_fb_{0.0f, 0.0f, 0.0f};
    EulerAngles euler_{};
    float earth_accel_up_ms2_ = 0.0f;
    float tilt_cos_ = 1.0f;
    bool accel_gate_open_ = false;
};

} // namespace fc
