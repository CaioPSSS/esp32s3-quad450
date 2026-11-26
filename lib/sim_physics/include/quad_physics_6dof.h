#pragma once

#include <cstdint>
#include "fc_types.h"

namespace fc::sim {

struct QuadPhysicalParams {
    float mass_kg          = 0.90f;   // 900 g AUW (Chassi 450mm + A2212 + 3S 2200mAh)
    float arm_length_m     = 0.225f;  // Raio centro-motor = 225 mm (diagonal 450 mm)
    float ixx_kgm2         = 0.0115f; // Inércia Roll
    float iyy_kgm2         = 0.0115f; // Inércia Pitch
    float izz_kgm2         = 0.0210f; // Inércia Yaw
    float motor_tau_s      = 0.032f;  // Atraso de 1ª ordem motor A2212 + hélice 1045 (32 ms)
    float max_thrust_n     = 7.50f;   // ~765 gf máx por motor @ 11.1V
    float yaw_drag_coeff_m = 0.018f;  // Razão torque de arrasto / empuxo da hélice 1045
    float linear_drag_nsm  = 0.45f;   // Arrasto aerodinâmico + rotor blade-flapping H-force (~26 km/h @ 20°)
    float angular_drag_nms = 0.008f;  // Amortecimento aerodinâmico rotacional
    float gyro_vib_amp_rads = 0.35f;  // Vibração mecânica sintética de hélice (~185 Hz)
    Vec3f true_gyro_bias_rads{-0.018f, 0.024f, -0.011f};
};

struct SimTruthState {
    Vec3f pos_ned_m{};         // [Norte, Leste, Down] (altitude acima de Home = -pos_ned_m.z)
    Vec3f vel_ned_ms{};
    Quatf q_body_to_ned{1.0f, 0.0f, 0.0f, 0.0f};
    EulerAngles euler{};
    Vec3f omega_body_rads{};
    Vec3f specific_force_body_ms2{0.0f, 0.0f, -kGravityMs2};
    float motor_actual[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float vbat_loaded_v = 12.2f;
    float total_current_a = 0.3f;
    bool on_ground = true;

    float altitude_m() const noexcept { return -pos_ned_m.z; }
    float climb_rate_ms() const noexcept { return -vel_ned_ms.z; }
};

/**
 * Simulador Físico 6-DoF de Corpo Rígido do Quadricóptero 450 mm.
 *
 * Modela atraso eletromecânico dos motores A2212, queda Ôhmica da LiPo 3S sob corrente,
 * dinâmica rotacional de Euler (I*w_dot + w x Iw), contato normal com o solo (incluindo
 * terreno inclinado no boot!), perturbações externas de vento e vibração de 185 Hz na IMU.
 */
class QuadPhysics6DoF {
public:
    explicit QuadPhysics6DoF(const QuadPhysicalParams& p = {}) : p_(p) { reset(); }

    void reset(float ground_roll_deg = 0.0f, float ground_pitch_deg = 0.0f) noexcept;

    void set_external_torque_body(const Vec3f& torque_nm) noexcept { ext_torque_nm_ = torque_nm; }
    void set_external_wind_force_ned(const Vec3f& force_n) noexcept { ext_force_ned_ = force_n; }
    void set_battery_ocv(float ocv_v) noexcept { battery_ocv_v_ = ocv_v; }

    void step(const MotorOutput& cmd, float dt_s) noexcept;

    ImuSample sample_imu(uint32_t now_us) const noexcept;
    BaroSample sample_baro(uint32_t now_us) const noexcept;

    const SimTruthState& truth() const noexcept { return s_; }

private:
    QuadPhysicalParams p_{};
    SimTruthState s_{};
    float sim_time_s_ = 0.0f;
    float ground_roll_rad_ = 0.0f;
    float ground_pitch_rad_ = 0.0f;
    float battery_ocv_v_ = 12.2f;
    Vec3f ext_torque_nm_{};
    Vec3f ext_force_ned_{};
};

} // namespace fc::sim
