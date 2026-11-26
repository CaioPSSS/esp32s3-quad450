#include "quad_physics_6dof.h"

#include <algorithm>
#include <cmath>

namespace fc::sim {

namespace {
Quatf quat_from_euler(float roll, float pitch, float yaw) noexcept {
    const float cr = std::cos(0.5f * roll);
    const float sr = std::sin(0.5f * roll);
    const float cp = std::cos(0.5f * pitch);
    const float sp = std::sin(0.5f * pitch);
    const float cy = std::cos(0.5f * yaw);
    const float sy = std::sin(0.5f * yaw);
    return {
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    };
}

EulerAngles euler_from_quat(const Quatf& q) noexcept {
    EulerAngles e;
    const float r31 = 2.0f * (q.x * q.z - q.w * q.y);
    const float r32 = 2.0f * (q.y * q.z + q.w * q.x);
    const float r33 = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    e.pitch_rad = std::asin(std::clamp(-r31, -1.0f, 1.0f));
    e.roll_rad  = std::atan2(r32, r33);
    e.yaw_rad   = std::atan2(2.0f * (q.w * q.z + q.x * q.y),
                             1.0f - 2.0f * (q.y * q.y + q.z * q.z));
    return e;
}

Vec3f rotate_body_to_ned(const Quatf& q, const Vec3f& v) noexcept {
    const float r11 = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    const float r12 = 2.0f * (q.x * q.y - q.w * q.z);
    const float r13 = 2.0f * (q.x * q.z + q.w * q.y);

    const float r21 = 2.0f * (q.x * q.y + q.w * q.z);
    const float r22 = 1.0f - 2.0f * (q.x * q.x + q.z * q.z);
    const float r23 = 2.0f * (q.y * q.z - q.w * q.x);

    const float r31 = 2.0f * (q.x * q.z - q.w * q.y);
    const float r32 = 2.0f * (q.y * q.z + q.w * q.x);
    const float r33 = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);

    return {
        r11 * v.x + r12 * v.y + r13 * v.z,
        r21 * v.x + r22 * v.y + r23 * v.z,
        r31 * v.x + r32 * v.y + r33 * v.z
    };
}

Vec3f rotate_ned_to_body(const Quatf& q, const Vec3f& v_ned) noexcept {
    const float r11 = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    const float r21 = 2.0f * (q.x * q.y + q.w * q.z);
    const float r31 = 2.0f * (q.x * q.z - q.w * q.y);

    const float r12 = 2.0f * (q.x * q.y - q.w * q.z);
    const float r22 = 1.0f - 2.0f * (q.x * q.x + q.z * q.z);
    const float r32 = 2.0f * (q.y * q.z + q.w * q.x);

    const float r13 = 2.0f * (q.x * q.z + q.w * q.y);
    const float r23 = 2.0f * (q.y * q.z - q.w * q.x);
    const float r33 = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);

    return {
        r11 * v_ned.x + r21 * v_ned.y + r31 * v_ned.z,
        r12 * v_ned.x + r22 * v_ned.y + r32 * v_ned.z,
        r13 * v_ned.x + r23 * v_ned.y + r33 * v_ned.z
    };
}
} // namespace

void QuadPhysics6DoF::reset(float ground_roll_deg, float ground_pitch_deg) noexcept {
    s_ = SimTruthState{};
    sim_time_s_ = 0.0f;
    ground_roll_rad_  = ground_roll_deg * kDegToRad;
    ground_pitch_rad_ = ground_pitch_deg * kDegToRad;
    s_.q_body_to_ned  = quat_from_euler(ground_roll_rad_, ground_pitch_rad_, 0.0f);
    s_.euler          = euler_from_quat(s_.q_body_to_ned);
    s_.on_ground      = true;
    s_.vbat_loaded_v  = battery_ocv_v_;
    s_.specific_force_body_ms2 = rotate_ned_to_body(s_.q_body_to_ned, {0.0f, 0.0f, -kGravityMs2});
    ext_torque_nm_ = {};
    ext_force_ned_ = {};
}

void QuadPhysics6DoF::step(const MotorOutput& cmd, float dt_s) noexcept {
    if (dt_s <= 0.0f) {
        return;
    }
    sim_time_s_ += dt_s;

    // 1. Dinâmica eletromecânica de 1ª ordem dos 4 motores + consumo de corrente e sag da bateria
    const float alpha_m = dt_s / (p_.motor_tau_s + dt_s);
    float total_current = 0.35f; // Consumo basal ESP32-S3 + sensores
    for (int i = 0; i < 4; ++i) {
        const float target = cmd.armed ? std::clamp(cmd.throttle[i], 0.0f, 1.0f) : 0.0f;
        s_.motor_actual[i] += alpha_m * (target - s_.motor_actual[i]);
        // A2212 puxa ~15A em 100% throttle (curva quadrática aproximada de potência)
        total_current += 15.0f * (0.25f * s_.motor_actual[i] + 0.75f * s_.motor_actual[i] * s_.motor_actual[i]);
    }
    s_.total_current_a = total_current;
    // Resistência interna equivalente ~28 mOhm de uma LiPo 3S 2200mAh 35C + descarga lenta
    battery_ocv_v_ = std::max(9.0f, battery_ocv_v_ - (total_current * dt_s / 3600.0f) * (1.2f / 2.2f));
    s_.vbat_loaded_v = std::max(8.5f, battery_ocv_v_ - 0.028f * total_current);

    // 2. Empuxo individual de cada conjunto Motor A2212 + Hélice 1045
    const float v_ratio = s_.vbat_loaded_v / 11.1f;
    float thrust_n[4];
    float total_thrust_n = 0.0f;
    for (int i = 0; i < 4; ++i) {
        const float m = s_.motor_actual[i];
        thrust_n[i] = p_.max_thrust_n * v_ratio * (0.60f * m + 0.40f * m * m);
        total_thrust_n += thrust_n[i];
    }

    // 3. Momentos em relação ao CG no sistema FRD:
    //    M1 [0]=FR(CCW), M2 [1]=RR(CW), M3 [2]=RL(CCW), M4 [3]=FL(CW)
    const float l_xy = p_.arm_length_m * 0.70710678f; // L / sqrt(2)
    const Vec3f torque_body{
        l_xy * ((thrust_n[2] + thrust_n[3]) - (thrust_n[0] + thrust_n[1])) + ext_torque_nm_.x,
        l_xy * ((thrust_n[0] + thrust_n[3]) - (thrust_n[1] + thrust_n[2])) + ext_torque_nm_.y,
        p_.yaw_drag_coeff_m * ((thrust_n[0] + thrust_n[2]) - (thrust_n[1] + thrust_n[3])) + ext_torque_nm_.z
    };

    // 4. Forças translacionais (Empuxo puxa em -Z FRD!)
    const Vec3f thrust_ned = rotate_body_to_ned(s_.q_body_to_ned, {0.0f, 0.0f, -total_thrust_n});
    const Vec3f drag_ned   = s_.vel_ned_ms * (-p_.linear_drag_nsm);
    const Vec3f grav_ned   = {0.0f, 0.0f, p_.mass_kg * kGravityMs2};
    const Vec3f net_force_ned = thrust_ned + drag_ned + ext_force_ned_ + grav_ned;

    // 5. Restrição de contato com o solo (altitude == 0 <=> pos_ned_m.z >= 0)
    if (s_.pos_ned_m.z >= 0.0f && net_force_ned.z >= 0.0f && s_.vel_ned_ms.z >= 0.0f) {
        // Permanece apoiado no solo: força normal cancela peso e segura inclinação do terreno
        s_.on_ground = true;
        s_.pos_ned_m.z = 0.0f;
        s_.vel_ned_ms = {0.0f, 0.0f, 0.0f};
        s_.omega_body_rads = {0.0f, 0.0f, 0.0f};
        s_.q_body_to_ned = quat_from_euler(ground_roll_rad_, ground_pitch_rad_, s_.euler.yaw_rad);
        s_.euler = euler_from_quat(s_.q_body_to_ned);
        s_.specific_force_body_ms2 = rotate_ned_to_body(s_.q_body_to_ned, {0.0f, 0.0f, -kGravityMs2});
        return;
    }

    s_.on_ground = false;

    // 6. Integração Rotacional de Euler 3D: I * w_dot = tau - w x (I * w) - c_w * w
    const Vec3f iw{
        p_.ixx_kgm2 * s_.omega_body_rads.x,
        p_.iyy_kgm2 * s_.omega_body_rads.y,
        p_.izz_kgm2 * s_.omega_body_rads.z
    };
    const Vec3f gyro_coupling = s_.omega_body_rads.cross(iw);
    const Vec3f alpha_body{
        (torque_body.x - gyro_coupling.x - p_.angular_drag_nms * s_.omega_body_rads.x) / p_.ixx_kgm2,
        (torque_body.y - gyro_coupling.y - p_.angular_drag_nms * s_.omega_body_rads.y) / p_.iyy_kgm2,
        (torque_body.z - gyro_coupling.z - p_.angular_drag_nms * s_.omega_body_rads.z) / p_.izz_kgm2
    };
    s_.omega_body_rads = s_.omega_body_rads + (alpha_body * dt_s);

    // Atualiza quaternion de atitude
    const float hdt = 0.5f * dt_s;
    const Quatf qp = s_.q_body_to_ned;
    const Vec3f& w = s_.omega_body_rads;
    s_.q_body_to_ned.w += (-qp.x * w.x - qp.y * w.y - qp.z * w.z) * hdt;
    s_.q_body_to_ned.x += ( qp.w * w.x + qp.y * w.z - qp.z * w.y) * hdt;
    s_.q_body_to_ned.y += ( qp.w * w.y - qp.x * w.z + qp.z * w.x) * hdt;
    s_.q_body_to_ned.z += ( qp.w * w.z + qp.x * w.y - qp.y * w.x) * hdt;

    const float qn = std::sqrt(s_.q_body_to_ned.w * s_.q_body_to_ned.w +
                               s_.q_body_to_ned.x * s_.q_body_to_ned.x +
                               s_.q_body_to_ned.y * s_.q_body_to_ned.y +
                               s_.q_body_to_ned.z * s_.q_body_to_ned.z);
    if (qn > 1e-9f) {
        s_.q_body_to_ned.w /= qn;
        s_.q_body_to_ned.x /= qn;
        s_.q_body_to_ned.y /= qn;
        s_.q_body_to_ned.z /= qn;
    }
    s_.euler = euler_from_quat(s_.q_body_to_ned);

    // 7. Integração Translacional NED + pouso
    const Vec3f accel_ned = net_force_ned * (1.0f / p_.mass_kg);
    s_.vel_ned_ms = s_.vel_ned_ms + (accel_ned * dt_s);
    s_.pos_ned_m  = s_.pos_ned_m  + (s_.vel_ned_ms * dt_s);

    if (s_.pos_ned_m.z > 0.0f) {
        s_.pos_ned_m.z = 0.0f;
        if (s_.vel_ned_ms.z > 0.0f) {
            s_.vel_ned_ms.z = 0.0f;
        }
        s_.vel_ned_ms.x *= 0.8f;
        s_.vel_ned_ms.y *= 0.8f;
        s_.on_ground = true;
    }

    // Força específica vista pelo acelerômetro FRD: f_b = R_ned_to_body * (a_kin_ned - [0,0,+g])
    const Vec3f specific_force_ned = accel_ned - Vec3f{0.0f, 0.0f, kGravityMs2};
    s_.specific_force_body_ms2 = rotate_ned_to_body(s_.q_body_to_ned, specific_force_ned);
}

ImuSample QuadPhysics6DoF::sample_imu(uint32_t now_us) const noexcept {
    ImuSample imu{};
    imu.timestamp_us = now_us;
    imu.healthy = true;

    const float avg_rpm_factor = 0.25f * (s_.motor_actual[0] + s_.motor_actual[1] +
                                          s_.motor_actual[2] + s_.motor_actual[3]);
    const float vib = p_.gyro_vib_amp_rads * avg_rpm_factor * std::sin(2.0f * kPi * 185.0f * sim_time_s_);
    const float acc_vib = 1.5f * avg_rpm_factor * std::cos(2.0f * kPi * 185.0f * sim_time_s_);

    imu.gyro_rads = s_.omega_body_rads + p_.true_gyro_bias_rads + Vec3f{vib, -vib, 0.5f * vib};
    imu.accel_ms2 = s_.specific_force_body_ms2 + Vec3f{acc_vib, -acc_vib, acc_vib};
    return imu;
}

BaroSample QuadPhysics6DoF::sample_baro(uint32_t now_us) const noexcept {
    BaroSample b{};
    b.timestamp_us = now_us;
    b.healthy = true;
    const float baro_noise = 0.08f * std::sin(2.0f * kPi * 3.7f * sim_time_s_);
    b.altitude_m = s_.altitude_m() + baro_noise;
    b.pressure_pa = 101325.0f * std::pow(1.0f - (0.0065f * b.altitude_m / 288.15f), 5.255f);
    return b;
}

} // namespace fc::sim
