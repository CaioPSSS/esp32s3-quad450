#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

namespace fc::proto {

// Parâmetros ajustáveis ao vivo via QGroundControl (MAVLink PARAM_SET), CLI USB e salvos na NVS
enum ParamIndex : uint8_t {
    PARAM_GYRO_LPF_HZ = 0,
    PARAM_ACCEL_LPF_HZ,
    PARAM_DTERM_LPF_HZ,
    PARAM_MAHONY_KP,
    PARAM_MAHONY_KI,
    PARAM_ANG_RP_KP,
    PARAM_MAX_TILT_DEG,
    PARAM_RATE_RP_KP,
    PARAM_RATE_RP_KI,
    PARAM_RATE_RP_KD,
    PARAM_RATE_YAW_KP,
    PARAM_RATE_YAW_KI,
    PARAM_MOTOR_IDLE,
    PARAM_VBAT_NOMINAL,
    PARAM_FS_HOVER_THR,
    PARAM_ALT_HOLD_KP,
    PARAM_VEL_Z_KP,
    PARAM_VEL_Z_KI,
    PARAM_VEL_Z_KD,
    PARAM_COUNT
};

struct FlightParams {
    float gyro_lpf_hz   = 65.0f;   // Corte Biquad Giroscópio (Hz) — rejeita 1x RPM hover (63-80 Hz)
    float accel_lpf_hz  = 30.0f;   // Corte Biquad Acelerômetro (Hz)
    float dterm_lpf_hz  = 35.0f;   // Corte Biquad Derivativo PID (Hz) — evita aquecimento em hélices 10"
    float mahony_kp     = 0.20f;   // Ganho proporcional gravidade em voo (tau=5s: zero washout em curvas!)
    float mahony_ki     = 0.008f;  // Ganho integral bias Mahony (ativo apenas nivelado)
    float ang_rp_kp     = 4.5f;    // Malha externa Ângulo -> Taxa (1/s)
    float max_tilt_deg  = 35.0f;   // Inclinação máxima em Stabilize / AltHold (°)
    float rate_rp_kp    = 0.14f;   // Ganho P Taxa Roll/Pitch
    float rate_rp_ki    = 0.28f;   // Ganho I Taxa Roll/Pitch (rejeição rápida de rajada/CG)
    float rate_rp_kd    = 0.0045f; // Ganho D Taxa Roll/Pitch (sobre medição)
    float rate_yaw_kp   = 0.25f;   // Ganho P Taxa Yaw
    float rate_yaw_ki   = 0.15f;   // Ganho I Taxa Yaw
    float motor_idle    = 0.06f;   // Rotação mínima armado (6%)
    float vbat_nominal  = 11.1f;   // Referência 3S p/ compensação de sag (V)
    float fs_hover_thr  = 0.36f;   // Throttle base inicial de hover / Failsafe
    float alt_hold_kp   = 1.15f;   // Malha externa AltHold: erro m -> razão subida m/s
    float vel_z_kp      = 0.22f;   // Malha interna Velocidade Vertical: ganho P
    float vel_z_ki      = 0.14f;   // Malha interna Velocidade Vertical: ganho I
    float vel_z_kd      = 0.028f;  // Amortecimento direto via aceleração vertical Earth-Z
};

struct ParamMeta {
    const char name[17]; // Máx 16 chars no padrão MAVLink PARAM_VALUE
    float min_val;
    float max_val;
};

inline constexpr ParamMeta kParamTable[PARAM_COUNT] = {
    {"GYRO_LPF_HZ",   20.0f, 250.0f},
    {"ACCEL_LPF_HZ",   5.0f, 100.0f},
    {"DTERM_LPF_HZ",  15.0f, 150.0f},
    {"MAHONY_KP",      0.1f,  10.0f},
    {"MAHONY_KI",      0.0f,   1.0f},
    {"ANG_RP_KP",      1.0f,  10.0f},
    {"MAX_TILT_DEG",  15.0f,  55.0f},
    {"RATE_RP_KP",     0.01f,  1.0f},
    {"RATE_RP_KI",     0.0f,   1.5f},
    {"RATE_RP_KD",     0.0f,   0.05f},
    {"RATE_YAW_KP",    0.01f,  1.0f},
    {"RATE_YAW_KI",    0.0f,   1.0f},
    {"MOTOR_IDLE",     0.03f,  0.15f},
    {"VBAT_NOMINAL",  10.5f,  12.6f},
    {"FS_HOVER_THR",   0.20f,  0.55f},
    {"ALT_HOLD_KP",    0.20f,  4.00f},
    {"VEL_Z_KP",       0.05f,  0.80f},
    {"VEL_Z_KI",       0.00f,  0.60f},
    {"VEL_Z_KD",       0.00f,  0.15f}
};

inline float* param_ptr_mutable(FlightParams& p, uint8_t idx) noexcept {
    switch (idx) {
        case PARAM_GYRO_LPF_HZ:  return &p.gyro_lpf_hz;
        case PARAM_ACCEL_LPF_HZ: return &p.accel_lpf_hz;
        case PARAM_DTERM_LPF_HZ: return &p.dterm_lpf_hz;
        case PARAM_MAHONY_KP:    return &p.mahony_kp;
        case PARAM_MAHONY_KI:    return &p.mahony_ki;
        case PARAM_ANG_RP_KP:    return &p.ang_rp_kp;
        case PARAM_MAX_TILT_DEG: return &p.max_tilt_deg;
        case PARAM_RATE_RP_KP:   return &p.rate_rp_kp;
        case PARAM_RATE_RP_KI:   return &p.rate_rp_ki;
        case PARAM_RATE_RP_KD:   return &p.rate_rp_kd;
        case PARAM_RATE_YAW_KP:  return &p.rate_yaw_kp;
        case PARAM_RATE_YAW_KI:  return &p.rate_yaw_ki;
        case PARAM_MOTOR_IDLE:   return &p.motor_idle;
        case PARAM_VBAT_NOMINAL: return &p.vbat_nominal;
        case PARAM_FS_HOVER_THR: return &p.fs_hover_thr;
        case PARAM_ALT_HOLD_KP:  return &p.alt_hold_kp;
        case PARAM_VEL_Z_KP:     return &p.vel_z_kp;
        case PARAM_VEL_Z_KI:     return &p.vel_z_ki;
        case PARAM_VEL_Z_KD:     return &p.vel_z_kd;
        default:                 return nullptr;
    }
}

inline bool get_param_by_index(const FlightParams& p, uint8_t idx, float& out_val) noexcept {
    FlightParams& mutable_ref = const_cast<FlightParams&>(p);
    const float* ptr = param_ptr_mutable(mutable_ref, idx);
    if (!ptr) {
        return false;
    }
    out_val = *ptr;
    return true;
}

inline bool set_param_by_index(FlightParams& p, uint8_t idx, float new_val) noexcept {
    if (idx >= PARAM_COUNT || !std::isfinite(new_val)) {
        return false;
    }
    float* ptr = param_ptr_mutable(p, idx);
    if (!ptr) {
        return false;
    }
    const float lo = kParamTable[idx].min_val;
    const float hi = kParamTable[idx].max_val;
    if (new_val < lo) new_val = lo;
    if (new_val > hi) new_val = hi;
    *ptr = new_val;
    return true;
}

inline int find_param_by_name(const char* mav_name_16) noexcept {
    if (!mav_name_16) {
        return -1;
    }
    for (uint8_t i = 0; i < PARAM_COUNT; ++i) {
        if (std::strncmp(kParamTable[i].name, mav_name_16, 16) == 0) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

} // namespace fc::proto
