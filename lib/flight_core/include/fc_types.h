#pragma once

#include <cmath>
#include <cstdint>

namespace fc {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;
constexpr float kGravityMs2 = 9.80665f;

// ============================================================================
// Convenção Aeronáutica Estrita:
// - Corpo: FRD (X = Frente, Y = Direita, Z = Baixo)
// - Terra: NED (X = Norte, Y = Leste, Z = Baixo)
// - Roll (+): Asa direita desce
// - Pitch (+): Nariz sobe
// - Yaw (+): Nariz gira para a direita (sentido horário visto de cima)
// ============================================================================

struct Vec3f {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    constexpr Vec3f() = default;
    constexpr Vec3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3f operator+(const Vec3f& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3f operator-(const Vec3f& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3f operator*(float s) const { return {x * s, y * s, z * s}; }

    constexpr float dot(const Vec3f& o) const { return x * o.x + y * o.y + z * o.z; }

    constexpr Vec3f cross(const Vec3f& o) const {
        return {
            y * o.z - z * o.y,
            z * o.x - x * o.z,
            x * o.y - y * o.x
        };
    }

    float norm_sq() const { return x * x + y * y + z * z; }
    float norm() const { return std::sqrt(norm_sq()); }
};

struct Quatf {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct EulerAngles {
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;

    float roll_deg() const { return roll_rad * kRadToDeg; }
    float pitch_deg() const { return pitch_rad * kRadToDeg; }
    float yaw_deg() const { return yaw_rad * kRadToDeg; }
};

enum class FlightMode : uint8_t {
    STABILIZE = 0,
    ALT_HOLD  = 1,
    POS_HOLD  = 2,
    RTH       = 3
};

enum class SystemState : uint8_t {
    BOOT_CALIBRATING = 0,
    DISARMED         = 1,
    BENCH_DIAGNOSTIC = 2,
    ARMED_IDLE       = 3,
    FLYING           = 4,
    FAILSAFE_DESCEND = 5,
    FAILSAFE_LOCKED  = 6
};

enum PreArmFlag : uint16_t {
    PREARM_OK               = 0,
    ERR_IMU_UNHEALTHY       = (1u << 0),
    ERR_GYRO_UNCALIBRATED   = (1u << 1),
    ERR_RC_LINK_LOST        = (1u << 2),
    ERR_ARM_SWITCH_BOOT_ON  = (1u << 3),
    ERR_THROTTLE_NOT_ZERO   = (1u << 4),
    ERR_TILT_TOO_HIGH       = (1u << 5),
    ERR_BATTERY_LOW         = (1u << 6),
    ERR_BENCH_MODE_ACTIVE   = (1u << 7)
};

struct ImuSample {
    Vec3f gyro_rads{};         // Velocidade angular FRD (rad/s)
    Vec3f accel_ms2{};         // Aceleração específica FRD (m/s²)
    uint32_t timestamp_us = 0;
    bool healthy = false;
};

struct BaroSample {
    float altitude_m = 0.0f;   // Altitude relativa ao solo (m, positivo para cima)
    float pressure_pa = 101325.0f;
    float temperature_c = 25.0f;
    uint32_t timestamp_us = 0;
    bool healthy = false;
};

struct RcCommand {
    float throttle = 0.0f;     // [0.0 .. 1.0]
    float roll = 0.0f;         // [-1.0 .. +1.0] (+ = direita)
    float pitch = 0.0f;        // [-1.0 .. +1.0] (+ = nariz p/ cima)
    float yaw = 0.0f;          // [-1.0 .. +1.0] (+ = horário)
    bool arm_switch = false;
    FlightMode mode = FlightMode::STABILIZE;
    uint32_t timestamp_us = 0;
    bool link_alive = false;
};

// Ordem física dos motores no chassi Quad-X 450 mm:
// [0] = M1 Frente-Direita (FR, CCW)
// [1] = M2 Traseira-Direita (RR, CW)
// [2] = M3 Traseira-Esquerda (RL, CCW)
// [3] = M4 Frente-Esquerda (FL, CW)
struct MotorOutput {
    float throttle[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // [0.0 .. 1.0]
    bool armed = false;
};

struct TelemetrySnapshot {
    SystemState state = SystemState::BOOT_CALIBRATING;
    FlightMode mode = FlightMode::STABILIZE;
    uint16_t prearm_flags = PREARM_OK;
    EulerAngles attitude{};
    float altitude_m = 0.0f;
    float climb_rate_ms = 0.0f;
    float vbat_v = 11.1f;
    MotorOutput motors{};
    uint16_t loop_dt_us = 1000;
    uint8_t cpu_load_pct = 0;
};

} // namespace fc
