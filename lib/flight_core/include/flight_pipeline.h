#pragma once

#include <cstdint>

#include "biquad_lpf.h"
#include "fc_types.h"
#include "gyro_calibrator.h"
#include "mahony_ahrs.h"
#include "param_registry.h"
#include "pid_cascade.h"
#include "quad_mixer.h"
#include "safety_fsm.h"

namespace fc {

// Frame de alta resolução (64 bytes exatos) pronto para gravação na PSRAM Blackbox (Sprint 6)
#pragma pack(push, 1)
struct BlackboxFrame {
    uint32_t timestamp_us      = 0;               // 4
    uint16_t dt_us             = 1000;            // 6
    uint8_t  state             = 0;               // 7
    uint8_t  flags             = 0;               // 8
    float    gyro_raw_rads[3]  = {0, 0, 0};       // 20
    float    gyro_filt_rads[3] = {0, 0, 0};       // 32
    int16_t  rate_sp_mrads[2]  = {0, 0};          // 36 (Roll/Pitch setpoint em mrad/s)
    int16_t  pid_p_permil[2]   = {0, 0};          // 40
    int16_t  pid_i_permil[2]   = {0, 0};          // 44
    int16_t  pid_d_permil[2]   = {0, 0};          // 48
    int16_t  euler_deg100[3]   = {0, 0, 0};       // 54 (centésimos de grau)
    uint16_t motor_permil[4]   = {0, 0, 0, 0};    // 62 (0 .. 1000)
    uint16_t vbat_mv           = 0;               // 64 bytes exatos!
};
static_assert(sizeof(BlackboxFrame) == 64, "BlackboxFrame deve ter exatos 64 bytes alinhados");
#pragma pack(pop)

struct PipelineStepOutput {
    MotorOutput motors{};
    TelemetrySnapshot telemetry{};
    BlackboxFrame blackbox{};
    bool buzzer_siren = false;
};

/**
 * Orquestrador Determinístico de 1000 Hz da Controladora de Voo (`flight_core`).
 *
 * Conecta em cascata:
 *   ImuSample -> GyroCalibrator -> BiquadVec3 (Gyro/Accel) -> MahonyAHRS
 *   -> SafetyStateMachine -> Curva Stick/Deadband -> CascadedPID -> QuadXMixer.
 *
 * Zero alocação dinâmica, tempo constante O(1), idêntico no SITL (PC) e no ESP32-S3!
 */
class FlightPipeline {
public:
    FlightPipeline() { init(proto::FlightParams{}); }

    void init(const proto::FlightParams& params,
              float sample_rate_hz = 1000.0f,
              uint32_t cal_samples = 2000) noexcept;

    /**
     * Atualiza ganhos e filtros em pleno voo sem causar degrau nos motores (MAVLink PARAM_SET).
     */
    void apply_params_live(const proto::FlightParams& params) noexcept;

    /**
     * Executa 1 passo completo da malha de tempo real (1 ms @ 1000 Hz).
     */
    PipelineStepOutput step_1khz(const ImuSample& imu,
                                 const RcCommand& rc,
                                 float vbat_v,
                                 uint32_t now_us,
                                 float dt_s) noexcept;

    /**
     * Permite recalibrar o giroscópio sob comando de solo (apenas quando DISARMED).
     */
    bool request_gyro_recalibration() noexcept;

    void set_allow_no_battery_bench(bool allow) noexcept { allow_no_bat_ = allow; }

    const proto::FlightParams& params() const noexcept { return params_; }
    const MahonyAHRS& ahrs() const noexcept { return ahrs_; }
    const GyroCalibrator& calibrator() const noexcept { return calibrator_; }
    const SafetyStateMachine& safety() const noexcept { return safety_; }

private:
    static float shape_stick(float v, float deadband = 0.025f, float expo = 0.25f) noexcept;

    float sample_rate_hz_ = 1000.0f;
    proto::FlightParams params_{};

    GyroCalibrator calibrator_{2000, 0.035f};
    BiquadVec3 gyro_lpf_{};
    BiquadVec3 accel_lpf_{};
    BiquadLPF vbat_lpf_{};
    MahonyAHRS ahrs_{};
    SafetyStateMachine safety_{};
    CascadedPID pid_{};
    QuadXMixer mixer_{};

    bool was_calibrated_ = false;
    bool allow_no_bat_ = false;
};

} // namespace fc
