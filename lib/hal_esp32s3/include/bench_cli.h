#pragma once

#include <Arduino.h>
#include <cstdint>

#include "blackbox_psram.h"
#include "fc_types.h"
#include "imu_orientation.h"
#include "param_registry.h"

namespace fc::hal {

enum class BenchPwmMode : uint8_t {
    NONE         = 0,
    SINGLE_MOTOR = 1,
    ESC_CAL_US   = 2
};

struct BenchMotorOverride {
    BenchPwmMode mode       = BenchPwmMode::NONE;
    uint8_t      motor_1to4 = 1;
    float        pct_0to25  = 0.0f;
    uint16_t     raw_us     = 1000;
    uint32_t     expire_ms  = 0; // 0 = sem expiração (usado apenas no esc_cal assistido)
};

struct BenchStatusView {
    const char*       imu_name    = "N/A";
    uint8_t           imu_whoami  = 0;
    ImuMountRotation  imu_rot     = ImuMountRotation::IMU_ROT_0_DEG;
    uint16_t          int1_hz     = 0;
    bool              baro_ok     = false;
    BaroSample        baro{};
    ImuSample         imu{};
    RcCommand         rc{};
    int8_t            rssi_dbm    = -100;
    TelemetrySnapshot telem{};
};

struct BenchControlHooks {
    void (*read_status)(BenchStatusView& out)                                = nullptr;
    bool (*set_bench_mode)(bool active)                                      = nullptr;
    void (*set_motor_override)(const BenchMotorOverride& ovr)                = nullptr;
    bool (*trigger_gyro_cal)()                                               = nullptr;
    bool (*set_imu_rotation)(ImuMountRotation rot)                           = nullptr;
    void (*tare_baro)()                                                      = nullptr;
    proto::FlightParams (*get_params)()                                      = nullptr;
    bool (*set_param)(uint8_t idx, float val)                                = nullptr;
    bool (*save_params_nvs)()                                                = nullptr;
    bool (*reset_params_nvs)()                                               = nullptr;
};

/**
 * Console Interativo de Bancada via USB-C Serial para os Portões 5, 6, 7 e 8!
 *
 * Intertravamentos de Segurança:
 * - Jamais entra em `BENCH_DIAGNOSTIC` se o drone estiver armado ou se a chave `ARM` do
 *   controle estiver acionada.
 * - `motor_test` corta automaticamente em 3 segundos ou ao apertar qualquer tecla!
 * - `esc_cal` guia passo a passo (2000 µs -> plugar bateria -> 2 bipes -> Enter -> 1000 µs).
 */
class BenchCli {
public:
    void begin(Stream& serial, PsramBlackbox* blackbox, const BenchControlHooks& hooks) noexcept;

    void update(uint32_t now_ms) noexcept;

private:
    void handle_line(char* line, uint32_t now_ms) noexcept;
    void stop_any_motor_override() noexcept;
    void print_help() noexcept;
    void print_status() noexcept;
    void print_prearm_decoded(uint16_t flags) noexcept;

    Stream* serial_ = nullptr;
    PsramBlackbox* blackbox_ = nullptr;
    BenchControlHooks hooks_{};

    char line_buf_[128] = {};
    size_t line_len_ = 0;

    // Modos interativos temporários no Core 0
    bool imu_stream_active_ = false;
    uint32_t imu_stream_end_ms_ = 0;
    uint32_t last_imu_print_ms_ = 0;

    bool motor_run_active_ = false;
    uint32_t motor_run_end_ms_ = 0;

    uint8_t esc_cal_step_ = 0; // 0=inativo, 1=aguardando "CONFIRM", 2=enviando 2000us aguardando Enter
};

} // namespace fc::hal
