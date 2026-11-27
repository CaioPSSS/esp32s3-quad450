#include <Arduino.h>
#include <Preferences.h>
#include <SPI.h>
#include <Wire.h>
#include <esp_timer.h>
#include <algorithm>
#include <cmath>
#include <cstring>

#include "bench_cli.h"
#include "blackbox_psram.h"
#include "drv_bmp280_i2c.h"
#include "drv_esc_pwm.h"
#include "drv_espnow_lr.h"
#include "drv_icm42688_spi.h"
#include "fc_types.h"
#include "flight_pipeline.h"
#include "imu_orientation.h"
#include "param_registry.h"
#include "radio_packets.h"
#include "seqlock.h"

using namespace fc;
using namespace fc::hal;
using namespace fc::proto;
using namespace fc::sync;

namespace {

// ============================================================================
// Pinagem Oficial da Placa Controladora 7x9 cm (ESP32-S3-DevKitC-1 N16R8)
// ============================================================================
constexpr int kPinAdcVbat   = 1;  // Divisor 39k / 10k (fator 4.90x -> 2.57V @ 12.6V <= 2.90V ADC1!)
constexpr int kPinEscM1Fr   = 4;  // Motor 1 Frente-Direita (CCW)
constexpr int kPinEscM2Rr   = 5;  // Motor 2 Traseira-Direita (CW)
constexpr int kPinEscM3Rl   = 6;  // Motor 3 Traseira-Esquerda (CCW)
constexpr int kPinEscM4Fl   = 7;  // Motor 4 Frente-Esquerda (CW)
constexpr int kPinI2cSda    = 8;  // BMP280 + futuro QMC5883L (ISRs alocadas no Core 0!)
constexpr int kPinI2cScl    = 9;
constexpr int kPinImuCs     = 10; // SPI FSPI 10 MHz -> ICM-42688-P
constexpr int kPinImuMosi   = 11;
constexpr int kPinImuSck    = 12;
constexpr int kPinImuMiso   = 13;
constexpr int kPinImuInt1   = 14; // Pulso Data-Ready 1000 Hz -> ISR em IRAM
constexpr int kPinBuzzer    = 21; // Buzzer ativo 5V NPN Active-HIGH (ex.: KY-012) via +5V_SYS
constexpr int kPinRgbLedV11 = 38; // LED RGB onboard do ESP32-S3-DevKitC-1 v1.1
constexpr int kPinRgbLedV10 = 48; // LED RGB onboard do ESP32-S3-DevKitC-1 v1.0 / YD-ESP32-S3

constexpr float   kVbatDividerScale = (39.0f + 10.0f) / 10.0f; // 4.90x (39k / 10k)
constexpr uint8_t kWifiLrChannel    = 1;
constexpr uint32_t kNvsParamMagic   = 0x51343532u; // "Q452" (19 params c/ filtros v2.1)

// Periféricos e Módulos Globais
SPIClass         g_imu_spi(FSPI);
ImuSpiDriver     g_imu;
Bmp280I2cDriver  g_baro;
EscPwmDriver     g_esc;
EspNowLrLink     g_radio;
PsramBlackbox    g_blackbox;
BenchCli         g_cli;
Preferences      g_prefs;
FlightPipeline   g_pipeline;

TaskHandle_t      g_flight_task_handle = nullptr;
TaskHandle_t      g_comms_task_handle  = nullptr;
volatile uint32_t g_int1_edge_total    = 0;
volatile uint16_t g_measured_int1_hz   = 0;

// ============================================================================
// Estruturas Lock-Free (SeqLock) entre Core 0 (Comms/Baro/CLI) e Core 1 (1 kHz)
// ============================================================================
struct BaroNavShared {
    BaroSample baro{};
};

struct Core1OutShared {
    TelemetrySnapshot telem{};
    ImuSample last_imu{};
    bool buzzer_siren = false;
};

struct Core0ToCore1Req {
    uint32_t seq_id = 0;
    bool apply_params = false;
    FlightParams params{};
    bool recal_gyro = false;
    bool change_bench = false;
    bool bench_enable = false;
    bool change_imu_rot = false;
    ImuMountRotation imu_rot = ImuMountRotation::IMU_ROT_0_DEG;
};

SeqLock<RcCommand>          g_rc_lock;
SeqLock<float>              g_vbat_lock;
SeqLock<BaroNavShared>      g_baro_lock;
SeqLock<BenchMotorOverride> g_bench_ovr_lock;
SeqLock<Core1OutShared>     g_core1_out_lock;
SeqLock<Core0ToCore1Req>    g_core1_req_lock;

FlightParams         g_shadow_params{};
ImuMountRotation     g_shadow_imu_rot = ImuMountRotation::IMU_ROT_0_DEG;
uint32_t             g_next_req_seq = 1;
volatile uint16_t    g_last_rc_seq = 0;

SeqLock<ParamSyncPkt> g_rx_param_pkt_lock;
volatile uint32_t     g_rx_param_pkt_cnt = 0;

// ============================================================================
// ISR Determinística em IRAM para o pulso de 1000 Hz do pino INT1 (GPIO14)
// ============================================================================
void IRAM_ATTR on_imu_int1_isr() {
    g_int1_edge_total = g_int1_edge_total + 1u;
    if (g_flight_task_handle != nullptr) {
        BaseType_t higher_prio_woken = pdFALSE;
        vTaskNotifyGiveFromISR(g_flight_task_handle, &higher_prio_woken);
        if (higher_prio_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

// ============================================================================
// Persistência NVS (Parâmetros PID/Filtros + Rotação de Montagem da IMU)
// ============================================================================
struct NvsStoredBlob {
    uint32_t magic = kNvsParamMagic;
    uint8_t  imu_rot = 0;
    uint8_t  reserved[3] = {0, 0, 0};
    FlightParams params{};
};

void load_nvs_settings() {
    g_shadow_params = FlightParams{};
    g_shadow_imu_rot = ImuMountRotation::IMU_ROT_0_DEG;

    if (g_prefs.begin("quad_fc", true)) {
        NvsStoredBlob blob{};
        if (g_prefs.getBytes("cfg", &blob, sizeof(blob)) == sizeof(blob) &&
            blob.magic == kNvsParamMagic) {
            // Revalida faixas seguras de todos os 19 parâmetros ao carregar da Flash
            FlightParams clean{};
            for (uint8_t i = 0; i < PARAM_COUNT; ++i) {
                float v = 0.0f;
                if (get_param_by_index(blob.params, i, v)) {
                    (void)set_param_by_index(clean, i, v);
                }
            }
            g_shadow_params = clean;
            if (blob.imu_rot <= 3u) {
                g_shadow_imu_rot = static_cast<ImuMountRotation>(blob.imu_rot);
            }
        }
        g_prefs.end();
    }
}

bool save_nvs_settings() {
    // Bloqueio crítico de segurança: gravação na Flash SPI suspende o cache XIP por 25..60 ms!
    // Jamais permitir escrita em NVS com motores armados.
    if (g_core1_out_lock.read().telem.motors.armed) {
        return false;
    }

    NvsStoredBlob blob{};
    blob.magic = kNvsParamMagic;
    blob.imu_rot = static_cast<uint8_t>(g_shadow_imu_rot);
    blob.params = g_shadow_params;

    if (!g_prefs.begin("quad_fc", false)) {
        return false;
    }
    const size_t written = g_prefs.putBytes("cfg", &blob, sizeof(blob));
    g_prefs.end();
    return written == sizeof(blob);
}

void push_core1_request(const Core0ToCore1Req& base_req) {
    Core0ToCore1Req r = base_req;
    r.seq_id = g_next_req_seq++;
    g_core1_req_lock.write(r);
}

// ============================================================================
// Callback ESP-NOW Long Range (Core 0 Wi-Fi Task)
// ============================================================================
void on_espnow_rx(const uint8_t mac[6], const uint8_t* data, size_t len, int8_t /*rssi_dbm*/) {
    if (!data || len < 4 || data[0] != kPacketMagic) {
        return;
    }

    if (data[1] == PKT_RC_COMMAND && len == sizeof(RcCommandPkt)) {
        RcCommandPkt pkt{};
        std::memcpy(&pkt, data, sizeof(pkt));
        if (!verify_packet(pkt, PKT_RC_COMMAND)) {
            return;
        }

        // Auto-Bind seguro ao Controle Caseiro quando ainda não pareado e fora de voo
        if (!g_radio.has_bound_peer()) {
            const Core1OutShared c1 = g_core1_out_lock.read();
            if (!c1.telem.motors.armed) {
                (void)g_radio.register_peer(mac);
            }
        }

        RcCommand rc{};
        rc.throttle     = std::clamp(static_cast<float>(pkt.thr_permil) * 0.001f, 0.0f, 1.0f);
        rc.roll         = std::clamp(static_cast<float>(pkt.roll_permil) * 0.001f, -1.0f, 1.0f);
        rc.pitch        = std::clamp(static_cast<float>(pkt.pitch_permil) * 0.001f, -1.0f, 1.0f);
        rc.yaw          = std::clamp(static_cast<float>(pkt.yaw_permil) * 0.001f, -1.0f, 1.0f);
        rc.arm_switch   = (pkt.arm_magic == kArmMagicValue);
        rc.mode         = (pkt.flight_mode <= 3u) ? static_cast<FlightMode>(pkt.flight_mode)
                                                  : FlightMode::STABILIZE;
        rc.timestamp_us = static_cast<uint32_t>(esp_timer_get_time());
        rc.link_alive   = true;

        g_last_rc_seq = pkt.seq;
        g_rc_lock.write(rc);
    } else if (data[1] == PKT_PARAM_SYNC && len == sizeof(ParamSyncPkt)) {
        ParamSyncPkt ppkt{};
        std::memcpy(&ppkt, data, sizeof(ppkt));
        if (verify_packet(ppkt, PKT_PARAM_SYNC)) {
            g_rx_param_pkt_lock.write(ppkt);
            g_rx_param_pkt_cnt = g_rx_param_pkt_cnt + 1u;
        }
    }
}

// ============================================================================
// CORE 1: Malha Crítica de Controle de Voo de 1000 Hz (Isolada do Wi-Fi/I2C!)
// ============================================================================
void flight_control_task(void* /*arg*/) {
    uint32_t prev_us = static_cast<uint32_t>(esp_timer_get_time());
    uint32_t last_int1_sample_us = prev_us;
    uint32_t seen_req_seq = 0;
    float cpu_load_ema = 5.0f;

    for (;;) {
        // Aguarda pulso físico INT1 da IMU (timeout de segurança de 5 ms caso fio INT1 falhe!)
        const uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
        const uint32_t now_us = static_cast<uint32_t>(esp_timer_get_time());

        float dt_s = static_cast<float>(now_us - prev_us) * 1.0e-6f;
        prev_us = now_us;
        if (!std::isfinite(dt_s) || dt_s < 0.0002f || dt_s > 0.02f) {
            dt_s = 0.001f;
        }

        // 1. Processa solicitações assíncronas do Core 0 (QGroundControl / Bench CLI)
        const Core0ToCore1Req req = g_core1_req_lock.read();
        if (req.seq_id != seen_req_seq) {
            seen_req_seq = req.seq_id;
            if (req.apply_params) {
                g_pipeline.apply_params_live(req.params);
            }
            if (req.change_imu_rot) {
                g_imu.set_mount_rotation(req.imu_rot);
                (void)g_pipeline.request_gyro_recalibration();
            }
            if (req.recal_gyro) {
                (void)g_pipeline.request_gyro_recalibration();
            }
            if (req.change_bench) {
                (void)g_pipeline.set_bench_diagnostic_mode(req.bench_enable);
            }
        }

        // 2. Leitura SPI de 10 MHz da IMU (~16 µs) + validação de integridade de INT1
        ImuSample imu{};
        if (notified > 0 && g_imu.is_detected()) {
            imu = g_imu.read_sample(now_us);
            if (imu.healthy) {
                last_int1_sample_us = now_us;
            }
        } else {
            // Sem pulso INT1: faz leitura apenas p/ diagnóstico visual na CLI, mas marca
            // `healthy = false` para o SafetyStateMachine bloquear armamento / cortar em <10 ms!
            if (g_imu.is_detected()) {
                imu = g_imu.read_sample(now_us);
            }
            imu.healthy = false;
            imu.timestamp_us = last_int1_sample_us;
        }

        // 3. Lê entradas compartilhadas Lock-Free (~0.2 µs, zero mutex!)
        const RcCommand rc = g_rc_lock.read();
        const float vbat_v = g_vbat_lock.read();
        const BaroNavShared baro_nav = g_baro_lock.read();

        // 4. Executa 1 passo determinístico de 1000 Hz do `flight_core` (c/ Fusão Vertical + AltHold!)
        PipelineStepOutput step_out = g_pipeline.step_1khz(imu, rc, vbat_v, now_us, dt_s, baro_nav.baro);

        // 5. Acionamento seguro dos 4 ESCs (400 Hz 14-bit LEDC — 6553 degraus finos de 0.15 µs)
        if (step_out.telemetry.state == SystemState::BENCH_DIAGNOSTIC) {
            const BenchMotorOverride ovr = g_bench_ovr_lock.read();
            const uint32_t now_ms = millis();
            if (ovr.mode == BenchPwmMode::SINGLE_MOTOR &&
                (ovr.expire_ms == 0 || static_cast<int32_t>(ovr.expire_ms - now_ms) > 0)) {
                g_esc.write_single_motor_bench(ovr.motor_1to4, ovr.pct_0to25);
            } else if (ovr.mode == BenchPwmMode::ESC_CAL_US) {
                g_esc.write_all_raw_us_bench(ovr.raw_us);
            } else {
                g_esc.disarm_all();
            }
        } else {
            g_esc.write_motors(step_out.motors);
        }

        // 6. Grava na Caixa-Preta PSRAM de 6.5 MB a 500 Hz (<0.5 µs)
        const SystemState st = step_out.telemetry.state;
        const bool should_rec = (st == SystemState::ARMED_IDLE ||
                                 st == SystemState::FLYING ||
                                 st == SystemState::FAILSAFE_DESCEND);
        g_blackbox.push_from_core1(step_out.blackbox, should_rec);

        // 7. Métricas de CPU do Core 1 e publicação de telemetria fundida para o Core 0
        const uint32_t exec_us = static_cast<uint32_t>(esp_timer_get_time()) - now_us;
        const float inst_pct = static_cast<float>(exec_us) * 0.1f; // exec_us / 1000us * 100%
        cpu_load_ema = 0.98f * cpu_load_ema + 0.02f * inst_pct;

        step_out.telemetry.cpu_load_pct = static_cast<uint8_t>(std::clamp(std::lround(cpu_load_ema), 1L, 100L));

        Core1OutShared c1_snap{};
        c1_snap.telem = step_out.telemetry;
        c1_snap.last_imu = imu;
        c1_snap.buzzer_siren = step_out.buzzer_siren;
        g_core1_out_lock.write(c1_snap);
    }
}

// ============================================================================
// Hooks da CLI de Bancada (Portões 5 a 8) executados no Core 0
// ============================================================================
void cli_read_status(BenchStatusView& out) {
    const Core1OutShared c1 = g_core1_out_lock.read();
    const BaroNavShared bn  = g_baro_lock.read();
    const RcCommand rc      = g_rc_lock.read();
    const uint32_t now_us   = static_cast<uint32_t>(esp_timer_get_time());

    out.imu_name   = g_imu.chip_name();
    out.imu_whoami = g_imu.who_am_i();
    out.imu_rot    = g_shadow_imu_rot;
    out.int1_hz    = g_measured_int1_hz;
    out.baro_ok    = g_baro.is_detected() && bn.baro.healthy;
    out.baro       = bn.baro;
    out.imu        = c1.last_imu;
    out.rc         = rc;
    out.rc.link_alive = rc.link_alive && ((now_us - rc.timestamp_us) < 500000u);
    out.rssi_dbm   = g_radio.last_rssi_dbm();
    out.telem      = c1.telem;
}

bool cli_set_bench_mode(bool active) {
    const Core1OutShared c1 = g_core1_out_lock.read();
    const RcCommand rc = g_rc_lock.read();
    const uint32_t now_us = static_cast<uint32_t>(esp_timer_get_time());
    const bool rc_fresh = rc.link_alive && ((now_us - rc.timestamp_us) < 500000u);

    if (active) {
        if (c1.telem.motors.armed || (rc_fresh && rc.arm_switch)) {
            return false;
        }
    }
    g_bench_ovr_lock.write(BenchMotorOverride{});

    Core0ToCore1Req req{};
    req.change_bench = true;
    req.bench_enable = active;
    push_core1_request(req);
    vTaskDelay(pdMS_TO_TICKS(5));
    return (g_core1_out_lock.read().telem.state == SystemState::BENCH_DIAGNOSTIC) == active;
}

void cli_set_motor_override(const BenchMotorOverride& ovr) {
    g_bench_ovr_lock.write(ovr);
}

bool cli_trigger_gyro_cal() {
    const Core1OutShared c1 = g_core1_out_lock.read();
    if (c1.telem.motors.armed) return false;
    Core0ToCore1Req req{};
    req.recal_gyro = true;
    push_core1_request(req);
    return true;
}

bool cli_set_imu_rotation(ImuMountRotation rot) {
    const Core1OutShared c1 = g_core1_out_lock.read();
    if (c1.telem.motors.armed) return false;
    g_shadow_imu_rot = rot;
    if (!save_nvs_settings()) return false;

    Core0ToCore1Req req{};
    req.change_imu_rot = true;
    req.imu_rot = rot;
    push_core1_request(req);
    return true;
}

void cli_tare_baro() {
    g_baro.reset_ground_reference();
}

FlightParams cli_get_params() {
    return g_shadow_params;
}

bool cli_set_param(uint8_t idx, float val) {
    if (!set_param_by_index(g_shadow_params, idx, val)) {
        return false;
    }
    Core0ToCore1Req req{};
    req.apply_params = true;
    req.params = g_shadow_params;
    push_core1_request(req);
    return true;
}

bool cli_save_params_nvs() {
    return save_nvs_settings();
}

bool cli_reset_params_nvs() {
    const Core1OutShared c1 = g_core1_out_lock.read();
    if (c1.telem.motors.armed) return false;
    g_shadow_params = FlightParams{};
    if (!save_nvs_settings()) return false;

    Core0ToCore1Req req{};
    req.apply_params = true;
    req.params = g_shadow_params;
    push_core1_request(req);
    return true;
}

// ============================================================================
// Leitura 8x Oversampled do Divisor VBAT 3S (GPIO1: 39k / 10k -> 4.90x)
// ============================================================================
float read_vbat_volts() {
    uint32_t mv_sum = 0;
    for (int i = 0; i < 8; ++i) {
        mv_sum += static_cast<uint32_t>(analogReadMilliVolts(kPinAdcVbat));
    }
    const float pin_v = (static_cast<float>(mv_sum) / 8.0f) * 0.001f;
    return pin_v * kVbatDividerScale;
}

void update_status_rgb(SystemState st, uint16_t prearm_flags, uint32_t now_ms) {
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    uint8_t r = 0, g = 0, b = 0;
    const bool blink = ((now_ms / 200u) & 1u) != 0u;
    switch (st) {
        case SystemState::BOOT_CALIBRATING:
            r = 25; g = 18; b = 0; // Amarelo: calibrando giroscópio
            break;
        case SystemState::DISARMED:
            if (prearm_flags == PREARM_OK) {
                r = 0; g = 28; b = 0; // Verde sólido: pronto para armar!
            } else {
                r = 0; g = 0; b = blink ? 25 : 4; // Azul piscando: Pre-Arm aguardando item
            }
            break;
        case SystemState::BENCH_DIAGNOSTIC:
            r = 0; g = 20; b = 20; // Ciano: modo bancada ativo
            break;
        case SystemState::ARMED_IDLE:
        case SystemState::FLYING:
            r = 35; g = 0; b = 0; // Vermelho forte: HÉLICES VIVAS!
            break;
        case SystemState::FAILSAFE_DESCEND:
        case SystemState::FAILSAFE_LOCKED:
            r = blink ? 40 : 0; g = 0; b = blink ? 35 : 0; // Roxo piscando: Failsafe!
            break;
    }

    // Só aciona o periférico RMT quando a cor de fato muda (0 Hz em voo estável!),
    // acionando GPIO48 (DevKitC-1 v1.0 / YD) e GPIO38 (DevKitC-1 v1.1).
    static uint8_t prev_r = 0xFF, prev_g = 0xFF, prev_b = 0xFF;
    if (r != prev_r || g != prev_g || b != prev_b) {
        prev_r = r;
        prev_g = g;
        prev_b = b;
        rgbLedWrite(kPinRgbLedV10, r, g, b);
        rgbLedWrite(kPinRgbLedV11, r, g, b);
    }
#else
    (void)st; (void)prearm_flags; (void)now_ms;
#endif
}

// ============================================================================
// CORE 0: Task Dedicada de Comunicações, Barômetro I2C, VBAT, LED/Buzzer e CLI
// (Garante que a ISR de hardware do Wire I2C e o RMT do LED fiquem no Core 0!)
// ============================================================================
void core0_comms_task(void* /*arg*/) {
    // Inicializa o barramento I2C dentro da task fixada no Core 0 para alocar sua ISR na CPU 0!
    g_baro.begin(Wire, kPinI2cSda, kPinI2cScl);

    uint32_t last_vbat_ms    = 0;
    uint32_t last_baro_ms    = 0;
    uint32_t last_telem_ms   = 0;
    uint32_t last_hz_meas_ms = 0;
    uint32_t prev_int1_edges = 0;
    uint32_t seen_param_pkt  = 0;
    uint16_t telem_seq       = 0;
    bool     was_calibrating = true;
    uint32_t cal_done_beep_until_ms = 0;

    for (;;) {
        const uint32_t now_ms = millis();
        const uint32_t now_us = static_cast<uint32_t>(esp_timer_get_time());

        // 1. Mede taxa real de pulsos INT1 a cada 1.0 segundo
        if (now_ms - last_hz_meas_ms >= 1000u) {
            last_hz_meas_ms = now_ms;
            const uint32_t cur_edges = g_int1_edge_total;
            g_measured_int1_hz = static_cast<uint16_t>(std::min<uint32_t>(cur_edges - prev_int1_edges, 65535u));
            prev_int1_edges = cur_edges;
        }

        // 2. Amostra tensão da bateria LiPo 3S a 50 Hz (20 ms)
        if (now_ms - last_vbat_ms >= 20u) {
            last_vbat_ms = now_ms;
            g_vbat_lock.write(read_vbat_volts());
        }

        // 3. Amostra barômetro BMP280 I2C a 40 Hz (25 ms) exclusivamente no Core 0
        if (now_ms - last_baro_ms >= 25u) {
            last_baro_ms = now_ms;
            BaroSample b{};
            if (g_baro.poll(now_us, b)) {
                BaroNavShared bn{b};
                g_baro_lock.write(bn);
            }
        }

        // 4. Processa comandos remotos de parâmetros vindos do QGroundControl via `ground_tx`
        const uint32_t cur_p_cnt = g_rx_param_pkt_cnt;
        if (cur_p_cnt != seen_param_pkt) {
            seen_param_pkt = cur_p_cnt;
            const ParamSyncPkt req = g_rx_param_pkt_lock.read();

            ParamSyncPkt ack{};
            ack.type      = PKT_PARAM_SYNC;
            ack.req_id    = req.req_id;
            ack.op        = PARAM_OP_ACK;
            ack.param_idx = req.param_idx;
            ack.status    = 1; // 0 = OK, 1 = Erro/Bloqueado

            if (req.op == PARAM_OP_GET) {
                float v = 0.0f;
                if (get_param_by_index(g_shadow_params, req.param_idx, v)) {
                    ack.value = v;
                    ack.status = 0;
                }
            } else if (req.op == PARAM_OP_SET) {
                if (cli_set_param(req.param_idx, req.value)) {
                    (void)get_param_by_index(g_shadow_params, req.param_idx, ack.value);
                    ack.status = 0;
                }
            } else if (req.op == PARAM_OP_SAVE_NVS) {
                ack.op = PARAM_OP_SAVE_NVS;
                if (save_nvs_settings()) {
                    ack.status = 0;
                }
            }

            seal_packet(ack);
            (void)g_radio.send(&ack, sizeof(ack));
        }

        // 5. Envia Telemetria Rápida (34 bytes) a 20 Hz (50 ms) para o Controle Caseiro + QGC
        const Core1OutShared c1 = g_core1_out_lock.read();
        if (now_ms - last_telem_ms >= 50u && g_radio.has_bound_peer()) {
            last_telem_ms = now_ms;

            auto clamp_i16 = [](float v) -> int16_t {
                return static_cast<int16_t>(std::clamp(std::lround(v), -32767L, 32767L));
            };

            TelemFastPkt tpkt{};
            tpkt.type          = PKT_TELEM_FAST;
            tpkt.seq           = telem_seq++;
            tpkt.state         = static_cast<uint8_t>(c1.telem.state);
            tpkt.flight_mode   = static_cast<uint8_t>(c1.telem.mode);
            tpkt.prearm_flags  = c1.telem.prearm_flags;
            tpkt.roll_deg10    = clamp_i16(c1.telem.attitude.roll_deg() * 10.0f);
            tpkt.pitch_deg10   = clamp_i16(c1.telem.attitude.pitch_deg() * 10.0f);
            tpkt.yaw_deg10     = clamp_i16(c1.telem.attitude.yaw_deg() * 10.0f);
            tpkt.alt_cm        = static_cast<int32_t>(std::lround(c1.telem.altitude_m * 100.0f));
            tpkt.vz_cms        = clamp_i16(c1.telem.climb_rate_ms * 100.0f);
            tpkt.vbat_mv       = static_cast<uint16_t>(std::clamp(std::lround(c1.telem.vbat_v * 1000.0f), 0L, 65535L));
            for (int i = 0; i < 4; ++i) {
                tpkt.motor_pct[i] = static_cast<uint8_t>(std::clamp(std::lround(c1.telem.motors.throttle[i] * 100.0f), 0L, 100L));
            }
            tpkt.drone_rssi_dbm = g_radio.last_rssi_dbm();
            tpkt.cpu_load_pct   = c1.telem.cpu_load_pct;
            tpkt.loop_dt_us     = c1.telem.loop_dt_us;
            tpkt.ack_seq        = g_last_rc_seq;
            seal_packet(tpkt);
            (void)g_radio.send(&tpkt, sizeof(tpkt));
        }

        // 6. Buzzer NPN Active-HIGH (GPIO21) + LED RGB (GPIO48 v1.0 / GPIO38 v1.1)
        if (was_calibrating && c1.telem.state != SystemState::BOOT_CALIBRATING) {
            was_calibrating = false;
            cal_done_beep_until_ms = now_ms + 180u;
        }

        const bool low_bat_flight = c1.telem.motors.armed && (c1.telem.vbat_v > 5.0f && c1.telem.vbat_v < 10.5f);
        if (now_ms < cal_done_beep_until_ms) {
            digitalWrite(kPinBuzzer, HIGH);
        } else if (c1.buzzer_siren || low_bat_flight) {
            digitalWrite(kPinBuzzer, ((now_ms / 150u) & 1u) ? HIGH : LOW);
        } else {
            digitalWrite(kPinBuzzer, LOW);
        }

        update_status_rgb(c1.telem.state, c1.telem.prearm_flags, now_ms);

        // 7. Console Interativo USB-C de Bancada
        g_cli.update(now_ms);

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

} // namespace

void setup() {
    // 1. Garante IMEDIATAMENTE 1000 µs nos 4 ESCs no primeiro milissegundo de boot!
    EscPwmPins esc_pins{kPinEscM1Fr, kPinEscM2Rr, kPinEscM3Rl, kPinEscM4Fl};
    g_esc.begin(esc_pins);
    g_esc.disarm_all();

    pinMode(kPinBuzzer, OUTPUT);
    digitalWrite(kPinBuzzer, LOW);

    Serial.begin(115200);
    Serial.setTxTimeoutMs(10); // Evita travamento de 2000 ms caso USB-CDC não esteja aberta no host

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // 2. Carrega ganhos PID/Filtros e orientação da IMU salvos na Flash NVS
    load_nvs_settings();
    g_pipeline.init(g_shadow_params, 1000.0f, 2000u);
    g_vbat_lock.write(read_vbat_volts());

    // 3. Aloca Caixa-Preta de 6.5 MB na Octal PSRAM
    g_blackbox.begin(2); // 500 Hz

    // 4. Inicializa IMU primária ICM-42688-P em SPI FSPI 10 MHz
    ImuSpiPins imu_pins{kPinImuCs, kPinImuMosi, kPinImuSck, kPinImuMiso, kPinImuInt1};
    g_imu.begin(g_imu_spi, imu_pins, g_shadow_imu_rot);

    // 5. Inicializa Rádio ESP-NOW Long Range puro (`WIFI_PHY_RATE_LORA_250K` @ 19.5 dBm)
    g_radio.set_rx_callback(&on_espnow_rx);
    g_radio.begin(kWifiLrChannel, true);

    // 6. Inicializa Console Interativo de Bancada (Portões 5 a 8)
    BenchControlHooks hooks{};
    hooks.read_status        = &cli_read_status;
    hooks.set_bench_mode     = &cli_set_bench_mode;
    hooks.set_motor_override = &cli_set_motor_override;
    hooks.trigger_gyro_cal   = &cli_trigger_gyro_cal;
    hooks.set_imu_rotation   = &cli_set_imu_rotation;
    hooks.tare_baro          = &cli_tare_baro;
    hooks.get_params         = &cli_get_params;
    hooks.set_param          = &cli_set_param;
    hooks.save_params_nvs    = &cli_save_params_nvs;
    hooks.reset_params_nvs   = &cli_reset_params_nvs;
    g_cli.begin(Serial, &g_blackbox, hooks);

    // 7. Lança `CommsCore0` explicitamente no Core 0 (onde Wire I2C, RMT RGB e CLI rodam)
    xTaskCreatePinnedToCore(
        &core0_comms_task,
        "CommsCore0",
        8192,
        nullptr,
        3,
        &g_comms_task_handle,
        0);

    // 8. Lança `FlightControlTask` isolada no Core 1 com prioridade máxima e engata ISR em GPIO14
    xTaskCreatePinnedToCore(
        &flight_control_task,
        "Flight1kHz",
        8192,
        nullptr,
        configMAX_PRIORITIES - 1,
        &g_flight_task_handle,
        1);

    attachInterrupt(digitalPinToInterrupt(kPinImuInt1), &on_imu_int1_isr, RISING);

    // Bipe curto duplo de inicialização concluída
    digitalWrite(kPinBuzzer, HIGH); delay(60);
    digitalWrite(kPinBuzzer, LOW);  delay(60);
    digitalWrite(kPinBuzzer, HIGH); delay(60);
    digitalWrite(kPinBuzzer, LOW);
}

void loop() {
    // No Arduino-ESP32 (`CONFIG_ARDUINO_RUNNING_CORE = 1`), `loopTask` roda no Core 1.
    // Mantemos `loop()` adormecida para reservar 100% do Core 1 à malha determinística `Flight1kHz`!
    vTaskDelay(pdMS_TO_TICKS(1000));
}
