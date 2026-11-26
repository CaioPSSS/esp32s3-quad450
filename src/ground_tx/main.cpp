#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <cstring>

#include "drv_espnow_lr.h"
#include "fc_types.h"
#include "gimbal_mapper.h"
#include "mavlink_bridge.h"
#include "oled_hud.h"
#include "radio_packets.h"
#include "seqlock.h"

using namespace fc;
using namespace fc::hal;
using namespace fc::proto;
using namespace fc::sync;
using namespace fc::tx;

namespace {

// ============================================================================
// Pinagem Verificada do Controle Caseiro (ESP32-S3-DevKitC-1 WROOM-1U)
// ============================================================================
constexpr int kPinAdcThrottle = 1;  // ADC1_CH0 (Gimbal Esq Vertical s/ mola)
constexpr int kPinAdcYaw      = 2;  // ADC1_CH1 (Gimbal Esq Horizontal)
constexpr int kPinAdcPitch    = 4;  // ADC1_CH3 (Gimbal Dir Vertical)
constexpr int kPinAdcRoll     = 5;  // ADC1_CH4 (Gimbal Dir Horizontal)
constexpr int kPinAdcTxBat    = 6;  // ADC1_CH5 (Divisor 100k+100k da 18650)

constexpr int kPinOledSda     = 8;
constexpr int kPinOledScl     = 9;

constexpr int kPinSwArmKill   = 10; // INPUT_PULLUP: LOW = ARM, HIGH = DISARM
constexpr int kPinSwModeA     = 11; // Chave 3 posições lado A
constexpr int kPinSwModeB     = 12; // Chave 3 posições lado B
constexpr int kPinBtnAux      = 13; // Botão AUX (Calibração no boot / Salvar NVS)
constexpr int kPinBuzzer      = 14; // Buzzer de alerta

constexpr uint8_t kWifiLrChannel = 1;

// Objetos globais e canais Lock-Free entre Core 1 (RC 100 Hz) e Core 0 (OLED + MAVLink)
EspNowLrLink     g_radio;
GimbalMapper     g_mapper;
OledHud          g_oled;
MavlinkUsbBridge g_mav_bridge;
Preferences      g_prefs;

struct RxTelemShared {
    TelemFastPkt pkt{};
    uint32_t rx_ms = 0;
    bool valid = false;
};

struct TxControlShared {
    NormalizedSticks sticks{};
    bool arm_switch_on = false;
    uint8_t cal_step = 0;
};

SeqLock<RxTelemShared>   g_rx_telem_lock;
SeqLock<TxControlShared> g_tx_ctrl_lock;
SeqLock<ParamSyncPkt>    g_rx_param_ack_lock;
volatile uint32_t        g_param_ack_counter = 0;

SeqLock<ParamSyncPkt>    g_tx_param_req_lock;
volatile uint32_t        g_tx_param_req_counter = 0;

// Oversampling 16x no ADC1 de 12 bits p/ eliminar ruído de potenciômetro
uint16_t read_adc_oversampled(int pin) {
    uint32_t sum = 0;
    for (int i = 0; i < 16; ++i) {
        sum += static_cast<uint32_t>(analogRead(pin));
    }
    return static_cast<uint16_t>(sum >> 4);
}

FlightMode read_mode_switch() {
    const bool a_low = (digitalRead(kPinSwModeA) == LOW);
    const bool b_low = (digitalRead(kPinSwModeB) == LOW);
    if (a_low) return FlightMode::STABILIZE;
    if (b_low) return FlightMode::POS_HOLD; // Fase 2 GPS PosHold / RTH
    return FlightMode::ALT_HOLD;
}

void on_espnow_packet(const uint8_t mac[6], const uint8_t* data, size_t len, int8_t rssi_dbm) {
    if (!data || len < 4 || data[0] != kPacketMagic) {
        return;
    }

    if (data[1] == PKT_TELEM_FAST && len == sizeof(TelemFastPkt)) {
        TelemFastPkt pkt{};
        std::memcpy(&pkt, data, sizeof(pkt));
        if (verify_packet(pkt, PKT_TELEM_FAST)) {
            if (!g_radio.has_bound_peer()) {
                (void)g_radio.register_peer(mac); // Auto-Bind Unicast ao Drone!
            }
            pkt.drone_rssi_dbm = rssi_dbm;
            RxTelemShared snap{pkt, millis(), true};
            g_rx_telem_lock.write(snap);
        }
    } else if (data[1] == PKT_PARAM_SYNC && len == sizeof(ParamSyncPkt)) {
        ParamSyncPkt ack{};
        std::memcpy(&ack, data, sizeof(ack));
        if (verify_packet(ack, PKT_PARAM_SYNC)) {
            g_rx_param_ack_lock.write(ack);
            g_param_ack_counter = g_param_ack_counter + 1u;
        }
    }
}

void load_calibration_nvs() {
    if (g_prefs.begin("quad_tx", true)) {
        GimbalCalibrationData saved{};
        if (g_prefs.getBytes("gcal", &saved, sizeof(saved)) == sizeof(saved) &&
            saved.magic == 0x4743414Cu) {
            g_mapper.set_calibration(saved);
        }
        g_prefs.end();
    }
}

void save_calibration_nvs(const GimbalCalibrationData& cal) {
    if (g_prefs.begin("quad_tx", false)) {
        g_prefs.putBytes("gcal", &cal, sizeof(cal));
        g_prefs.end();
    }
}

// ============================================================================
// CORE 1: Task Determinística de 100 Hz (Leitura Gimbals + Chaves + ESP-NOW TX)
// ============================================================================
void rc_transmitter_task(void* /*arg*/) {
    TickType_t last_wake = xTaskGetTickCount();
    uint16_t seq = 0;
    uint32_t seen_param_req = 0;
    uint8_t cal_step = 0;

    // Se o botão AUX estiver pressionado no boot (e chave ARM desligada!), entra no Wizard de Calibração
    if (digitalRead(kPinBtnAux) == LOW && digitalRead(kPinSwArmKill) == HIGH) {
        cal_step = 1;
        g_mapper.begin_calibration();
        while (digitalRead(kPinBtnAux) == LOW) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    bool prev_aux = false;

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10)); // Exatos 100 Hz (10 ms)

        const uint16_t raw_thr   = read_adc_oversampled(kPinAdcThrottle);
        const uint16_t raw_yaw   = read_adc_oversampled(kPinAdcYaw);
        const uint16_t raw_pitch = read_adc_oversampled(kPinAdcPitch);
        const uint16_t raw_roll  = read_adc_oversampled(kPinAdcRoll);

        const bool arm_sw_on = (digitalRead(kPinSwArmKill) == LOW);
        const bool aux_pressed = (digitalRead(kPinBtnAux) == LOW);
        const bool aux_rising = (aux_pressed && !prev_aux);
        prev_aux = aux_pressed;

        // Modo Assistente de Calibração dos Gimbals (jamais transmite ARM enquanto calibra!)
        if (cal_step == 1) {
            g_mapper.record_min_max_sample(raw_thr, raw_yaw, raw_pitch, raw_roll);
            if (aux_rising) {
                cal_step = 2;
            }
            TxControlShared st{{}, false, cal_step};
            g_tx_ctrl_lock.write(st);
            continue;
        } else if (cal_step == 2) {
            if (aux_rising) {
                if (g_mapper.finalize_calibration(raw_thr, raw_yaw, raw_pitch, raw_roll)) {
                    save_calibration_nvs(g_mapper.calibration());
                    cal_step = 0;
                } else {
                    cal_step = 1;
                    g_mapper.begin_calibration();
                }
            }
            TxControlShared st{{}, false, cal_step};
            g_tx_ctrl_lock.write(st);
            continue;
        }

        const NormalizedSticks sticks = g_mapper.map_raw(raw_thr, raw_yaw, raw_pitch, raw_roll);
        TxControlShared st{sticks, arm_sw_on, 0};
        g_tx_ctrl_lock.write(st);

        // Monta e envia pacote RC de 20 bytes via ESP-NOW Long Range
        RcCommandPkt pkt{};
        pkt.type = PKT_RC_COMMAND;
        pkt.seq = seq++;
        pkt.thr_permil   = sticks.thr_permil;
        pkt.roll_permil  = sticks.roll_permil;
        pkt.pitch_permil = sticks.pitch_permil;
        pkt.yaw_permil   = sticks.yaw_permil;
        pkt.arm_magic    = arm_sw_on ? kArmMagicValue : kDisarmMagicValue;
        pkt.flight_mode  = static_cast<uint8_t>(read_mode_switch());
        pkt.aux_flags    = aux_pressed ? 0x01u : 0x00u;
        seal_packet(pkt);
        (void)g_radio.send(&pkt, sizeof(pkt));

        // Se a ponte MAVLink no Core 0 enfileirou ajuste de parâmetro, envia logo em seguida
        const uint32_t cur_req = g_tx_param_req_counter;
        if (cur_req != seen_param_req) {
            seen_param_req = cur_req;
            const ParamSyncPkt ppkt = g_tx_param_req_lock.read();
            (void)g_radio.send(&ppkt, sizeof(ppkt));
        }
    }
}

} // namespace

void setup() {
    Serial.begin(115200); // Porta USB-C Nativa (MAVLink v2 USB-OTG p/ QGroundControl!)

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    pinMode(kPinSwArmKill, INPUT_PULLUP);
    pinMode(kPinSwModeA,   INPUT_PULLUP);
    pinMode(kPinSwModeB,   INPUT_PULLUP);
    pinMode(kPinBtnAux,    INPUT_PULLUP);
    pinMode(kPinBuzzer,    OUTPUT);
    digitalWrite(kPinBuzzer, LOW);

    load_calibration_nvs();
    g_oled.begin(Wire, kPinOledSda, kPinOledScl, 0x3C);
    g_mav_bridge.begin(Serial);

    g_radio.set_rx_callback(&on_espnow_packet);
    g_radio.begin(kWifiLrChannel, true);

    // Fixa o transmissor RC de 100 Hz exclusivamente no Core 1 (zero jitter de I2C/USB!)
    xTaskCreatePinnedToCore(
        &rc_transmitter_task,
        "RcTx100Hz",
        4096,
        nullptr,
        configMAX_PRIORITIES - 2,
        nullptr,
        1);
}

void loop() {
    // Core 0: Ponte MAVLink USB-OTG + Renderização paginada OLED + Alerta sonoro de bateria/link
    const uint32_t now_ms = millis();
    static uint32_t last_ack_seen = 0;
    static uint32_t last_hud_render_ms = 0;
    static uint32_t last_page_flush_ms = 0;

    const RxTelemShared rx_snap = g_rx_telem_lock.read();
    const bool linked = rx_snap.valid && ((now_ms - rx_snap.rx_ms) < 600u);

    if (rx_snap.valid && rx_snap.rx_ms > 0) {
        static uint32_t prev_telem_rx_ms = 0;
        if (rx_snap.rx_ms != prev_telem_rx_ms) {
            prev_telem_rx_ms = rx_snap.rx_ms;
            g_mav_bridge.on_drone_telemetry(rx_snap.pkt, now_ms);
        }
    }

    const uint32_t ack_cnt = g_param_ack_counter;
    if (ack_cnt != last_ack_seen) {
        last_ack_seen = ack_cnt;
        g_mav_bridge.on_drone_param_ack(g_rx_param_ack_lock.read());
    }

    ParamSyncPkt out_param{};
    if (g_mav_bridge.update(now_ms, out_param)) {
        g_tx_param_req_lock.write(out_param);
        g_tx_param_req_counter = g_tx_param_req_counter + 1u;
    }

    // Atualiza framebuffer do HUD a 12 Hz e envia 1 página (2.8 ms) a cada 10 ms
    if (now_ms - last_hud_render_ms >= 80u) {
        last_hud_render_ms = now_ms;
        const TxControlShared ctrl = g_tx_ctrl_lock.read();

        // Lê tensão da célula 18650 (divisor 100k + 100k -> x2)
        const float tx_vbat = (static_cast<float>(analogReadMilliVolts(kPinAdcTxBat)) * 2.0f) / 1000.0f;

        HudDisplayData hud{};
        hud.drone_linked = linked;
        hud.tx_rssi_dbm = rx_snap.pkt.drone_rssi_dbm;
        hud.tx_vbat_v = tx_vbat;
        hud.telem = rx_snap.pkt;
        hud.sticks = ctrl.sticks;
        hud.arm_switch_on = ctrl.arm_switch_on;
        hud.cal_wizard_step = ctrl.cal_step;
        g_oled.render_hud(hud);

        // Alarme sonoro intermitente no Controle se bateria do Drone < 10.6V ou Failsafe ativo!
        const bool low_drone_bat = linked && (rx_snap.pkt.vbat_mv > 5000u && rx_snap.pkt.vbat_mv < 10600u);
        const bool in_failsafe = linked && (rx_snap.pkt.state == static_cast<uint8_t>(SystemState::FAILSAFE_DESCEND) ||
                                            rx_snap.pkt.state == static_cast<uint8_t>(SystemState::FAILSAFE_LOCKED));
        if (low_drone_bat || in_failsafe) {
            digitalWrite(kPinBuzzer, ((now_ms / 200u) & 1u) ? HIGH : LOW);
        } else {
            digitalWrite(kPinBuzzer, LOW);
        }
    }

    if (now_ms - last_page_flush_ms >= 10u) {
        last_page_flush_ms = now_ms;
        g_oled.flush_next_page();
    }

    delay(2);
}
