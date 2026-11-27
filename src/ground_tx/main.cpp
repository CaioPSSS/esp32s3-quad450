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
    int8_t local_rssi_dbm = -100;
    uint32_t rx_ms = 0;
    bool valid = false;
};

struct TxControlShared {
    NormalizedSticks sticks{};
    float tx_vbat_v = 4.0f;
    bool arm_switch_on = false;
    uint8_t cal_step = 0;
    uint32_t save_nvs_seq = 0;
};

SeqLock<RxTelemShared>   g_rx_telem_lock;
SeqLock<TxControlShared> g_tx_ctrl_lock;
SeqLock<ParamSyncPkt>    g_rx_param_ack_lock;
volatile uint32_t        g_param_ack_counter = 0;

SeqLock<ParamSyncPkt>    g_tx_param_req_lock;
volatile uint32_t        g_tx_param_req_counter = 0;

// Oversampling 16x no ADC1 de 12 bits p/ eliminar ruído de potenciômetro
// (Barramento superior dos gimbals possui resistor série 560R + 10uF -> máx <= 2.81V linear!)
uint16_t read_adc_oversampled(int pin) {
    uint32_t sum = 0;
    for (int i = 0; i < 16; ++i) {
        sum += static_cast<uint32_t>(analogRead(pin));
    }
    return static_cast<uint16_t>(sum >> 4);
}

// Leitura 4x oversampled da bateria 18650 (divisor 100k + 100k -> x2), executada na mesma task do ADC1!
float read_tx_battery_volts() {
    uint32_t mv_sum = 0;
    for (int i = 0; i < 4; ++i) {
        mv_sum += static_cast<uint32_t>(analogReadMilliVolts(kPinAdcTxBat));
    }
    return (static_cast<float>(mv_sum) * 0.5f) * 0.001f; // (mv_sum / 4) * 2.0 / 1000.0
}

FlightMode read_mode_switch() {
    const bool a_low = (digitalRead(kPinSwModeA) == LOW);
    const bool b_low = (digitalRead(kPinSwModeB) == LOW);
    if (a_low) return FlightMode::STABILIZE;
    if (b_low) return FlightMode::POS_HOLD; // Fase 1: CINE (AltHold suave 20 deg); Fase 2: GPS PosHold
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
            // Preserva pkt.drone_rssi_dbm (RSSI de Uplink medido no Drone!) e salva rssi_dbm local separado
            RxTelemShared snap{pkt, rssi_dbm, millis(), true};
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
// CORE 1: Task Determinística de 100 Hz (Leitura ADC1 Unificada + Chaves + TX)
// ============================================================================
void rc_transmitter_task(void* /*arg*/) {
    TickType_t last_wake = xTaskGetTickCount();
    uint16_t seq = 0;
    uint32_t seen_param_req = 0;
    uint8_t cal_step = 0;
    uint8_t vbat_div = 0;
    float tx_vbat_v = read_tx_battery_volts();
    uint16_t aux_hold_ticks = 0;
    bool aux_long_latched = false;
    uint32_t save_nvs_seq = 0;

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

        // Todas as leituras do periférico SAR ADC1 ocorrem sequencialmente nesta única task!
        const uint16_t raw_thr   = read_adc_oversampled(kPinAdcThrottle);
        const uint16_t raw_yaw   = read_adc_oversampled(kPinAdcYaw);
        const uint16_t raw_pitch = read_adc_oversampled(kPinAdcPitch);
        const uint16_t raw_roll  = read_adc_oversampled(kPinAdcRoll);
        if (++vbat_div >= 10u) { // 10 Hz p/ bateria 18650
            vbat_div = 0;
            tx_vbat_v = 0.85f * tx_vbat_v + 0.15f * read_tx_battery_volts();
        }

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
            TxControlShared st{{}, tx_vbat_v, false, cal_step, save_nvs_seq};
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
            TxControlShared st{{}, tx_vbat_v, false, cal_step, save_nvs_seq};
            g_tx_ctrl_lock.write(st);
            continue;
        }

        // Segurar [AUX] por 1.5s com chave ARM desligada solicita gravação dos ganhos na NVS do Drone!
        if (aux_pressed && !arm_sw_on) {
            if (++aux_hold_ticks >= 150u && !aux_long_latched) {
                aux_long_latched = true;
                ++save_nvs_seq;
            }
        } else {
            aux_hold_ticks = 0;
            if (!aux_pressed) {
                aux_long_latched = false;
            }
        }

        const NormalizedSticks sticks = g_mapper.map_raw(raw_thr, raw_yaw, raw_pitch, raw_roll);
        TxControlShared st{sticks, tx_vbat_v, arm_sw_on, 0, save_nvs_seq};
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

        // Se a ponte MAVLink no Core 0 enfileirou ajuste/leitura de parâmetro, envia logo em seguida
        const uint32_t cur_req = g_tx_param_req_counter;
        if (cur_req != seen_param_req) {
            seen_param_req = cur_req;
            const ParamSyncPkt ppkt = g_tx_param_req_lock.read();
            (void)g_radio.send(&ppkt, sizeof(ppkt));
        }
    }
}

// ============================================================================
// CORE 0: Ponte MAVLink USB-OTG + Renderização Paginada OLED + Alerta Sonoro
// (Inicializa Wire I2C dentro desta task p/ alocar ISR I2C na CPU 0!)
// ============================================================================
void core0_ui_task(void* /*arg*/) {
    g_oled.begin(Wire, kPinOledSda, kPinOledScl, 0x3C);

    uint32_t last_ack_seen = 0;
    uint32_t prev_telem_rx_ms = 0;
    uint32_t last_hud_render_ms = 0;
    uint32_t last_page_flush_ms = 0;
    uint32_t seen_save_nvs_seq = 0;
    uint32_t save_toast_until_ms = 0;
    uint32_t save_beep_until_ms = 0;

    for (;;) {
        const uint32_t now_ms = millis();
        const RxTelemShared rx_snap = g_rx_telem_lock.read();
        const TxControlShared ctrl = g_tx_ctrl_lock.read();
        const bool linked = rx_snap.valid && ((now_ms - rx_snap.rx_ms) < 600u);

        if (ctrl.save_nvs_seq != seen_save_nvs_seq) {
            seen_save_nvs_seq = ctrl.save_nvs_seq;
            g_mav_bridge.request_save_params_to_drone_nvs();
            save_toast_until_ms = now_ms + 1200u;
            save_beep_until_ms = now_ms + 120u;
        }

        if (rx_snap.valid && rx_snap.rx_ms > 0 && rx_snap.rx_ms != prev_telem_rx_ms) {
            prev_telem_rx_ms = rx_snap.rx_ms;
            g_mav_bridge.on_drone_telemetry(rx_snap.pkt, now_ms);
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

        // Atualiza framebuffer do HUD a 12.5 Hz e envia 1 página (2.8 ms) a cada 10 ms
        if (now_ms - last_hud_render_ms >= 80u) {
            last_hud_render_ms = now_ms;

            HudDisplayData hud{};
            hud.drone_linked = linked;
            hud.tx_rssi_dbm = rx_snap.local_rssi_dbm;
            hud.tx_vbat_v = ctrl.tx_vbat_v;
            hud.telem = rx_snap.pkt;
            hud.sticks = ctrl.sticks;
            hud.arm_switch_on = ctrl.arm_switch_on;
            hud.nvs_save_toast = (now_ms < save_toast_until_ms);
            hud.cal_wizard_step = ctrl.cal_step;
            g_oled.render_hud(hud);
        }

        // Alarme sonoro no Controle: bateria 3S < 10.6V, bateria TX < 3.45V, Failsafe ou confirmação NVS
        const bool low_drone_bat = linked && (rx_snap.pkt.vbat_mv > 5000u && rx_snap.pkt.vbat_mv < 10600u);
        const bool low_tx_bat = (ctrl.tx_vbat_v > 2.5f && ctrl.tx_vbat_v < 3.45f);
        const bool in_failsafe = linked && (rx_snap.pkt.state == static_cast<uint8_t>(SystemState::FAILSAFE_DESCEND) ||
                                            rx_snap.pkt.state == static_cast<uint8_t>(SystemState::FAILSAFE_LOCKED));
        if (now_ms < save_beep_until_ms) {
            digitalWrite(kPinBuzzer, HIGH);
        } else if (low_drone_bat || low_tx_bat || in_failsafe) {
            digitalWrite(kPinBuzzer, ((now_ms / 200u) & 1u) ? HIGH : LOW);
        } else {
            digitalWrite(kPinBuzzer, LOW);
        }

        if (now_ms - last_page_flush_ms >= 10u) {
            last_page_flush_ms = now_ms;
            g_oled.flush_next_page();
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

} // namespace

void setup() {
    Serial.begin(115200);       // Porta USB-C Nativa (MAVLink v2 USB-OTG p/ QGroundControl!)
    Serial.setTxTimeoutMs(0);   // Nunca bloqueia o HUD se usado s/ cabo USB ou em powerbank!

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    pinMode(kPinSwArmKill, INPUT_PULLUP);
    pinMode(kPinSwModeA,   INPUT_PULLUP);
    pinMode(kPinSwModeB,   INPUT_PULLUP);
    pinMode(kPinBtnAux,    INPUT_PULLUP);
    pinMode(kPinBuzzer,    OUTPUT);
    digitalWrite(kPinBuzzer, LOW);

    load_calibration_nvs();
    g_mav_bridge.begin(Serial);

    g_radio.set_rx_callback(&on_espnow_packet);
    g_radio.begin(kWifiLrChannel, true);

    // Fixa UI (OLED I2C + MAVLink USB) no Core 0 e Transmissor RC 100 Hz no Core 1
    xTaskCreatePinnedToCore(
        &core0_ui_task,
        "UiCore0",
        6144,
        nullptr,
        2,
        nullptr,
        0);

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
    vTaskDelay(pdMS_TO_TICKS(1000));
}
