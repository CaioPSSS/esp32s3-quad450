#include "mavlink_bridge.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace fc::tx {

using namespace fc::mav;
using namespace fc::proto;

template <typename MsgT>
void MavlinkUsbBridge::send_mav(uint32_t msgid, const MsgT& payload) noexcept {
    if (!usb_) return;
    uint8_t frame[280];
    const size_t n = encode_mavlink_v2(msgid, 1, 1, tx_seq_, payload, frame);
    if (n > 0) {
        usb_->write(frame, n);
    }
}

void MavlinkUsbBridge::send_status_text(uint8_t severity, const char* msg) noexcept {
    MsgStatusText st{};
    st.severity = severity;
    std::snprintf(st.text, sizeof(st.text), "%s", msg ? msg : "");
    send_mav(MSG_ID_STATUSTEXT, st);
}

void MavlinkUsbBridge::send_param_to_gcs(uint8_t idx) noexcept {
    float val = 0.0f;
    if (!get_param_by_index(shadow_params_, idx, val)) {
        return;
    }
    MsgParamValue pv{};
    pv.param_value = val;
    pv.param_count = PARAM_COUNT;
    pv.param_index = idx;
    const size_t nlen = std::strlen(kParamTable[idx].name);
    std::memcpy(pv.param_id, kParamTable[idx].name, std::min<size_t>(nlen, 16u));
    pv.param_type = MAV_PARAM_TYPE_REAL32;
    send_mav(MSG_ID_PARAM_VALUE, pv);
}

void MavlinkUsbBridge::on_drone_telemetry(const TelemFastPkt& pkt, uint32_t now_ms) noexcept {
    last_telem_ = pkt;
    has_telem_ = true;
    last_telem_ms_ = now_ms;

    if (pkt.state != prev_drone_state_) {
        prev_drone_state_ = pkt.state;
        char text[50];
        std::snprintf(text, sizeof(text), "Drone State -> %u (PreArm=0x%02X)",
                      static_cast<unsigned>(pkt.state),
                      static_cast<unsigned>(pkt.prearm_flags));
        send_status_text(MAV_SEVERITY_NOTICE, text);
    } else if (pkt.state == static_cast<uint8_t>(SystemState::DISARMED) &&
               pkt.prearm_flags != prev_prearm_flags_) {
        prev_prearm_flags_ = pkt.prearm_flags;
        if (pkt.prearm_flags & ERR_THROTTLE_NOT_ZERO) {
            send_status_text(MAV_SEVERITY_WARNING, "PreArm: Baixe o Throttle a zero!");
        } else if (pkt.prearm_flags & ERR_ARM_SWITCH_BOOT_ON) {
            send_status_text(MAV_SEVERITY_WARNING, "PreArm: Desligue chave ARM (OFF->ON)");
        } else if (pkt.prearm_flags & ERR_TILT_TOO_HIGH) {
            send_status_text(MAV_SEVERITY_WARNING, "PreArm: Nivele o drone (< 10 graus)");
        } else if (pkt.prearm_flags & ERR_BATTERY_LOW) {
            send_status_text(MAV_SEVERITY_WARNING, "PreArm: Bateria 3S abaixo de 10.8V!");
        }
    }
}

void MavlinkUsbBridge::on_drone_param_ack(const ParamSyncPkt& pkt) noexcept {
    if (pkt.op == PARAM_OP_ACK && pkt.param_idx < PARAM_COUNT) {
        if (pkt.status == 0) {
            (void)set_param_by_index(shadow_params_, pkt.param_idx, pkt.value);
            send_param_to_gcs(pkt.param_idx);
        }
        if (pending_tx_ && pkt.req_id == pending_pkt_.req_id) {
            pending_tx_ = false;
            if (pending_pkt_.op == PARAM_OP_GET && pkt.status == 0 && pkt.param_idx == initial_pull_idx_) {
                ++initial_pull_idx_;
                if (initial_pull_idx_ >= PARAM_COUNT) {
                    synced_from_drone_ = true;
                }
            }
        }
    } else if (pkt.op == PARAM_OP_SAVE_NVS && pending_tx_ && pkt.req_id == pending_pkt_.req_id) {
        pending_tx_ = false;
        if (pkt.status == 0) {
            send_status_text(MAV_SEVERITY_INFO, "Parametros gravados na NVS do Drone!");
        } else {
            send_status_text(MAV_SEVERITY_WARNING, "NVS bloqueado: desarme o drone antes!");
        }
    }
}

void MavlinkUsbBridge::request_save_params_to_drone_nvs() noexcept {
    pending_pkt_ = ParamSyncPkt{};
    pending_pkt_.type = PKT_PARAM_SYNC;
    pending_pkt_.req_id = next_req_id_++;
    pending_pkt_.op = PARAM_OP_SAVE_NVS;
    seal_packet(pending_pkt_);
    pending_tx_ = true;
    retry_count_ = 0;
    last_param_tx_ms_ = 0;
}

bool MavlinkUsbBridge::update(uint32_t now_ms, ParamSyncPkt& out_espnow_param_pkt) noexcept {
    // 1. Lê bytes vindos do QGroundControl na porta USB-OTG
    if (usb_) {
        int budget = 128;
        ParsedMavMessage rx{};
        while (budget-- > 0 && usb_->available() > 0) {
            const uint8_t b = static_cast<uint8_t>(usb_->read());
            if (!parser_.push_byte(b, rx)) {
                continue;
            }

            if (rx.msgid == MSG_ID_PARAM_REQUEST_LIST) {
                gcs_list_send_idx_ = 0;
            } else if (rx.msgid == MSG_ID_PARAM_REQUEST_READ) {
                const auto req = rx.unpack<MsgParamRequestRead>();
                int idx = req.param_index;
                if (idx < 0) idx = find_param_by_name(req.param_id);
                if (idx >= 0 && idx < PARAM_COUNT) {
                    send_param_to_gcs(static_cast<uint8_t>(idx));
                }
            } else if (rx.msgid == MSG_ID_PARAM_SET) {
                const auto ps = rx.unpack<MsgParamSet>();
                const int idx = find_param_by_name(ps.param_id);
                if (idx >= 0 && set_param_by_index(shadow_params_, static_cast<uint8_t>(idx), ps.param_value)) {
                    float clamped_val = ps.param_value;
                    (void)get_param_by_index(shadow_params_, static_cast<uint8_t>(idx), clamped_val);

                    // Agenda envio confiável (c/ retry até ACK) para o Drone via ESP-NOW!
                    pending_pkt_ = ParamSyncPkt{};
                    pending_pkt_.type = PKT_PARAM_SYNC;
                    pending_pkt_.req_id = next_req_id_++;
                    pending_pkt_.op = PARAM_OP_SET;
                    pending_pkt_.param_idx = static_cast<uint8_t>(idx);
                    pending_pkt_.value = clamped_val;
                    seal_packet(pending_pkt_);
                    pending_tx_ = true;
                    retry_count_ = 0;
                    last_param_tx_ms_ = 0;

                    send_param_to_gcs(static_cast<uint8_t>(idx));
                }
            } else if (rx.msgid == MSG_ID_COMMAND_LONG) {
                const auto cmd = rx.unpack<MsgCommandLong>();
                MsgCommandAck ack{};
                ack.command = cmd.command;
                ack.target_system = rx.sysid;
                ack.target_component = rx.compid;

                if (cmd.command == MAV_CMD_PREFLIGHT_STORAGE && cmd.param1 >= 0.5f) {
                    request_save_params_to_drone_nvs();
                    ack.result = MAV_RESULT_ACCEPTED;
                    send_mav(MSG_ID_COMMAND_ACK, ack);
                } else {
                    // Segurança soberana: recusa armamento pelo tablet, orientando usar a chave física!
                    ack.result = MAV_RESULT_DENIED;
                    send_mav(MSG_ID_COMMAND_ACK, ack);
                    if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                        send_status_text(MAV_SEVERITY_WARNING, "Use a chave fisica ARM no Controle!");
                    }
                }
            }
        }
    }

    // Streaming cadenciado de PARAM_VALUE (2 quadros = 74 B por tick) p/ não saturar FIFO USB-CDC
    if (gcs_list_send_idx_ < PARAM_COUNT) {
        for (int k = 0; k < 2 && gcs_list_send_idx_ < PARAM_COUNT; ++k) {
            send_param_to_gcs(gcs_list_send_idx_++);
        }
    }

    const bool link_fresh = has_telem_ && ((now_ms - last_telem_ms_) < 1000u);

    // Sincronização automática inicial: puxa todos os 19 parâmetros reais da NVS do drone ao parear!
    if (link_fresh && !synced_from_drone_ && !pending_tx_ && initial_pull_idx_ < PARAM_COUNT) {
        pending_pkt_ = ParamSyncPkt{};
        pending_pkt_.type = PKT_PARAM_SYNC;
        pending_pkt_.req_id = next_req_id_++;
        pending_pkt_.op = PARAM_OP_GET;
        pending_pkt_.param_idx = initial_pull_idx_;
        seal_packet(pending_pkt_);
        pending_tx_ = true;
        retry_count_ = 0;
        last_param_tx_ms_ = 0;
    }

    // 2. HEARTBEAT (1 Hz)
    if (now_ms - last_hb_ms_ >= 1000u) {
        last_hb_ms_ = now_ms;
        MsgHeartbeat hb{};
        hb.custom_mode = last_telem_.flight_mode;
        hb.base_mode = MAV_MODE_FLAG_CUSTOM_MODE | MAV_MODE_FLAG_STABILIZE | MAV_MODE_FLAG_MANUAL_INPUT;
        const bool armed = (last_telem_.state == static_cast<uint8_t>(SystemState::ARMED_IDLE) ||
                            last_telem_.state == static_cast<uint8_t>(SystemState::FLYING) ||
                            last_telem_.state == static_cast<uint8_t>(SystemState::FAILSAFE_DESCEND));
        if (armed) hb.base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
        hb.system_status = !link_fresh ? MAV_STATE_UNINIT : (armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
        send_mav(MSG_ID_HEARTBEAT, hb);
    }

    if (link_fresh) {
        // 3. SYS_STATUS (2 Hz)
        if (now_ms - last_sys_ms_ >= 500u) {
            last_sys_ms_ = now_ms;
            MsgSysStatus sys{};
            constexpr uint32_t kGyroAccelBaro = 0x01u | 0x02u | 0x08u;
            sys.onboard_control_sensors_present = kGyroAccelBaro;
            sys.onboard_control_sensors_enabled = kGyroAccelBaro;
            sys.onboard_control_sensors_health  = (last_telem_.prearm_flags & ERR_IMU_UNHEALTHY) ? 0u : kGyroAccelBaro;
            sys.load = static_cast<uint16_t>(last_telem_.cpu_load_pct) * 10u;
            sys.voltage_battery = last_telem_.vbat_mv;
            const float v = static_cast<float>(last_telem_.vbat_mv) / 1000.0f;
            sys.battery_remaining = static_cast<int8_t>(
                std::clamp((v - 10.2f) / (12.6f - 10.2f) * 100.0f, 0.0f, 100.0f));
            send_mav(MSG_ID_SYS_STATUS, sys);
        }

        // 4. ATTITUDE (20 Hz)
        if (now_ms - last_att_ms_ >= 50u) {
            last_att_ms_ = now_ms;
            MsgAttitude att{};
            att.time_boot_ms = now_ms;
            att.roll  = (static_cast<float>(last_telem_.roll_deg10)  * 0.1f) * kDegToRad;
            att.pitch = (static_cast<float>(last_telem_.pitch_deg10) * 0.1f) * kDegToRad;
            att.yaw   = (static_cast<float>(last_telem_.yaw_deg10)   * 0.1f) * kDegToRad;
            send_mav(MSG_ID_ATTITUDE, att);
        }

        // 5. VFR_HUD + SERVO_OUTPUT_RAW (10 Hz)
        if (now_ms - last_hud_ms_ >= 100u) {
            last_hud_ms_ = now_ms;
            MsgVfrHud hud{};
            hud.alt = static_cast<float>(last_telem_.alt_cm) * 0.01f;
            hud.climb = static_cast<float>(last_telem_.vz_cms) * 0.01f;
            int16_t hdg = last_telem_.yaw_deg10 / 10;
            if (hdg < 0) hdg += 360;
            hud.heading = hdg;
            hud.throttle = (last_telem_.motor_pct[0] + last_telem_.motor_pct[1] +
                            last_telem_.motor_pct[2] + last_telem_.motor_pct[3]) / 4u;
            send_mav(MSG_ID_VFR_HUD, hud);

            MsgServoOutputRaw srv{};
            srv.time_usec = now_ms * 1000u;
            srv.servo1_raw = 1000u + static_cast<uint16_t>(last_telem_.motor_pct[0]) * 10u;
            srv.servo2_raw = 1000u + static_cast<uint16_t>(last_telem_.motor_pct[1]) * 10u;
            srv.servo3_raw = 1000u + static_cast<uint16_t>(last_telem_.motor_pct[2]) * 10u;
            srv.servo4_raw = 1000u + static_cast<uint16_t>(last_telem_.motor_pct[3]) * 10u;
            send_mav(MSG_ID_SERVO_OUTPUT_RAW, srv);
        }
    }

    // 6. Retransmissão confiável de parâmetros para o Drone (até 5 tentativas a cada 120 ms)
    if (pending_tx_ && (last_param_tx_ms_ == 0u || (now_ms - last_param_tx_ms_) >= 120u)) {
        last_param_tx_ms_ = now_ms;
        out_espnow_param_pkt = pending_pkt_;
        if (++retry_count_ >= 5u) {
            pending_tx_ = false;
        }
        return true;
    }

    return false;
}

} // namespace fc::tx
