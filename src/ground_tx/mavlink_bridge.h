#pragma once

#include <Stream.h>
#include <cstdint>

#include "fc_types.h"
#include "mavlink_v2_min.h"
#include "param_registry.h"
#include "radio_packets.h"

namespace fc::tx {

/**
 * Ponte Bidirecional ESP-NOW <-> MAVLink v2 (USB-OTG p/ QGroundControl no Celular/Tablet).
 *
 * Responsabilidades:
 * 1. Converte `TelemFastPkt` do drone em `HEARTBEAT`, `SYS_STATUS`, `ATTITUDE`, `VFR_HUD`,
 *    `SERVO_OUTPUT_RAW` e alertas `STATUSTEXT` claros de Pre-Arm/Failsafe!
 * 2. Sincroniza os 15 parâmetros de voo (`FlightParams`) com o drone via `ParamSyncPkt`
 *    com confirmação ACK e retransmissão automática caso um pacote de rádio se perca!
 */
class MavlinkUsbBridge {
public:
    void begin(Stream& usb_stream) noexcept { usb_ = &usb_stream; }

    void on_drone_telemetry(const proto::TelemFastPkt& pkt, uint32_t now_ms) noexcept;
    void on_drone_param_ack(const proto::ParamSyncPkt& pkt) noexcept;

    /**
     * Lê bytes da USB-OTG (QGroundControl), emite telemetria periódica e retorna `true`
     * se houver um `ParamSyncPkt` pendente para transmitir ao drone via ESP-NOW.
     */
    bool update(uint32_t now_ms, proto::ParamSyncPkt& out_espnow_param_pkt) noexcept;

    void request_save_params_to_drone_nvs() noexcept;

    const proto::FlightParams& shadow_params() const noexcept { return shadow_params_; }

private:
    template <typename MsgT>
    void send_mav(uint32_t msgid, const MsgT& payload) noexcept;
    void send_param_to_gcs(uint8_t idx) noexcept;
    void send_status_text(uint8_t severity, const char* msg) noexcept;

    Stream* usb_ = nullptr;
    uint8_t tx_seq_ = 0;
    mav::MavlinkV2Parser parser_{};

    proto::TelemFastPkt last_telem_{};
    bool has_telem_ = false;
    uint32_t last_telem_ms_ = 0;
    uint8_t prev_drone_state_ = 0xFF;
    uint16_t prev_prearm_flags_ = 0xFFFF;

    uint32_t last_hb_ms_ = 0;
    uint32_t last_sys_ms_ = 0;
    uint32_t last_att_ms_ = 0;
    uint32_t last_hud_ms_ = 0;

    // Sincronização confiável de parâmetros com o drone via ACK/Retry
    proto::FlightParams shadow_params_{};
    bool pending_tx_ = false;
    proto::ParamSyncPkt pending_pkt_{};
    uint32_t last_param_tx_ms_ = 0;
    uint8_t retry_count_ = 0;
    uint8_t next_req_id_ = 1;
};

} // namespace fc::tx
