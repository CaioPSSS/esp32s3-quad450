#pragma once

#include <cstddef>
#include <cstdint>

namespace fc::hal {

using EspNowRxHandler = void (*)(const uint8_t mac[6],
                                 const uint8_t* data,
                                 size_t len,
                                 int8_t rssi_dbm);

/**
 * Driver compartilhado ESP-NOW Long Range (WIFI_PROTOCOL_LR @ 19.5 dBm) para ESP32-S3.
 *
 * Recursos:
 * - Configura Wi-Fi STA sem conectar a roteador, trava canal fixo e ativa PHY Long Range
 *   da Espressif (~500 m a 1+ km com antenas externas IPEX 3-5 dBi!).
 * - Captura o RSSI real em dBm de cada pacote recebido via filtro promíscuo leve.
 * - Suporta Auto-Bind (inicia em Broadcast FF:FF:FF:FF:FF:FF e registra peer Unicast
 *   automaticamente quando solicitado após validar pacote com CRC16!).
 */
class EspNowLrLink {
public:
    bool begin(uint8_t wifi_channel = 1, bool enable_lr_mode = true) noexcept;

    void set_rx_callback(EspNowRxHandler cb) noexcept;

    bool register_peer(const uint8_t peer_mac[6]) noexcept;
    bool has_bound_peer() const noexcept { return has_peer_; }
    const uint8_t* bound_peer_mac() const noexcept { return peer_mac_; }

    /**
     * Envia payload para o peer pareado (ou Broadcast se ainda em busca de Auto-Bind).
     */
    bool send(const void* data, size_t len) noexcept;

    int8_t last_rssi_dbm() const noexcept;
    uint32_t tx_ok_count() const noexcept;
    uint32_t tx_fail_count() const noexcept;

private:
    uint8_t channel_ = 1;
    uint8_t peer_mac_[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    bool has_peer_ = false;
};

} // namespace fc::hal
