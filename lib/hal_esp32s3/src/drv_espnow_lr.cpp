#include "drv_espnow_lr.h"

#include <Arduino.h>
#include <WiFi.h>
#include <cstring>
#include <esp_idf_version.h>
#include <esp_now.h>
#include <esp_wifi.h>

namespace fc::hal {

namespace {
constexpr uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

volatile int8_t   g_last_rssi_dbm = -100;
volatile uint32_t g_tx_ok_count   = 0;
volatile uint32_t g_tx_fail_count = 0;
EspNowRxHandler   g_user_rx_cb    = nullptr;

// Sniffer passivo ultraleve apenas para capturar o RSSI real da camada física 802.11
void IRAM_ATTR promiscuous_rssi_sniffer(void* buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) {
        return;
    }
    const auto* pkt = static_cast<const wifi_promiscuous_pkt_t*>(buf);
    if (pkt) {
        g_last_rssi_dbm = static_cast<int8_t>(pkt->rx_ctrl.rssi);
    }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void on_espnow_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (!info || !data || len <= 0) {
        return;
    }
    if (info->rx_ctrl) {
        g_last_rssi_dbm = static_cast<int8_t>(info->rx_ctrl->rssi);
    }
    if (g_user_rx_cb) {
        g_user_rx_cb(info->src_addr, data, static_cast<size_t>(len), g_last_rssi_dbm);
    }
}
#else
void on_espnow_recv(const uint8_t* mac_addr, const uint8_t* data, int len) {
    if (!mac_addr || !data || len <= 0) {
        return;
    }
    if (g_user_rx_cb) {
        g_user_rx_cb(mac_addr, data, static_cast<size_t>(len), g_last_rssi_dbm);
    }
}
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void on_espnow_send(const wifi_tx_info_t* /*info*/, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        g_tx_ok_count = g_tx_ok_count + 1u;
    } else {
        g_tx_fail_count = g_tx_fail_count + 1u;
    }
}
#else
void on_espnow_send(const uint8_t* /*mac_addr*/, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        g_tx_ok_count = g_tx_ok_count + 1u;
    } else {
        g_tx_fail_count = g_tx_fail_count + 1u;
    }
}
#endif

} // namespace

bool EspNowLrLink::begin(uint8_t wifi_channel, bool enable_lr_mode) noexcept {
    channel_ = (wifi_channel >= 1 && wifi_channel <= 13) ? wifi_channel : 1;

    WiFi.persistent(false);
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_STA);

    // Desativa economia de energia do rádio Wi-Fi (essencial p/ latência < 2 ms!)
    esp_wifi_set_ps(WIFI_PS_NONE);

    // Ativa protocolo proprietário Long Range (802.11 LR) da Espressif se desejado
    const uint8_t proto_mask = enable_lr_mode
                                   ? (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR)
                                   : (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    esp_wifi_set_protocol(WIFI_IF_STA, proto_mask);

    // Fixa o canal de operação e potência máxima (+19.5 dBm = 78 unidades de 0.25 dBm)
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(channel_, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous_rx_cb(&promiscuous_rssi_sniffer);
    wifi_promiscuous_filter_t filt{};
    filt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT; // ESP-NOW usa Vendor Action Management Frames!
    esp_wifi_set_promiscuous_filter(&filt);
    esp_wifi_set_max_tx_power(78);

    if (esp_now_init() != ESP_OK) {
        return false;
    }

    esp_now_register_recv_cb(&on_espnow_recv);
    esp_now_register_send_cb(&on_espnow_send);

    // Adiciona peer de Broadcast inicial para descoberta / Auto-Bind sem recompilar MAC
    esp_now_peer_info_t bcast{};
    std::memcpy(bcast.peer_addr, kBroadcastMac, 6);
    bcast.channel = channel_;
    bcast.ifidx = WIFI_IF_STA;
    bcast.encrypt = false;
    if (!esp_now_is_peer_exist(kBroadcastMac)) {
        (void)esp_now_add_peer(&bcast);
    }

    std::memcpy(peer_mac_, kBroadcastMac, 6);
    has_peer_ = false;
    return true;
}

void EspNowLrLink::set_rx_callback(EspNowRxHandler cb) noexcept {
    g_user_rx_cb = cb;
}

bool EspNowLrLink::register_peer(const uint8_t peer_mac[6]) noexcept {
    if (!peer_mac) {
        return false;
    }
    if (std::memcmp(peer_mac, kBroadcastMac, 6) == 0) {
        return false;
    }

    if (!esp_now_is_peer_exist(peer_mac)) {
        esp_now_peer_info_t p{};
        std::memcpy(p.peer_addr, peer_mac, 6);
        p.channel = channel_;
        p.ifidx = WIFI_IF_STA;
        p.encrypt = false;
        if (esp_now_add_peer(&p) != ESP_OK) {
            return false;
        }
    }

    std::memcpy(peer_mac_, peer_mac, 6);
    has_peer_ = true;
    return true;
}

bool EspNowLrLink::send(const void* data, size_t len) noexcept {
    if (!data || len == 0 || len > 250) {
        return false;
    }
    const uint8_t* target = has_peer_ ? peer_mac_ : kBroadcastMac;
    return esp_now_send(target, static_cast<const uint8_t*>(data), len) == ESP_OK;
}

int8_t EspNowLrLink::last_rssi_dbm() const noexcept {
    return g_last_rssi_dbm;
}

uint32_t EspNowLrLink::tx_ok_count() const noexcept {
    return g_tx_ok_count;
}

uint32_t EspNowLrLink::tx_fail_count() const noexcept {
    return g_tx_fail_count;
}

} // namespace fc::hal
