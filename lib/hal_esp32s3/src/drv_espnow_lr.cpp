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
volatile bool     g_has_bound_mac = false;
uint8_t           g_bound_mac[6]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
EspNowRxHandler   g_user_rx_cb    = nullptr;

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
// Fallback apenas para ESP-IDF 4.x onde esp_now_recv_info_t não expunha rx_ctrl->rssi
void IRAM_ATTR promiscuous_rssi_sniffer(void* buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) {
        return;
    }
    const auto* pkt = static_cast<const wifi_promiscuous_pkt_t*>(buf);
    if (pkt) {
        g_last_rssi_dbm = static_cast<int8_t>(pkt->rx_ctrl.rssi);
    }
}
#endif

void configure_peer_lr_rate(const uint8_t peer_addr[6], bool enable_lr_mode) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
    if (!enable_lr_mode || !peer_addr) {
        return;
    }
    esp_now_rate_config_t rcfg{};
    rcfg.phymode = WIFI_PHY_MODE_LR;
    rcfg.rate    = WIFI_PHY_RATE_LORA_250K;
    rcfg.ersu    = false;
    rcfg.dcm     = false;
    (void)esp_now_set_peer_rate_config(peer_addr, &rcfg);
#else
    (void)peer_addr;
    (void)enable_lr_mode;
#endif
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void on_espnow_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (!info || !info->src_addr || !data || len <= 0) {
        return;
    }
    int8_t pkt_rssi = g_last_rssi_dbm;
    if (info->rx_ctrl) {
        pkt_rssi = static_cast<int8_t>(info->rx_ctrl->rssi);
        if (!g_has_bound_mac || std::memcmp(info->src_addr, g_bound_mac, 6) == 0) {
            g_last_rssi_dbm = pkt_rssi;
        }
    }
    if (g_user_rx_cb) {
        g_user_rx_cb(info->src_addr, data, static_cast<size_t>(len), pkt_rssi);
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
    lr_mode_ = enable_lr_mode;
    g_has_bound_mac = false;
    std::memcpy(g_bound_mac, kBroadcastMac, 6);

    WiFi.persistent(false);
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_STA);

    // Desativa economia de energia do rádio Wi-Fi (essencial p/ latência < 2 ms!)
    esp_wifi_set_ps(WIFI_PS_NONE);

    // Quando enable_lr_mode=true, ativa WIFI_PROTOCOL_LR puro para forçar modulação DSSS 250 kbps
    // (-105 dBm de sensibilidade) já nos quadros Broadcast de Auto-Bind!
    const uint8_t proto_mask = enable_lr_mode
                                   ? WIFI_PROTOCOL_LR
                                   : (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    esp_wifi_set_protocol(WIFI_IF_STA, proto_mask);

    // Fixa o canal de operação e potência máxima (+19.5 dBm = 78 unidades de 0.25 dBm)
    esp_wifi_set_channel(channel_, WIFI_SECOND_CHAN_NONE);
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&promiscuous_rssi_sniffer);
    wifi_promiscuous_filter_t filt{};
    filt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
    esp_wifi_set_promiscuous_filter(&filt);
#endif
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
    configure_peer_lr_rate(kBroadcastMac, lr_mode_);

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
    configure_peer_lr_rate(peer_mac, lr_mode_);

    std::memcpy(peer_mac_, peer_mac, 6);
    std::memcpy(g_bound_mac, peer_mac, 6);
    g_has_bound_mac = true;
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
