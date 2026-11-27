#pragma once

#include <Wire.h>
#include <cstdint>

#include "fc_types.h"
#include "gimbal_mapper.h"
#include "radio_packets.h"

namespace fc::tx {

struct HudDisplayData {
    bool     drone_linked = false;
    int8_t   tx_rssi_dbm  = -100;
    float    tx_vbat_v    = 4.0f;
    proto::TelemFastPkt telem{};
    NormalizedSticks sticks{};
    bool     arm_switch_on = false;
    bool     nvs_save_toast = false;
    uint8_t  cal_wizard_step = 0; // 0=normal, 1=movendo extremos, 2=centrando
};

/**
 * Driver Gráfico Autônomo SSD1306 128x64 I2C com Atualização Incremental por Página.
 *
 * Transfere apenas 1 página (128 bytes = ~2.8 ms @ 400 kHz) por chamada de `flush_next_page()`,
 * garantindo que o barramento I2C seja suave e zero bibliotecas externas sejam exigidas!
 */
class OledHud {
public:
    bool begin(TwoWire& wire, int sda_pin = 8, int scl_pin = 9, uint8_t addr = 0x3C) noexcept;

    void render_hud(const HudDisplayData& d) noexcept;

    /**
     * Envia 1 das 8 páginas de 128 bytes ao display (ciclo completo = 8 chamadas).
     */
    void flush_next_page() noexcept;
    void flush_all() noexcept;

private:
    void clear_buffer() noexcept;
    void draw_pixel(int x, int y, bool on = true) noexcept;
    void draw_hline(int x, int y, int w) noexcept;
    void draw_rect(int x, int y, int w, int h, bool fill = false) noexcept;
    void draw_char(int x, int y, char c) noexcept;
    void draw_text(int x, int y, const char* str) noexcept;
    bool send_cmd(uint8_t cmd) noexcept;

    TwoWire* wire_ = nullptr;
    uint8_t addr_ = 0x3C;
    bool present_ = false;
    uint8_t page_cursor_ = 0;
    uint8_t buf_[1024] = {}; // 128 x 64 / 8
};

} // namespace fc::tx
