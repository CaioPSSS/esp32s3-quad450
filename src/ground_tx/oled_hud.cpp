#include "oled_hud.h"

#include <Arduino.h>
#include <algorithm>
#include <cstdio>
#include <cstring>

namespace fc::tx {

namespace {

// Fonte compacta 5x7 ASCII para caracteres 32..90 (' ' até 'Z' + símbolos essenciais)
const uint8_t kFont5x7[][5] PROGMEM = {
    {0x00,0x00,0x00,0x00,0x00}, // 32 ' '
    {0x00,0x00,0x5F,0x00,0x00}, // 33 '!'
    {0x00,0x07,0x00,0x07,0x00}, // 34 '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // 35 '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // 36 '$'
    {0x23,0x13,0x08,0x64,0x62}, // 37 '%'
    {0x36,0x49,0x55,0x22,0x50}, // 38 '&'
    {0x00,0x05,0x03,0x00,0x00}, // 39 '''
    {0x00,0x1C,0x22,0x41,0x00}, // 40 '('
    {0x00,0x41,0x22,0x1C,0x00}, // 41 ')'
    {0x14,0x08,0x3E,0x08,0x14}, // 42 '*'
    {0x08,0x08,0x3E,0x08,0x08}, // 43 '+'
    {0x00,0x50,0x30,0x00,0x00}, // 44 ','
    {0x08,0x08,0x08,0x08,0x08}, // 45 '-'
    {0x00,0x60,0x60,0x00,0x00}, // 46 '.'
    {0x20,0x10,0x08,0x04,0x02}, // 47 '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // 48 '0'
    {0x00,0x42,0x7F,0x40,0x00}, // 49 '1'
    {0x42,0x61,0x51,0x49,0x46}, // 50 '2'
    {0x21,0x41,0x45,0x4B,0x31}, // 51 '3'
    {0x18,0x14,0x12,0x7F,0x10}, // 52 '4'
    {0x27,0x45,0x45,0x45,0x39}, // 53 '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // 54 '6'
    {0x01,0x71,0x09,0x05,0x03}, // 55 '7'
    {0x36,0x49,0x49,0x49,0x36}, // 56 '8'
    {0x06,0x49,0x49,0x29,0x1E}, // 57 '9'
    {0x00,0x36,0x36,0x00,0x00}, // 58 ':'
    {0x00,0x56,0x36,0x00,0x00}, // 59 ';'
    {0x08,0x14,0x22,0x41,0x00}, // 60 '<'
    {0x14,0x14,0x14,0x14,0x14}, // 61 '='
    {0x00,0x41,0x22,0x14,0x08}, // 62 '>'
    {0x02,0x01,0x51,0x09,0x06}, // 63 '?'
    {0x32,0x49,0x79,0x41,0x3E}, // 64 '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 65 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 66 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 67 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 68 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 69 'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 70 'F'
    {0x3E,0x41,0x49,0x49,0x7A}, // 71 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 72 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 73 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 74 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 75 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 76 'L'
    {0x7F,0x02,0x0C,0x02,0x7F}, // 77 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 78 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 79 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 80 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 81 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 82 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 83 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 84 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 85 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 86 'V'
    {0x3F,0x40,0x38,0x40,0x3F}, // 87 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 88 'X'
    {0x07,0x08,0x70,0x08,0x07}, // 89 'Y'
    {0x61,0x51,0x49,0x45,0x43}  // 90 'Z'
};

const char* short_state_str(uint8_t s) {
    switch (static_cast<SystemState>(s)) {
        case SystemState::BOOT_CALIBRATING: return "CAL-GYRO";
        case SystemState::DISARMED:         return "DISARMED";
        case SystemState::BENCH_DIAGNOSTIC: return "BENCH";
        case SystemState::ARMED_IDLE:       return "ARM-IDLE";
        case SystemState::FLYING:           return "FLYING!";
        case SystemState::FAILSAFE_DESCEND: return "FS-DESC!";
        case SystemState::FAILSAFE_LOCKED:  return "FS-LOCK!";
    }
    return "WAIT-LINK";
}

const char* short_mode_str(uint8_t m) {
    switch (static_cast<FlightMode>(m)) {
        case FlightMode::STABILIZE: return "STAB";
        case FlightMode::ALT_HOLD:  return "ALTH";
        case FlightMode::POS_HOLD:  return "CINE"; // Fase 1: ALT_HOLD suave (20 deg max)
        case FlightMode::RTH:       return "CINE";
    }
    return "STAB";
}

const char* first_prearm_reason(uint16_t flags) {
    if (flags == PREARM_OK)                return "OK P/ ARMAR";
    if (flags & ERR_IMU_UNHEALTHY)         return "ERR:IMU FALHA";
    if (flags & ERR_GYRO_UNCALIBRATED)     return "AGUARDE CAL..";
    if (flags & ERR_RC_LINK_LOST)          return "SEM SINAL RX";
    if (flags & ERR_ARM_SWITCH_BOOT_ON)    return "DESL CHAVE ARM";
    if (flags & ERR_THROTTLE_NOT_ZERO)     return "BAIXE THROTTLE";
    if (flags & ERR_TILT_TOO_HIGH)         return "NIVELE DRONE";
    if (flags & ERR_BATTERY_LOW)           return "BATERIA BAIXA!";
    if (flags & ERR_BENCH_MODE_ACTIVE)     return "MODO BANCADA";
    return "PREARM BLOQ";
}

} // namespace

bool OledHud::send_cmd(uint8_t cmd) noexcept {
    if (!wire_) return false;
    wire_->beginTransmission(addr_);
    wire_->write(0x00); // Co=0, D/C#=0 (command)
    wire_->write(cmd);
    return wire_->endTransmission() == 0;
}

bool OledHud::begin(TwoWire& wire, int sda_pin, int scl_pin, uint8_t addr) noexcept {
    wire_ = &wire;
    addr_ = addr;
    wire_->begin(sda_pin, scl_pin, 400000u);

    // Sequência de inicialização padrão SSD1306 128x64 (charge pump interno 3.3V)
    const uint8_t init_seq[] = {
        0xAE,       // Display OFF
        0xD5, 0x80, // Clock divide
        0xA8, 0x3F, // Multiplex 64
        0xD3, 0x00, // Offset 0
        0x40,       // Start line 0
        0x8D, 0x14, // Charge pump ON
        0x20, 0x02, // Page addressing mode
        0xA1,       // Segment remap
        0xC8,       // COM scan dec
        0xDA, 0x12, // COM pins 128x64
        0x81, 0xCF, // Contrast
        0xD9, 0xF1, // Precharge
        0xDB, 0x40, // VCOMH
        0xA4,       // Resume RAM
        0xA6,       // Normal display
        0xAF        // Display ON
    };

    present_ = true;
    for (uint8_t c : init_seq) {
        if (!send_cmd(c)) {
            present_ = false;
            return false;
        }
    }
    clear_buffer();
    flush_all();
    return true;
}

void OledHud::clear_buffer() noexcept {
    std::memset(buf_, 0, sizeof(buf_));
}

void OledHud::draw_pixel(int x, int y, bool on) noexcept {
    if (x < 0 || x >= 128 || y < 0 || y >= 64) return;
    const size_t idx = static_cast<size_t>((y / 8) * 128 + x);
    const uint8_t mask = static_cast<uint8_t>(1u << (y & 7));
    if (on) buf_[idx] |= mask;
    else    buf_[idx] &= static_cast<uint8_t>(~mask);
}

void OledHud::draw_hline(int x, int y, int w) noexcept {
    for (int i = 0; i < w; ++i) {
        draw_pixel(x + i, y, true);
    }
}

void OledHud::draw_rect(int x, int y, int w, int h, bool fill) noexcept {
    for (int j = 0; j < h; ++j) {
        for (int i = 0; i < w; ++i) {
            if (fill || i == 0 || i == w - 1 || j == 0 || j == h - 1) {
                draw_pixel(x + i, y + j, true);
            }
        }
    }
}

void OledHud::draw_char(int x, int y, char c) noexcept {
    if (c >= 'a' && c <= 'z') {
        c = static_cast<char>(c - ('a' - 'A'));
    }
    if (c < 32 || c > 90) {
        c = '?';
    }
    const uint8_t* glyph = kFont5x7[c - 32];
    for (int col = 0; col < 5; ++col) {
        const uint8_t bits = pgm_read_byte(&glyph[col]);
        for (int row = 0; row < 7; ++row) {
            if (bits & (1u << row)) {
                draw_pixel(x + col, y + row, true);
            }
        }
    }
}

void OledHud::draw_text(int x, int y, const char* str) noexcept {
    if (!str) return;
    int cx = x;
    while (*str && cx <= 122) {
        draw_char(cx, y, *str++);
        cx += 6;
    }
}

void OledHud::render_hud(const HudDisplayData& d) noexcept {
    clear_buffer();

    if (d.cal_wizard_step > 0) {
        draw_text(4, 2, "CALIBRACAO GIMBALS");
        draw_hline(0, 11, 128);
        if (d.cal_wizard_step == 1) {
            draw_text(2, 18, "1. MOVA OS 2 STICKS");
            draw_text(2, 28, "EM TODOS OS CANTOS!");
            draw_text(2, 44, "DEPOIS APERTE [AUX]");
        } else {
            draw_text(2, 18, "2. SOLTE NO CENTRO");
            draw_text(2, 28, "E THROTTLE NO ZERO!");
            draw_text(2, 44, "APERTE [AUX] SALVAR");
        }
        return;
    }

    char line[26];

    // Linha 1 (Topo): Estado + Modo + RSSI
    if (!d.drone_linked) {
        std::snprintf(line, sizeof(line), "BUSCANDO DRONE... LR");
    } else {
        std::snprintf(line, sizeof(line), "%-8s %-4s %4ddB",
                      short_state_str(d.telem.state),
                      short_mode_str(d.telem.flight_mode),
                      static_cast<int>(d.telem.drone_rssi_dbm));
    }
    draw_text(0, 0, line);
    draw_hline(0, 9, 128);

    // Linha 2: Bateria 3S do Drone + Bateria 18650 do Controle
    const float drone_v = static_cast<float>(d.telem.vbat_mv) / 1000.0f;
    std::snprintf(line, sizeof(line), "3S:%5.2fV  TX:%4.2fV", drone_v, d.tx_vbat_v);
    draw_text(0, 13, line);

    // Linha 3: Roll, Pitch e Altitude em metros
    std::snprintf(line, sizeof(line), "R:%+4d P:%+4d A:%4.1fM",
                  static_cast<int>(d.telem.roll_deg10 / 10),
                  static_cast<int>(d.telem.pitch_deg10 / 10),
                  static_cast<float>(d.telem.alt_cm) / 100.0f);
    draw_text(0, 24, line);

    // Linha 4: Diagnóstico de Pré-Arm ou Barra de Throttle
    if (d.drone_linked && d.telem.state == static_cast<uint8_t>(SystemState::DISARMED)) {
        draw_text(0, 36, first_prearm_reason(d.telem.prearm_flags));
    } else {
        std::snprintf(line, sizeof(line), "THR:%3u%%  SW:%s",
                      static_cast<unsigned>(d.sticks.thr_permil / 10u),
                      d.arm_switch_on ? "ARM!" : "SAFE");
        draw_text(0, 36, line);
    }

    // Rodapé (y=49..62): Barra visual de Throttle + 4 barras dos motores M1..M4
    draw_rect(0, 50, 62, 12, false);
    const int thr_w = std::clamp<int>((d.sticks.thr_permil * 58) / 1000, 0, 58);
    if (thr_w > 0) {
        draw_rect(2, 52, thr_w, 8, true);
    }

    for (int m = 0; m < 4; ++m) {
        const int bx = 68 + m * 15;
        draw_rect(bx, 50, 12, 12, false);
        const int mh = std::clamp<int>((d.telem.motor_pct[m] * 10) / 100, 0, 10);
        if (mh > 0) {
            draw_rect(bx + 2, 61 - mh, 8, mh, true);
        }
    }
}

void OledHud::flush_next_page() noexcept {
    if (!present_ || !wire_) return;

    const uint8_t p = page_cursor_;
    page_cursor_ = static_cast<uint8_t>((page_cursor_ + 1u) & 0x07u);

    send_cmd(static_cast<uint8_t>(0xB0u | p)); // Set page address 0..7
    send_cmd(0x00u);                           // Lower col = 0
    send_cmd(0x10u);                           // Higher col = 0

    // O buffer interno do Wire no ESP32 suporta blocos de 64 bytes com folga -> 2 blocos de 64B
    const uint8_t* page_ptr = &buf_[p * 128u];
    for (int half = 0; half < 2; ++half) {
        wire_->beginTransmission(addr_);
        wire_->write(0x40u); // Data stream
        wire_->write(page_ptr + half * 64, 64);
        (void)wire_->endTransmission();
    }
}

void OledHud::flush_all() noexcept {
    for (int i = 0; i < 8; ++i) {
        flush_next_page();
    }
}

} // namespace fc::tx
