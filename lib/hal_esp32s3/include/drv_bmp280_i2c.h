#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <cstdint>

#include "fc_types.h"

namespace fc::hal {

/**
 * Driver I2C 400 kHz Não-Bloqueante para Barômetro BMP280 / BME280 (GPIO8 SDA, GPIO9 SCL).
 *
 * Opera exclusivamente no Core 0 (zero interferência na malha de 1 kHz do Core 1!):
 * - Auto-detecta endereço `0x76` ou `0x77` e identifica `BMP280` (0x58) ou `BME280` (0x60).
 * - Configura amostragem contínua interna (`osrs_p = x16`, `osrs_t = x2`, filtro IIR = 4).
 * - Faz tara automática da pressão ao nível do solo (`p0_pa_`) nas primeiras 30 leituras.
 */
class Bmp280I2cDriver {
public:
    bool begin(TwoWire& wire, int sda_pin = 8, int scl_pin = 9) noexcept;

    /**
     * Lê burst de 6 bytes (`0xF7..0xFC`, ~0.25 ms @ 400 kHz) e atualiza `BaroSample`.
     */
    bool poll(uint32_t now_us, BaroSample& out_sample) noexcept;

    /**
     * Zera a referência de altitude relativa para o nível atual do solo (usar quando DISARMED).
     */
    void reset_ground_reference() noexcept {
        tare_count_ = 0;
        tare_accum_pa_ = 0.0f;
    }

    bool is_detected() const noexcept { return detected_; }
    uint8_t i2c_address() const noexcept { return addr_; }
    uint8_t chip_id() const noexcept { return chip_id_; }
    float ground_pressure_pa() const noexcept { return p0_pa_; }

private:
    bool probe_address(uint8_t addr) noexcept;
    bool read_calibration_trim() noexcept;
    bool write_reg(uint8_t reg, uint8_t val) noexcept;
    bool read_bytes(uint8_t start_reg, uint8_t* buf, size_t len) noexcept;

    TwoWire* wire_ = nullptr;
    uint8_t addr_ = 0x76;
    uint8_t chip_id_ = 0x00;
    bool detected_ = false;

    // Coeficientes de fábrica Bosch (NVM 0x88..0x9F)
    uint16_t dig_T1_ = 0;
    int16_t  dig_T2_ = 0, dig_T3_ = 0;
    uint16_t dig_P1_ = 0;
    int16_t  dig_P2_ = 0, dig_P3_ = 0, dig_P4_ = 0, dig_P5_ = 0;
    int16_t  dig_P6_ = 0, dig_P7_ = 0, dig_P8_ = 0, dig_P9_ = 0;

    float p0_pa_ = 101325.0f;
    float tare_accum_pa_ = 0.0f;
    uint8_t tare_count_ = 0;
};

} // namespace fc::hal
