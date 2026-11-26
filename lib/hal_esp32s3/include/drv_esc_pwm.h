#pragma once

#include <Arduino.h>
#include <cstdint>

#include "fc_types.h"

namespace fc::hal {

struct EscPwmPins {
    int m1_fr = 4; // Frente-Direita (CCW) -> Pino 2 do SN74HCT125N
    int m2_rr = 5; // Traseira-Direita (CW) -> Pino 5 do SN74HCT125N
    int m3_rl = 6; // Traseira-Esquerda (CCW) -> Pino 9 do SN74HCT125N
    int m4_fl = 7; // Frente-Esquerda (CW) -> Pino 12 do SN74HCT125N
};

/**
 * Driver de Hardware LEDC 14-bit @ 400 Hz para 4x ESCs 30A (via buffer 5V SN74HCT125N).
 *
 * Resolução temporal:
 * - Período @ 400 Hz = 2500 µs = 16384 contagens (14 bits)
 * - 1000 µs (Desarmado / Parado) = 6554 contagens
 * - 2000 µs (100% Potência)      = 13107 contagens
 * - 6.553 degraus finos de resolução (~0.15 µs por degrau)!
 */
class EscPwmDriver {
public:
    bool begin(const EscPwmPins& pins = EscPwmPins{}) noexcept;

    /**
     * Comanda os 4 motores durante operação normal: se `out.armed == false`, crava 1000 µs!
     */
    void write_motors(const MotorOutput& out) noexcept;

    /**
     * Força imediatamente 1000 µs nos 4 motores.
     */
    void disarm_all() noexcept;

    /**
     * Uso exclusivo no Portão 6 (`BENCH_DIAGNOSTIC` sem hélices!): aciona 1 motor individual
     * limitado por segurança em no máximo 30% (`1300 µs`), mantendo os outros 3 em `1000 µs`.
     */
    void write_single_motor_bench(uint8_t motor_1_to_4, float pct_0_to_30) noexcept;

    /**
     * Uso exclusivo na calibração de endpoints dos ESCs (`esc_cal` em `BENCH_DIAGNOSTIC` sem hélices!).
     */
    void write_all_raw_us_bench(uint16_t pulse_us) noexcept;

    uint16_t last_pulse_us(uint8_t idx_0_to_3) const noexcept {
        return (idx_0_to_3 < 4) ? last_us_[idx_0_to_3] : 1000u;
    }

    static uint32_t pulse_us_to_duty_14bit(uint16_t pulse_us) noexcept;

private:
    void write_channel_us(uint8_t idx, uint16_t pulse_us) noexcept;

    int pins_[4] = {4, 5, 6, 7};
    uint16_t last_us_[4] = {1000, 1000, 1000, 1000};
    bool initialized_ = false;
};

} // namespace fc::hal
