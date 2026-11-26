#pragma once

#include <Arduino.h>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "flight_pipeline.h"

namespace fc::hal {

/**
 * Gravador Caixa-Preta (Blackbox) Lock-Free de Alta Velocidade na Octal PSRAM (até 6.5 MB).
 *
 * - Capacidade padrão no ESP32-S3 N16R8: 106.496 frames x 64 B = 6.5 MB (~3.55 min @ 500 Hz!).
 * - Escrita O(1) em RAM (< 0.5 µs) a partir do Core 1 sem bloquear nem alocar memória.
 * - Leitura/exportação CSV e Binária (c/ CRC-16) no Core 0 quando o drone está desarmado.
 */
class PsramBlackbox {
public:
    bool begin(uint16_t decimation_div = 2) noexcept; // div=2 -> 500 Hz a partir de 1000 Hz

    /**
     * Chamado a 1000 Hz pelo Core 1 (`FlightControlTask`).
     * Grava automaticamente quando o drone está armado (ou quando `bench_record` está ativo).
     */
    void push_from_core1(const BlackboxFrame& frame, bool is_armed_or_failsafe) noexcept;

    void clear() noexcept;

    void set_bench_recording(bool enable) noexcept {
        bench_record_.store(enable, std::memory_order_relaxed);
    }
    bool is_bench_recording() const noexcept {
        return bench_record_.load(std::memory_order_relaxed);
    }

    size_t capacity_frames() const noexcept { return capacity_; }
    size_t stored_frames() const noexcept { return count_.load(std::memory_order_acquire); }
    uint16_t sample_rate_hz() const noexcept { return static_cast<uint16_t>(1000u / decimation_div_); }
    bool using_psram() const noexcept { return in_psram_; }

    /**
     * Exporta todos os quadros gravados em formato CSV legível (apenas quando desarmado).
     */
    bool dump_csv(Stream& out) noexcept;

    /**
     * Exporta todos os quadros em formato binário compacto ("BBOX" + CRC-16-CCITT).
     */
    bool dump_binary(Stream& out) noexcept;

private:
    size_t oldest_index(size_t current_head, size_t current_count) const noexcept;

    BlackboxFrame* buf_ = nullptr;
    size_t capacity_ = 0;
    uint16_t decimation_div_ = 2;
    uint16_t tick_div_ = 0;
    bool in_psram_ = false;

    std::atomic<size_t> write_head_{0};
    std::atomic<size_t> count_{0};
    std::atomic<bool>   dump_active_{false};
    std::atomic<bool>   bench_record_{false};
};

} // namespace fc::hal
