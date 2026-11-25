#pragma once

#include <atomic>
#include <cstdint>
#include <type_traits>

namespace fc::sync {

/**
 * SeqLock<T>: Primitiva Lock-Free para compartilhar structs entre Core 0 e Core 1
 * no ESP32-S3 sem usar Mutex (zero inversão de prioridade no loop de 1000 Hz!).
 *
 * Premissa: 1 único escritor (Single-Producer) e 1 ou mais leitores (Multi-Consumer).
 * - Sequência PAR   => dado consistente.
 * - Sequência ÍMPAR => escrita em andamento.
 */
template <typename T>
class SeqLock {
    static_assert(std::is_trivially_copyable_v<T>,
                  "SeqLock<T> exige um tipo trivialmente copiavel (POD/struct simples)");

public:
    SeqLock() = default;
    explicit SeqLock(const T& initial) : data_(initial) {}

    void write(const T& value) noexcept {
        const uint32_t s = seq_.load(std::memory_order_relaxed);
        seq_.store(s + 1u, std::memory_order_release);
        std::atomic_thread_fence(std::memory_order_acq_rel);

        data_ = value;

        std::atomic_thread_fence(std::memory_order_acq_rel);
        seq_.store(s + 2u, std::memory_order_release);
    }

    T read() const noexcept {
        T copy{};
        uint32_t s1 = 0;
        uint32_t s2 = 0;
        do {
            s1 = seq_.load(std::memory_order_acquire);
            if (s1 & 1u) {
                continue; // Escritor no meio da cópia; tenta novamente em nanosegundos
            }
            copy = data_;
            std::atomic_thread_fence(std::memory_order_acquire);
            s2 = seq_.load(std::memory_order_relaxed);
        } while (s1 != s2);
        return copy;
    }

    uint32_t sequence() const noexcept {
        return seq_.load(std::memory_order_acquire);
    }

private:
    std::atomic<uint32_t> seq_{0u};
    T data_{};
};

} // namespace fc::sync
