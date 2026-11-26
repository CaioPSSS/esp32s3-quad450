#include "blackbox_psram.h"

#include <esp32-hal-psram.h>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "radio_packets.h"

namespace fc::hal {

namespace {
constexpr size_t kTargetPsramFrames = 106496u; // 106.496 * 64 B = 6.815.744 B (6.5 MB exatos!)
constexpr size_t kFallbackRamFrames = 512u;    // 32 KB caso testado em placa sem PSRAM

#pragma pack(push, 1)
struct BinaryDumpHeader {
    char     magic[4]       = {'B', 'B', 'O', 'X'};
    uint16_t version        = 1;
    uint16_t frame_size     = sizeof(BlackboxFrame);
    uint16_t sample_rate_hz = 500;
    uint16_t reserved       = 0;
    uint32_t frame_count    = 0;
};
static_assert(sizeof(BinaryDumpHeader) == 16, "BinaryDumpHeader deve ter 16 bytes");
#pragma pack(pop)
} // namespace

bool PsramBlackbox::begin(uint16_t decimation_div) noexcept {
    decimation_div_ = (decimation_div == 0) ? 1u : decimation_div;
    tick_div_ = 0;
    write_head_.store(0, std::memory_order_relaxed);
    count_.store(0, std::memory_order_relaxed);

    if (buf_) {
        std::free(buf_);
        buf_ = nullptr;
        capacity_ = 0;
    }

    if (psramFound()) {
        const size_t free_ps = ESP.getFreePsram();
        size_t want_frames = kTargetPsramFrames;
        if (free_ps < want_frames * sizeof(BlackboxFrame) + (256u * 1024u)) {
            want_frames = (free_ps * 3u / 4u) / sizeof(BlackboxFrame);
        }
        if (want_frames >= 1024u) {
            buf_ = static_cast<BlackboxFrame*>(ps_malloc(want_frames * sizeof(BlackboxFrame)));
            if (buf_) {
                capacity_ = want_frames;
                in_psram_ = true;
                return true;
            }
        }
    }

    buf_ = static_cast<BlackboxFrame*>(std::malloc(kFallbackRamFrames * sizeof(BlackboxFrame)));
    if (buf_) {
        capacity_ = kFallbackRamFrames;
        in_psram_ = false;
        return true;
    }
    return false;
}

void PsramBlackbox::push_from_core1(const BlackboxFrame& frame, bool is_armed_or_failsafe) noexcept {
    if (!buf_ || capacity_ == 0) return;
    if (dump_active_.load(std::memory_order_acquire)) return;

    const bool should_record = is_armed_or_failsafe || bench_record_.load(std::memory_order_relaxed);
    if (!should_record) {
        return;
    }

    if (++tick_div_ < decimation_div_) {
        return;
    }
    tick_div_ = 0;

    const size_t head = write_head_.load(std::memory_order_relaxed);
    buf_[head] = frame;

    const size_t next_head = (head + 1u >= capacity_) ? 0u : (head + 1u);
    write_head_.store(next_head, std::memory_order_release);

    const size_t cur_cnt = count_.load(std::memory_order_relaxed);
    if (cur_cnt < capacity_) {
        count_.store(cur_cnt + 1u, std::memory_order_release);
    }
}

void PsramBlackbox::clear() noexcept {
    dump_active_.store(true, std::memory_order_release);
    write_head_.store(0, std::memory_order_release);
    count_.store(0, std::memory_order_release);
    tick_div_ = 0;
    dump_active_.store(false, std::memory_order_release);
}

size_t PsramBlackbox::oldest_index(size_t current_head, size_t current_count) const noexcept {
    if (current_count < capacity_) {
        return 0u;
    }
    return current_head; // Ring buffer cheio: head aponta exatamente para o item mais antigo!
}

bool PsramBlackbox::dump_csv(Stream& out) noexcept {
    if (!buf_ || capacity_ == 0) return false;

    dump_active_.store(true, std::memory_order_release);
    delay(2); // Aguarda qualquer escrita residual de <0.5 µs do Core 1 encerrar

    const size_t total = count_.load(std::memory_order_acquire);
    const size_t head  = write_head_.load(std::memory_order_acquire);
    const size_t start = oldest_index(head, total);

    out.printf("# BLACKBOX_CSV_BEGIN count=%u rate_hz=%u\n",
               static_cast<unsigned>(total),
               static_cast<unsigned>(sample_rate_hz()));
    out.println("timestamp_us,dt_us,state,flags,"
                "gx_raw,gy_raw,gz_raw,gx_filt,gy_filt,gz_filt,"
                "rate_sp_r_mrads,rate_sp_p_mrads,"
                "pid_p_r,pid_p_p,pid_i_r,pid_i_p,pid_d_r,pid_d_p,"
                "roll_deg100,pitch_deg100,yaw_deg100,"
                "m1_permil,m2_permil,m3_permil,m4_permil,vbat_mv");

    char line[280];
    for (size_t i = 0; i < total; ++i) {
        const size_t idx = (start + i) % capacity_;
        const BlackboxFrame& f = buf_[idx];
        const int n = std::snprintf(
            line, sizeof(line),
            "%lu,%u,%u,%u,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u\n",
            static_cast<unsigned long>(f.timestamp_us),
            static_cast<unsigned>(f.dt_us),
            static_cast<unsigned>(f.state),
            static_cast<unsigned>(f.flags),
            static_cast<double>(f.gyro_raw_rads[0]),
            static_cast<double>(f.gyro_raw_rads[1]),
            static_cast<double>(f.gyro_raw_rads[2]),
            static_cast<double>(f.gyro_filt_rads[0]),
            static_cast<double>(f.gyro_filt_rads[1]),
            static_cast<double>(f.gyro_filt_rads[2]),
            static_cast<int>(f.rate_sp_mrads[0]),
            static_cast<int>(f.rate_sp_mrads[1]),
            static_cast<int>(f.pid_p_permil[0]),
            static_cast<int>(f.pid_p_permil[1]),
            static_cast<int>(f.pid_i_permil[0]),
            static_cast<int>(f.pid_i_permil[1]),
            static_cast<int>(f.pid_d_permil[0]),
            static_cast<int>(f.pid_d_permil[1]),
            static_cast<int>(f.euler_deg100[0]),
            static_cast<int>(f.euler_deg100[1]),
            static_cast<int>(f.euler_deg100[2]),
            static_cast<unsigned>(f.motor_permil[0]),
            static_cast<unsigned>(f.motor_permil[1]),
            static_cast<unsigned>(f.motor_permil[2]),
            static_cast<unsigned>(f.motor_permil[3]),
            static_cast<unsigned>(f.vbat_mv));
        if (n > 0) {
            out.write(reinterpret_cast<const uint8_t*>(line), static_cast<size_t>(std::min<int>(n, sizeof(line) - 1)));
        }
        if ((i & 0x3Fu) == 0u) {
            delay(1); // Cede CPU ao Watchdog/Wi-Fi do Core 0 a cada 64 linhas
        }
    }
    out.println("# BLACKBOX_CSV_END");

    dump_active_.store(false, std::memory_order_release);
    return true;
}

bool PsramBlackbox::dump_binary(Stream& out) noexcept {
    if (!buf_ || capacity_ == 0) return false;

    dump_active_.store(true, std::memory_order_release);
    delay(2);

    const size_t total = count_.load(std::memory_order_acquire);
    const size_t head  = write_head_.load(std::memory_order_acquire);
    const size_t start = oldest_index(head, total);

    BinaryDumpHeader hdr{};
    hdr.sample_rate_hz = sample_rate_hz();
    hdr.frame_count = static_cast<uint32_t>(total);

    out.write(reinterpret_cast<const uint8_t*>(&hdr), sizeof(hdr));

    // Calcula CRC-16-CCITT incremental sobre cabeçalho + todos os quadros
    uint16_t crc = 0xFFFFu;
    auto update_crc = [&crc](const uint8_t* data, size_t len) {
        for (size_t k = 0; k < len; ++k) {
            crc ^= static_cast<uint16_t>(data[k]) << 8;
            for (int b = 0; b < 8; ++b) {
                crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                                      : static_cast<uint16_t>(crc << 1);
            }
        }
    };
    update_crc(reinterpret_cast<const uint8_t*>(&hdr), sizeof(hdr));

    for (size_t i = 0; i < total; ++i) {
        const size_t idx = (start + i) % capacity_;
        const uint8_t* raw_frame = reinterpret_cast<const uint8_t*>(&buf_[idx]);
        out.write(raw_frame, sizeof(BlackboxFrame));
        update_crc(raw_frame, sizeof(BlackboxFrame));
        if ((i & 0x7Fu) == 0u) {
            delay(1);
        }
    }

    out.write(reinterpret_cast<const uint8_t*>(&crc), sizeof(crc));
    dump_active_.store(false, std::memory_order_release);
    return true;
}

} // namespace fc::hal
