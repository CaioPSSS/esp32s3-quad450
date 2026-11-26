#include "drv_bmp280_i2c.h"

#include <cmath>

namespace fc::hal {

namespace {
constexpr uint8_t kRegCalibStart = 0x88;
constexpr uint8_t kRegChipId     = 0xD0;
constexpr uint8_t kRegReset      = 0xE0;
constexpr uint8_t kRegCtrlHum    = 0xF2; // Usado se for um BME280 vendido como BMP280!
constexpr uint8_t kRegCtrlMeas   = 0xF4;
constexpr uint8_t kRegConfig     = 0xF5;
constexpr uint8_t kRegPressMsb   = 0xF7;
constexpr uint8_t kTareTargetSamples = 30;

inline uint16_t u16_le(const uint8_t* p) noexcept {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

inline int16_t s16_le(const uint8_t* p) noexcept {
    return static_cast<int16_t>(u16_le(p));
}
} // namespace

bool Bmp280I2cDriver::write_reg(uint8_t reg, uint8_t val) noexcept {
    if (!wire_) return false;
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    wire_->write(val);
    return wire_->endTransmission() == 0;
}

bool Bmp280I2cDriver::read_bytes(uint8_t start_reg, uint8_t* buf, size_t len) noexcept {
    if (!wire_ || !buf || len == 0) return false;
    wire_->beginTransmission(addr_);
    wire_->write(start_reg);
    if (wire_->endTransmission(false) != 0) {
        return false;
    }
    const size_t got = wire_->requestFrom(addr_, static_cast<uint8_t>(len));
    if (got != len) {
        return false;
    }
    for (size_t i = 0; i < len; ++i) {
        buf[i] = static_cast<uint8_t>(wire_->read());
    }
    return true;
}

bool Bmp280I2cDriver::probe_address(uint8_t addr) noexcept {
    addr_ = addr;
    uint8_t id = 0;
    if (!read_bytes(kRegChipId, &id, 1)) {
        return false;
    }
    if (id == 0x58 || id == 0x56 || id == 0x57 || id == 0x60) {
        chip_id_ = id;
        return true;
    }
    return false;
}

bool Bmp280I2cDriver::read_calibration_trim() noexcept {
    uint8_t cal[24] = {};
    if (!read_bytes(kRegCalibStart, cal, sizeof(cal))) {
        return false;
    }
    dig_T1_ = u16_le(&cal[0]);
    dig_T2_ = s16_le(&cal[2]);
    dig_T3_ = s16_le(&cal[4]);
    dig_P1_ = u16_le(&cal[6]);
    dig_P2_ = s16_le(&cal[8]);
    dig_P3_ = s16_le(&cal[10]);
    dig_P4_ = s16_le(&cal[12]);
    dig_P5_ = s16_le(&cal[14]);
    dig_P6_ = s16_le(&cal[16]);
    dig_P7_ = s16_le(&cal[18]);
    dig_P8_ = s16_le(&cal[20]);
    dig_P9_ = s16_le(&cal[22]);
    return dig_T1_ != 0 && dig_P1_ != 0;
}

bool Bmp280I2cDriver::begin(TwoWire& wire, int sda_pin, int scl_pin) noexcept {
    wire_ = &wire;
    detected_ = false;
    tare_count_ = 0;
    tare_accum_pa_ = 0.0f;

    wire_->begin(sda_pin, scl_pin, 400000u);
    wire_->setTimeOut(5); // Garante que falha no fio I2C jamais trave o Core 0!

    if (!probe_address(0x76) && !probe_address(0x77)) {
        return false;
    }

    (void)write_reg(kRegReset, 0xB6);
    delay(10);

    if (!read_calibration_trim()) {
        return false;
    }

    if (chip_id_ == 0x60) {
        // BME280: desliga oversampling de umidade p/ acelerar conversão de pressão!
        (void)write_reg(kRegCtrlHum, 0x00);
    }

    // CONFIG (0xF5): t_sb = 0.5 ms (000), IIR filter = 4 (010 -> bits [4:2] = 0x08)
    if (!write_reg(kRegConfig, 0x08)) {
        return false;
    }

    // CTRL_MEAS (0xF4): osrs_t = x2 (010), osrs_p = x16 (101), mode = Normal (11) -> 0x57
    if (!write_reg(kRegCtrlMeas, 0x57)) {
        return false;
    }

    detected_ = true;
    return true;
}

bool Bmp280I2cDriver::poll(uint32_t now_us, BaroSample& out_sample) noexcept {
    if (!detected_) {
        out_sample.healthy = false;
        return false;
    }

    uint8_t raw[6] = {};
    if (!read_bytes(kRegPressMsb, raw, sizeof(raw))) {
        out_sample.healthy = false;
        return false;
    }

    const int32_t adc_P = (static_cast<int32_t>(raw[0]) << 12) |
                          (static_cast<int32_t>(raw[1]) << 4)  |
                          (static_cast<int32_t>(raw[2]) >> 4);
    const int32_t adc_T = (static_cast<int32_t>(raw[3]) << 12) |
                          (static_cast<int32_t>(raw[4]) << 4)  |
                          (static_cast<int32_t>(raw[5]) >> 4);

    if (adc_P == 0x80000 || adc_T == 0x80000) {
        // Conversão inicial após boot ainda não concluída
        return false;
    }

    // Compensação oficial Bosch em ponto flutuante (precisão de ~0.08 m!)
    const float var1_t = (static_cast<float>(adc_T) / 16384.0f - static_cast<float>(dig_T1_) / 1024.0f) *
                         static_cast<float>(dig_T2_);
    const float diff_t = static_cast<float>(adc_T) / 131072.0f - static_cast<float>(dig_T1_) / 8192.0f;
    const float var2_t = (diff_t * diff_t) * static_cast<float>(dig_T3_);
    const float t_fine = var1_t + var2_t;
    const float temp_c = t_fine / 5120.0f;

    float var1_p = (t_fine * 0.5f) - 64000.0f;
    float var2_p = var1_p * var1_p * static_cast<float>(dig_P6_) / 32768.0f;
    var2_p = var2_p + var1_p * static_cast<float>(dig_P5_) * 2.0f;
    var2_p = (var2_p * 0.25f) + (static_cast<float>(dig_P4_) * 65536.0f);
    var1_p = (static_cast<float>(dig_P3_) * var1_p * var1_p / 524288.0f +
              static_cast<float>(dig_P2_) * var1_p) / 524288.0f;
    var1_p = (1.0f + var1_p / 32768.0f) * static_cast<float>(dig_P1_);
    if (std::fabs(var1_p) < 1e-6f) {
        out_sample.healthy = false;
        return false;
    }

    float p_pa = 1048576.0f - static_cast<float>(adc_P);
    p_pa = (p_pa - (var2_p / 4096.0f)) * 6250.0f / var1_p;
    var1_p = static_cast<float>(dig_P9_) * p_pa * p_pa / 2147483648.0f;
    var2_p = p_pa * static_cast<float>(dig_P8_) / 32768.0f;
    p_pa = p_pa + (var1_p + var2_p + static_cast<float>(dig_P7_)) / 16.0f;

    if (!std::isfinite(p_pa) || p_pa < 30000.0f || p_pa > 120000.0f) {
        out_sample.healthy = false;
        return false;
    }

    if (tare_count_ < kTareTargetSamples) {
        tare_accum_pa_ += p_pa;
        ++tare_count_;
        p0_pa_ = tare_accum_pa_ / static_cast<float>(tare_count_);
    }

    out_sample.pressure_pa = p_pa;
    out_sample.temperature_c = temp_c;
    out_sample.altitude_m = 44330.0f * (1.0f - std::pow(p_pa / p0_pa_, 0.190294957f));
    out_sample.timestamp_us = now_us;
    out_sample.healthy = true;
    return true;
}

} // namespace fc::hal
