#include "drv_esc_pwm.h"

#include <algorithm>
#include <cmath>

namespace fc::hal {

namespace {
constexpr uint32_t kPwmFreqHz      = 400u;
constexpr uint8_t  kPwmResBits     = 14u;
constexpr uint32_t kPwmMaxCounts   = (1u << kPwmResBits); // 16384
constexpr uint32_t kPwmPeriodUs    = 2500u;               // 1e6 / 400 Hz
constexpr uint16_t kMinPulseUs     = 1000u;
constexpr uint16_t kMaxPulseUs     = 2000u;
constexpr float    kMaxBenchPct    = 30.0f;
} // namespace

uint32_t EscPwmDriver::pulse_us_to_duty_14bit(uint16_t pulse_us) noexcept {
    const uint32_t clamped_us = std::clamp<uint32_t>(pulse_us, kMinPulseUs, kMaxPulseUs);
    // Arredondamento inteiro exato: (clamped_us * 16384 + 1250) / 2500
    return (clamped_us * kPwmMaxCounts + (kPwmPeriodUs / 2u)) / kPwmPeriodUs;
}

uint32_t EscPwmDriver::pulse_us_f_to_duty_14bit(float pulse_us) noexcept {
    if (!std::isfinite(pulse_us)) {
        pulse_us = static_cast<float>(kMinPulseUs);
    }
    const float clamped_us = std::clamp(pulse_us, static_cast<float>(kMinPulseUs), static_cast<float>(kMaxPulseUs));
    const float raw_duty = (clamped_us * static_cast<float>(kPwmMaxCounts)) / static_cast<float>(kPwmPeriodUs);
    return static_cast<uint32_t>(std::lround(raw_duty));
}

void EscPwmDriver::write_channel_us(uint8_t idx, uint16_t pulse_us) noexcept {
    write_channel_us_f(idx, static_cast<float>(pulse_us));
}

void EscPwmDriver::write_channel_us_f(uint8_t idx, float pulse_us) noexcept {
    if (!initialized_ || idx >= 4) return;
    if (!std::isfinite(pulse_us)) {
        pulse_us = static_cast<float>(kMinPulseUs);
    }
    const float clamped_us = std::clamp(pulse_us, static_cast<float>(kMinPulseUs), static_cast<float>(kMaxPulseUs));
    last_us_[idx] = static_cast<uint16_t>(std::lround(clamped_us));
    const uint32_t duty = pulse_us_f_to_duty_14bit(clamped_us);

#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    ledcWrite(pins_[idx], duty);
#else
    ledcWrite(idx, duty);
#endif
}

bool EscPwmDriver::begin(const EscPwmPins& pins) noexcept {
    pins_[0] = pins.m1_fr;
    pins_[1] = pins.m2_rr;
    pins_[2] = pins.m3_rl;
    pins_[3] = pins.m4_fl;

    const uint32_t disarmed_duty = pulse_us_to_duty_14bit(kMinPulseUs);

    for (uint8_t i = 0; i < 4; ++i) {
        pinMode(pins_[i], OUTPUT);
        digitalWrite(pins_[i], LOW);
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
        if (!ledcAttach(pins_[i], kPwmFreqHz, kPwmResBits)) {
            return false;
        }
        ledcWrite(pins_[i], disarmed_duty);
#else
        ledcSetup(i, kPwmFreqHz, kPwmResBits);
        ledcAttachPin(pins_[i], i);
        ledcWrite(i, disarmed_duty);
#endif
        last_us_[i] = kMinPulseUs;
    }

    initialized_ = true;
    return true;
}

void EscPwmDriver::disarm_all() noexcept {
    for (uint8_t i = 0; i < 4; ++i) {
        write_channel_us(i, kMinPulseUs);
    }
}

void EscPwmDriver::write_motors(const MotorOutput& out) noexcept {
    if (!out.armed) {
        disarm_all();
        return;
    }

    for (uint8_t i = 0; i < 4; ++i) {
        float thr = out.throttle[i];
        if (!std::isfinite(thr)) {
            thr = 0.0f;
        }
        thr = std::clamp(thr, 0.0f, 1.0f);
        const float pulse_us = 1000.0f + thr * 1000.0f;
        write_channel_us_f(i, pulse_us);
    }
}

void EscPwmDriver::write_single_motor_bench(uint8_t motor_1_to_4, float pct_0_to_30) noexcept {
    if (!std::isfinite(pct_0_to_30) || motor_1_to_4 < 1 || motor_1_to_4 > 4) {
        disarm_all();
        return;
    }
    const float safe_pct = std::clamp(pct_0_to_30, 0.0f, kMaxBenchPct);
    const uint16_t active_us = static_cast<uint16_t>(std::lround(1000.0f + safe_pct * 10.0f));
    for (uint8_t i = 0; i < 4; ++i) {
        write_channel_us(i, (i == (motor_1_to_4 - 1u)) ? active_us : kMinPulseUs);
    }
}

void EscPwmDriver::write_all_raw_us_bench(uint16_t pulse_us) noexcept {
    const uint16_t safe_us = std::clamp<uint16_t>(pulse_us, kMinPulseUs, kMaxPulseUs);
    for (uint8_t i = 0; i < 4; ++i) {
        write_channel_us(i, safe_us);
    }
}

} // namespace fc::hal
