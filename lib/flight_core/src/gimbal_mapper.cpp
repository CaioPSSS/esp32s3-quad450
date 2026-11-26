#include "gimbal_mapper.h"

#include <algorithm>
#include <cmath>

namespace fc {

namespace {
constexpr uint16_t kMinValidSpanCounts = 500; // Exige curso mínimo real p/ aceitar calibração
} // namespace

uint16_t GimbalMapper::map_unipolar_permil(uint16_t raw,
                                           const AxisCalibration& ax,
                                           uint16_t bottom_deadzone_permil) noexcept {
    const int32_t span = static_cast<int32_t>(ax.max_raw) - static_cast<int32_t>(ax.min_raw);
    if (span < 50) {
        return 0u;
    }

    int32_t num = ax.inverted
                      ? (static_cast<int32_t>(ax.max_raw) - static_cast<int32_t>(raw))
                      : (static_cast<int32_t>(raw) - static_cast<int32_t>(ax.min_raw));

    int32_t permil = (num * 1000) / span;
    permil = std::clamp<int32_t>(permil, 0, 1000);

    if (permil <= static_cast<int32_t>(bottom_deadzone_permil)) {
        return 0u;
    }
    const int32_t rescaled = ((permil - static_cast<int32_t>(bottom_deadzone_permil)) * 1000) /
                             (1000 - static_cast<int32_t>(bottom_deadzone_permil));
    return static_cast<uint16_t>(std::clamp<int32_t>(rescaled, 0, 1000));
}

int16_t GimbalMapper::map_bipolar_permil(uint16_t raw,
                                         const AxisCalibration& ax,
                                         int16_t center_deadzone_permil) noexcept {
    const int32_t r = static_cast<int32_t>(raw);
    const int32_t c = static_cast<int32_t>(ax.center_raw);
    const int32_t hi_span = std::max<int32_t>(static_cast<int32_t>(ax.max_raw) - c, 25);
    const int32_t lo_span = std::max<int32_t>(c - static_cast<int32_t>(ax.min_raw), 25);

    int32_t permil = 0;
    if (r >= c) {
        permil = ((r - c) * 1000) / hi_span;
    } else {
        permil = -(((c - r) * 1000) / lo_span);
    }

    if (ax.inverted) {
        permil = -permil;
    }
    permil = std::clamp<int32_t>(permil, -1000, 1000);

    const int32_t abs_p = (permil >= 0) ? permil : -permil;
    if (abs_p <= static_cast<int32_t>(center_deadzone_permil)) {
        return 0;
    }
    const int32_t mag = ((abs_p - center_deadzone_permil) * 1000) / (1000 - center_deadzone_permil);
    return static_cast<int16_t>((permil >= 0) ? mag : -mag);
}

NormalizedSticks GimbalMapper::map_raw(uint16_t raw_thr,
                                       uint16_t raw_yaw,
                                       uint16_t raw_pitch,
                                       uint16_t raw_roll) const noexcept {
    NormalizedSticks out{};
    out.thr_permil   = map_unipolar_permil(raw_thr, cal_.throttle, 25);
    out.yaw_permil   = map_bipolar_permil(raw_yaw, cal_.yaw, 20);
    out.pitch_permil = map_bipolar_permil(raw_pitch, cal_.pitch, 20);
    out.roll_permil  = map_bipolar_permil(raw_roll, cal_.roll, 20);
    return out;
}

void GimbalMapper::begin_calibration() noexcept {
    for (int i = 0; i < 4; ++i) {
        min_seen_[i] = 4095u;
        max_seen_[i] = 0u;
    }
}

void GimbalMapper::record_min_max_sample(uint16_t raw_thr, uint16_t raw_yaw,
                                         uint16_t raw_pitch, uint16_t raw_roll) noexcept {
    const uint16_t vals[4] = {raw_thr, raw_yaw, raw_pitch, raw_roll};
    for (int i = 0; i < 4; ++i) {
        if (vals[i] < min_seen_[i]) min_seen_[i] = vals[i];
        if (vals[i] > max_seen_[i]) max_seen_[i] = vals[i];
    }
}

bool GimbalMapper::finalize_calibration(uint16_t rest_thr_zero, uint16_t center_yaw,
                                        uint16_t center_pitch, uint16_t center_roll) noexcept {
    for (int i = 0; i < 4; ++i) {
        if (max_seen_[i] <= min_seen_[i] ||
            (max_seen_[i] - min_seen_[i]) < kMinValidSpanCounts) {
            return false; // Piloto esqueceu de mover um dos eixos durante a etapa 1!
        }
    }

    // Detecta automaticamente se o potenciômetro do Throttle foi soldado invertido!
    const uint16_t dist_to_min = (rest_thr_zero > min_seen_[0]) ? (rest_thr_zero - min_seen_[0]) : (min_seen_[0] - rest_thr_zero);
    const uint16_t dist_to_max = (max_seen_[0] > rest_thr_zero) ? (max_seen_[0] - rest_thr_zero) : (rest_thr_zero - max_seen_[0]);
    const bool thr_inverted = (dist_to_max < dist_to_min);

    cal_.magic = 0x4743414Cu;
    cal_.throttle = {min_seen_[0], rest_thr_zero, max_seen_[0], thr_inverted};
    cal_.yaw      = {min_seen_[1], center_yaw,    max_seen_[1], cal_.yaw.inverted};
    cal_.pitch    = {min_seen_[2], center_pitch,  max_seen_[2], cal_.pitch.inverted};
    cal_.roll     = {min_seen_[3], center_roll,   max_seen_[3], cal_.roll.inverted};
    return true;
}

} // namespace fc
