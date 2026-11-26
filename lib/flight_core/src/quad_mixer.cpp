#include "quad_mixer.h"

#include <algorithm>
#include <cmath>

namespace fc {

namespace {
constexpr float kMinTiltCosBoost = 0.70f; // Máx ~45° (~1.43x boost vertical)
constexpr float kMinVbatScale    = 0.85f; // Bateria 100% cheia (12.6V) reduz ganho levemente
constexpr float kMaxVbatScale    = 1.20f; // Bateria baixa (~9.3V) compensa até +20%
constexpr float kMaxYawAuthority = 0.30f; // Impede que Yaw roube autoridade de Roll/Pitch
} // namespace

void QuadXMixer::configure(float motor_idle, float vbat_nominal_v) noexcept {
    if (std::isfinite(motor_idle)) {
        motor_idle_ = std::clamp(motor_idle, 0.03f, 0.20f);
    }
    if (std::isfinite(vbat_nominal_v) && vbat_nominal_v > 6.0f) {
        vbat_nominal_v_ = vbat_nominal_v;
    }
}

MotorOutput QuadXMixer::mix(bool armed,
                            bool in_air,
                            float throttle_cmd,
                            const TorqueCommand& torque,
                            float tilt_cos,
                            float vbat_measured_v) noexcept {
    MotorOutput out{};
    saturated_ = false;

    if (!armed) {
        out.armed = false;
        return out; // Todos os 4 motores estritamente em 0.0f (1000 µs)
    }

    out.armed = true;

    // Em ARMED_IDLE no solo antes da decolagem: mantém rotação uniforme de marcha lenta
    if (!in_air) {
        for (int i = 0; i < 4; ++i) {
            out.throttle[i] = motor_idle_;
        }
        return out;
    }

    const float thr_clamped = std::clamp(std::isfinite(throttle_cmd) ? throttle_cmd : motor_idle_,
                                         motor_idle_, 1.0f);

    // 1. Fator de linearização da tensão da bateria 3S (ignora se VBAT desconectado na bancada)
    float vbat_scale = 1.0f;
    if (std::isfinite(vbat_measured_v) && vbat_measured_v > 6.0f) {
        vbat_scale = std::clamp(vbat_nominal_v_ / vbat_measured_v, kMinVbatScale, kMaxVbatScale);
    }

    // 2. Compensação geométrica de perda de sustentação vertical em curvas (1 / cos(tilt))
    const float safe_tilt_cos = std::clamp(std::isfinite(tilt_cos) ? tilt_cos : 1.0f,
                                           kMinTiltCosBoost, 1.0f);
    const float hover_excess = (thr_clamped - motor_idle_) * vbat_scale / safe_tilt_cos;
    const float thr_effective = std::clamp(motor_idle_ + hover_excess, motor_idle_, 0.95f);

    // 3. Momentos escalados pelo VBAT, com teto de Yaw para proteger Roll/Pitch
    const float r = (std::isfinite(torque.roll)  ? torque.roll  : 0.0f) * vbat_scale;
    const float p = (std::isfinite(torque.pitch) ? torque.pitch : 0.0f) * vbat_scale;
    const float y = std::clamp((std::isfinite(torque.yaw) ? torque.yaw : 0.0f) * vbat_scale,
                               -kMaxYawAuthority, kMaxYawAuthority);

    // 4. Matriz Canônica FRD Quad-X
    float diff[4] = {
        -r + p + y, // M1: Frente-Direita (FR, CCW)
        -r - p - y, // M2: Traseira-Direita (RR, CW)
        +r - p + y, // M3: Traseira-Esquerda (RL, CCW)
        +r + p - y  // M4: Frente-Esquerda (FL, CW)
    };

    // 5. Desaturação Airmode: preserva 100% da razão diferencial de atitude!
    float diff_min = diff[0];
    float diff_max = diff[0];
    for (int i = 1; i < 4; ++i) {
        diff_min = std::min(diff_min, diff[i]);
        diff_max = std::max(diff_max, diff[i]);
    }

    const float max_span = 1.0f - motor_idle_;
    const float req_span = diff_max - diff_min;
    if (req_span > max_span && req_span > 1e-6f) {
        // Excede a faixa física total [idle .. 1.0]: comprime proporcionalmente e avisa anti-windup
        const float compression = max_span / req_span;
        for (int i = 0; i < 4; ++i) {
            diff[i] *= compression;
        }
        diff_min *= compression;
        diff_max *= compression;
        saturated_ = true;
    }

    // Desloca o throttle comum se algum motor passar de 1.0 ou cair abaixo de motor_idle_
    float thr_shifted = thr_effective;
    if (thr_shifted + diff_max > 1.0f) {
        thr_shifted = 1.0f - diff_max;
        saturated_ = true;
    }
    if (thr_shifted + diff_min < motor_idle_) {
        thr_shifted = motor_idle_ - diff_min;
        saturated_ = true;
    }

    for (int i = 0; i < 4; ++i) {
        out.throttle[i] = std::clamp(thr_shifted + diff[i], motor_idle_, 1.0f);
    }

    return out;
}

uint16_t QuadXMixer::to_pwm_us(float motor_norm, bool armed) noexcept {
    if (!armed || !std::isfinite(motor_norm)) {
        return 1000u;
    }
    const float clamped = std::clamp(motor_norm, 0.0f, 1.0f);
    return static_cast<uint16_t>(std::lround(1000.0f + 1000.0f * clamped));
}

} // namespace fc
