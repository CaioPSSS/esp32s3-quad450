#include "biquad_lpf.h"

#include <cmath>

namespace fc {

namespace {
constexpr float kSqrt2 = 1.41421356237309504880f;
constexpr float kMaxCutoffRatio = 0.49f;
} // namespace

void BiquadLPF::compute_coefficients(float sample_rate_hz, float cutoff_hz) noexcept {
    if (!std::isfinite(sample_rate_hz) || !std::isfinite(cutoff_hz) ||
        sample_rate_hz <= 0.0f || cutoff_hz <= 0.0f ||
        cutoff_hz >= kMaxCutoffRatio * sample_rate_hz) {
        b0_ = 1.0f;
        b1_ = 0.0f;
        b2_ = 0.0f;
        a1_ = 0.0f;
        a2_ = 0.0f;
        passthrough_ = true;
        return;
    }

    // Transformação bilinear com pré-warping exato na frequência de corte (Butterworth Q = 1/sqrt(2))
    const float omega = std::tan(kPi * cutoff_hz / sample_rate_hz);
    const float omega2 = omega * omega;
    const float k = 1.0f + kSqrt2 * omega + omega2;
    const float inv_k = 1.0f / k;

    b0_ = omega2 * inv_k;
    b1_ = 2.0f * b0_;
    b2_ = b0_;
    a1_ = 2.0f * (omega2 - 1.0f) * inv_k;
    a2_ = (1.0f - kSqrt2 * omega + omega2) * inv_k;
    passthrough_ = false;
}

void BiquadLPF::configure(float sample_rate_hz, float cutoff_hz) noexcept {
    compute_coefficients(sample_rate_hz, cutoff_hz);
    reset(0.0f);
}

void BiquadLPF::reconfigure_bumpless(float sample_rate_hz, float cutoff_hz, float steady_value) noexcept {
    compute_coefficients(sample_rate_hz, cutoff_hz);
    reset(steady_value);
}

float BiquadLPF::apply(float sample) noexcept {
    // Blindagem crítica de voo: um único glitch SPI gerando NaN/Inf contaminaria
    // permanentemente os registradores recursivos IIR (z1_, z2_) se não for barrado!
    if (!std::isfinite(sample)) {
        return last_out_;
    }

    if (passthrough_) {
        last_out_ = sample;
        return sample;
    }

    // Forma Direta II Transposta (menor erro de quantização em float 32-bit)
    const float y = b0_ * sample + z1_;
    z1_ = b1_ * sample - a1_ * y + z2_;
    z2_ = b2_ * sample - a2_ * y;
    last_out_ = y;
    return y;
}

void BiquadLPF::reset(float steady_value) noexcept {
    if (!std::isfinite(steady_value) || passthrough_) {
        z1_ = 0.0f;
        z2_ = 0.0f;
        last_out_ = std::isfinite(steady_value) ? steady_value : 0.0f;
        return;
    }

    // Condição de equilíbrio DC (x[n] = y[n] = steady_value):
    //   z2 = (b2 - a2) * y0
    //   z1 = (1 - b0) * y0   (pois b0 + b1 + b2 = 1 + a1 + a2)
    z2_ = (b2_ - a2_) * steady_value;
    z1_ = (1.0f - b0_) * steady_value;
    last_out_ = steady_value;
}

} // namespace fc
