#pragma once

#include "fc_types.h"

namespace fc {

/**
 * Filtro Passa-Baixa Biquad Butterworth de 2ª Ordem (Forma Direta II Transposta).
 *
 * Usado para eliminar vibrações mecânicas das hélices/motores (tipicamente > 80 Hz)
 * antes do estimador de atitude Mahony e no termo Derivativo do PID.
 *
 * Equações dos coeficientes (Butterworth 2ª ordem, Q = 1/sqrt(2)):
 *   omega = tan(pi * cutoff_hz / sample_rate_hz)
 *   k     = 1 + sqrt(2) * omega + omega^2
 *   b0    = omega^2 / k
 *   b1    = 2 * b0
 *   b2    = b0
 *   a1    = 2 * (omega^2 - 1) / k
 *   a2    = (1 - sqrt(2) * omega + omega^2) / k
 *
 * Equação de diferenças (Direct Form II Transposed):
 *   y  = b0 * x + z1
 *   z1 = b1 * x - a1 * y + z2
 *   z2 = b2 * x - a2 * y
 */
class BiquadLPF {
public:
    BiquadLPF() = default;
    BiquadLPF(float sample_rate_hz, float cutoff_hz) {
        configure(sample_rate_hz, cutoff_hz);
    }

    /**
     * Calcula os coeficientes do filtro Butterworth e zera o estado interno.
     * Regra de segurança: se sample_rate_hz <= 0, cutoff_hz <= 0, ou
     * cutoff_hz >= 0.49f * sample_rate_hz (perto/acima de Nyquist),
     * configura o filtro em modo passthrough (b0=1, b1=b2=a1=a2=0).
     */
    void configure(float sample_rate_hz, float cutoff_hz) noexcept;

    /**
     * Atualiza a frequência de corte em pleno voo (ex.: via MAVLink PARAM_SET)
     * pré-carregando os registradores z1/z2 para evitar degrau/transiente na saída.
     */
    void reconfigure_bumpless(float sample_rate_hz, float cutoff_hz, float steady_value) noexcept;

    /**
     * Processa 1 amostra em tempo constante O(1) e retorna o valor filtrado.
     * Protegido contra NaN/Inf vindos de glitches de barramento SPI/I2C.
     */
    float apply(float sample) noexcept;

    /**
     * Reinicia os registradores internos para estado estacionário em `steady_value`
     * (padrão 0.0f).
     */
    void reset(float steady_value = 0.0f) noexcept;

    bool is_passthrough() const noexcept { return passthrough_; }
    float last_output() const noexcept { return last_out_; }

private:
    void compute_coefficients(float sample_rate_hz, float cutoff_hz) noexcept;

    float b0_ = 1.0f;
    float b1_ = 0.0f;
    float b2_ = 0.0f;
    float a1_ = 0.0f;
    float a2_ = 0.0f;
    float z1_ = 0.0f;
    float z2_ = 0.0f;
    float last_out_ = 0.0f;
    bool passthrough_ = true;
};

/**
 * Wrapper conveniente de 3 eixos independentes (X, Y, Z) para Giroscópio e Acelerômetro.
 */
class BiquadVec3 {
public:
    BiquadVec3() = default;
    BiquadVec3(float sample_rate_hz, float cutoff_hz) {
        configure(sample_rate_hz, cutoff_hz);
    }

    void configure(float sample_rate_hz, float cutoff_hz) noexcept {
        fx_.configure(sample_rate_hz, cutoff_hz);
        fy_.configure(sample_rate_hz, cutoff_hz);
        fz_.configure(sample_rate_hz, cutoff_hz);
    }

    void reconfigure_bumpless(float sample_rate_hz, float cutoff_hz) noexcept {
        fx_.reconfigure_bumpless(sample_rate_hz, cutoff_hz, fx_.last_output());
        fy_.reconfigure_bumpless(sample_rate_hz, cutoff_hz, fy_.last_output());
        fz_.reconfigure_bumpless(sample_rate_hz, cutoff_hz, fz_.last_output());
    }

    Vec3f apply(const Vec3f& v) noexcept {
        return {fx_.apply(v.x), fy_.apply(v.y), fz_.apply(v.z)};
    }

    void reset(const Vec3f& steady = {}) noexcept {
        fx_.reset(steady.x);
        fy_.reset(steady.y);
        fz_.reset(steady.z);
    }

private:
    BiquadLPF fx_{};
    BiquadLPF fy_{};
    BiquadLPF fz_{};
};

} // namespace fc
