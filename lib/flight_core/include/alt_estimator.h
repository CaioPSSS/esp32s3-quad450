#pragma once

#include <cstdint>

#include "fc_types.h"
#include "param_registry.h"

namespace fc {

/**
 * Estimador Vertical Complementar de 3 Estados (Altitude + Velocidade + Bias Accel-Z).
 *
 * Funde:
 * - Predição cinemática a 1000 Hz usando `MahonyAHRS::earth_accel_up_ms2()` (já livre de inclinação!).
 * - Correção a 25–50 Hz usando o barômetro `BMP280` (`BaroSample`).
 *
 * Recursos de robustez:
 * - Estima continuamente o bias térmico vertical do acelerômetro (`accel_bias_up_ms2_`).
 * - Enquanto desarmado no solo, faz tara suave contínua de `ground_offset_m_` para absorver
 *   variação meteorológica e aquecimento da placa, garantindo decolagem sempre em `0.00 m`!
 * - Possui *watchdog* de frescor do barômetro (`250 ms`) para fallback seguro p/ `STABILIZE`.
 */
class VerticalEstimator {
public:
    void reset() noexcept;

    /**
     * Passo de predição de 1000 Hz usando a aceleração vertical no referencial da Terra (+ = cima).
     */
    void predict_1khz(float earth_accel_up_ms2, bool motors_armed, float dt_s) noexcept;

    /**
     * Passo de correção assíncrono chamado sempre que `baro.healthy == true` e `timestamp_us` é novo.
     */
    void correct_baro(const BaroSample& baro, bool motors_armed) noexcept;

    bool is_baro_healthy(uint32_t now_us, uint32_t timeout_us = 250000u) const noexcept;

    float altitude_m() const noexcept { return alt_m_; }
    float climb_rate_ms() const noexcept { return vel_up_ms_; }
    float accel_bias_ms2() const noexcept { return accel_bias_ms2_; }
    float unbiased_accel_up_ms2() const noexcept { return last_unbiased_acc_ms2_; }

private:
    float alt_m_ = 0.0f;
    float vel_up_ms_ = 0.0f;
    float accel_bias_ms2_ = 0.0f;
    float last_unbiased_acc_ms2_ = 0.0f;

    float ground_baro_zero_m_ = 0.0f;
    uint32_t last_baro_us_ = 0;
    bool initialized_ = false;
};

/**
 * Controlador em Cascata de Altitude (`ALT_HOLD`) + Pouso Guiado de Failsafe + Detector de Toque no Solo.
 *
 * - Aprende continuamente o `hover_throttle` real em voo nivelado (absorve peso extra ou bateria!).
 * - Entrada *bumpless* (sem tranco nos motores ao virar a chave de `STABILIZE` -> `ALT_HOLD`).
 * - Zona morta central no stick `[0.42 .. 0.58]` trava `target_alt_m_`; fora dela comanda subida/descida
 *   proporcional suave (`±1.8 m/s` em `ALT_HOLD`, ou `±1.0 m/s` no modo `ALT_CINE` suave).
 * - Em `FAILSAFE_DESCEND`, controla velocidade real de descida (`-0.60 m/s` acima de `1.5 m` e
 *   *flare* suave de `-0.35 m/s` perto do chão) e detecta toque firme na grama para cortar hélices!
 */
class AltHoldController {
public:
    AltHoldController() { configure(proto::FlightParams{}); }

    void configure(const proto::FlightParams& params) noexcept;
    void reset(float base_hover_thr = 0.36f) noexcept;

    /**
     * Chamado enquanto o piloto voa manualmente em `STABILIZE` p/ rastrear o throttle real de hover
     * e manter o controlador pronto para entrada sem degrau (*bumpless*).
     */
    void observe_manual_flight(float manual_throttle,
                               bool in_air,
                               float tilt_cos,
                               const VerticalEstimator& est,
                               float dt_s) noexcept;

    /**
     * Calcula o comando de throttle `[0.12 .. 0.90]` nos modos `ALT_HOLD` e `ALT_CINE`.
     */
    float update_alt_hold(float stick_thr_0to1,
                          float max_climb_rate_ms,
                          const VerticalEstimator& est,
                          float dt_s) noexcept;

    /**
     * Calcula o comando de throttle durante `FAILSAFE_DESCEND` com barômetro saudável
     * e atualiza o detector automático de pouso (`is_touchdown_detected()`).
     */
    float update_failsafe_descent(const VerticalEstimator& est,
                                  float failsafe_elapsed_s,
                                  float dt_s) noexcept;

    bool is_touchdown_detected() const noexcept { return touchdown_detected_; }
    float target_altitude_m() const noexcept { return target_alt_m_; }
    float learned_hover_throttle() const noexcept { return hover_thr_est_; }

private:
    float compute_velocity_loop(float target_vz_ms, const VerticalEstimator& est, float dt_s) noexcept;

    float kp_alt_ = 1.15f;
    float kp_vz_  = 0.22f;
    float ki_vz_  = 0.14f;
    float kd_vz_  = 0.028f;

    float base_hover_thr_ = 0.36f;
    float hover_thr_est_  = 0.36f;
    float target_alt_m_   = 0.0f;
    float vel_i_          = 0.0f;

    bool active_hold_        = false;
    bool active_failsafe_    = false;
    float touchdown_timer_s_ = 0.0f;
    bool touchdown_detected_ = false;
};

} // namespace fc
