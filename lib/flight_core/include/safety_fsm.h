#pragma once

#include <cstdint>
#include "fc_types.h"

namespace fc {

struct SafetyConfig {
    float max_arm_tilt_deg      = 10.0f;  // Inclinação máx p/ permitir armar no solo
    float crash_flip_deg        = 65.0f;  // Inclinação de capotagem -> corte instantâneo!
    float min_prearm_vbat_v     = 10.8f;  // 3.60 V/célula mínimo para decolar
    float critical_flight_vbat_v = 9.6f;  // 3.20 V/célula crítico em failsafe
    float max_arm_throttle      = 0.05f;  // Throttle <= 5% p/ armar
    float liftoff_throttle      = 0.10f;  // Acima de 10% transita ARMED_IDLE -> FLYING
    float armed_idle_timeout_s  = 10.0f;  // Desarma sozinho se ficar 10s parado sem acelerar
    uint32_t prearm_rc_max_age_us = 150000; // 150 ms p/ considerar rádio vivo no pré-arm
    uint32_t flight_rc_timeout_us = 300000; // 300 ms sem pacotes -> Estágio 1 Failsafe
    uint32_t imu_timeout_us       = 10000;  // 10 ms sem pulso INT1 -> corte de emergência
    float failsafe_hover_thr      = 0.36f;  // Throttle inicial de descida cega em Estágio 1
    float failsafe_min_thr        = 0.20f;  // Piso da rampa de descida cega antes de cortar
    float failsafe_max_descend_s  = 6.0f;   // Duração máx de descida cega (sem baro) antes de Estágio 2
    float failsafe_max_baro_descend_s = 45.0f; // Teto seguro p/ descida controlada c/ barômetro (~25m AGL)
};

struct SafetyInputs {
    uint32_t now_us           = 0;
    bool imu_healthy          = false;
    uint32_t last_imu_us      = 0;
    bool gyro_calibrated      = false;
    RcCommand rc{};
    float tilt_cos            = 1.0f;
    float vbat_v              = 0.0f;
    bool baro_healthy         = false; // Se true, usa descida controlada por velocidade + detector de pouso
    bool touchdown_detected   = false; // Acionado pelo AltHoldController ao detectar toque firme na grama
    bool allow_no_battery_bench = false;
};

struct SafetyDecision {
    SystemState state       = SystemState::BOOT_CALIBRATING;
    uint16_t prearm_flags   = PREARM_OK;
    bool motors_armed       = false;
    bool in_air             = false;
    bool buzzer_siren       = false;
    RcCommand effective_rc{}; // Em FAILSAFE_DESCEND, substitui sticks por descida nivelada!
};

/**
 * Máquina de Estados Crítica de Segurança, Intertravamentos de Pré-Armamento e
 * Failsafe em 2 Estágios (Nivela/Desce -> Trava/Sirene).
 */
class SafetyStateMachine {
public:
    SafetyStateMachine() { configure(SafetyConfig{}); }
    explicit SafetyStateMachine(const SafetyConfig& cfg) { configure(cfg); }

    void configure(const SafetyConfig& cfg) noexcept;
    void reset() noexcept;

    void set_bench_diagnostic_mode(bool active) noexcept;

    SafetyDecision update(const SafetyInputs& in, float dt_s) noexcept;

    SystemState state() const noexcept { return state_; }
    uint16_t prearm_flags() const noexcept { return prearm_flags_; }
    float state_elapsed_s() const noexcept { return state_elapsed_s_; }

private:
    uint16_t evaluate_prearm_flags(const SafetyInputs& in, bool rc_fresh_prearm, bool imu_fresh) const noexcept;

    SafetyConfig cfg_{};
    float max_arm_tilt_cos_ = 0.98480775f; // cos(10 deg)
    float crash_flip_cos_   = 0.42261826f; // cos(65 deg)

    SystemState state_ = SystemState::BOOT_CALIBRATING;
    uint16_t prearm_flags_ = PREARM_OK;

    bool observed_switch_disarmed_ = false;
    bool prev_arm_switch_ = false;
    float state_elapsed_s_ = 0.0f;
};

} // namespace fc
