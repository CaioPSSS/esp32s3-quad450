#include "bench_cli.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>

namespace fc::hal {

namespace {
const char* state_name(SystemState s) noexcept {
    switch (s) {
        case SystemState::BOOT_CALIBRATING: return "BOOT_CALIBRATING";
        case SystemState::DISARMED:         return "DISARMED";
        case SystemState::BENCH_DIAGNOSTIC: return "BENCH_DIAGNOSTIC";
        case SystemState::ARMED_IDLE:       return "ARMED_IDLE";
        case SystemState::FLYING:           return "FLYING";
        case SystemState::FAILSAFE_DESCEND: return "FAILSAFE_DESCEND";
        case SystemState::FAILSAFE_LOCKED:  return "FAILSAFE_LOCKED";
        default:                            return "UNKNOWN";
    }
}

int rotation_to_deg(ImuMountRotation r) noexcept {
    switch (r) {
        case ImuMountRotation::IMU_ROT_0_DEG:   return 0;
        case ImuMountRotation::IMU_ROT_90_DEG:  return 90;
        case ImuMountRotation::IMU_ROT_180_DEG: return 180;
        case ImuMountRotation::IMU_ROT_270_DEG: return 270;
        default:                                return 0;
    }
}
} // namespace

void BenchCli::begin(Stream& serial, PsramBlackbox* blackbox, const BenchControlHooks& hooks) noexcept {
    serial_ = &serial;
    blackbox_ = blackbox;
    hooks_ = hooks;
    line_len_ = 0;
}

void BenchCli::stop_any_motor_override() noexcept {
    motor_run_active_ = false;
    esc_cal_step_ = 0;
    if (hooks_.set_motor_override) {
        hooks_.set_motor_override(BenchMotorOverride{});
    }
}

void BenchCli::update(uint32_t now_ms) noexcept {
    if (!serial_) return;

    // 1. Timeout automático de segurança do `motor_test` (Portão 6: máx 3 segundos!)
    if (motor_run_active_ && static_cast<int32_t>(now_ms - motor_run_end_ms_) >= 0) {
        stop_any_motor_override();
        serial_->println("[BENCH] Tempo de 3.0s concluido -> Motores parados em 1000 us.");
    }

    // 2. Streaming de verificação de eixos da IMU (Portão 5)
    if (imu_stream_active_) {
        if (static_cast<int32_t>(now_ms - imu_stream_end_ms_) >= 0) {
            imu_stream_active_ = false;
            serial_->println("[PORTAO 5] Fim da leitura ao vivo da IMU.");
        } else if (now_ms - last_imu_print_ms_ >= 200u && hooks_.read_status) {
            last_imu_print_ms_ = now_ms;
            BenchStatusView st{};
            hooks_.read_status(st);
            serial_->printf("  Roll:%+6.1f deg | Pitch:%+6.1f deg | Yaw:%+6.1f deg | "
                            "Gyro(deg/s):[%+5.1f %+5.1f %+5.1f] | Acc(m/s2):[%+5.2f %+5.2f %+5.2f]\n",
                            static_cast<double>(st.telem.attitude.roll_deg()),
                            static_cast<double>(st.telem.attitude.pitch_deg()),
                            static_cast<double>(st.telem.attitude.yaw_deg()),
                            static_cast<double>(st.imu.gyro_rads.x * kRadToDeg),
                            static_cast<double>(st.imu.gyro_rads.y * kRadToDeg),
                            static_cast<double>(st.imu.gyro_rads.z * kRadToDeg),
                            static_cast<double>(st.imu.accel_ms2.x),
                            static_cast<double>(st.imu.accel_ms2.y),
                            static_cast<double>(st.imu.accel_ms2.z));
        }
    }

    // 3. Leitura não-bloqueante da porta USB-C
    while (serial_->available() > 0) {
        const char c = static_cast<char>(serial_->read());

        // Se um motor estiver girando no Portão 6, QUALQUER tecla aborta imediatamente!
        if (motor_run_active_ && c != '\r' && c != '\n') {
            stop_any_motor_override();
            serial_->println("\n[PARADA IMEDIATA] Motor cortado p/ 1000 us!");
            line_len_ = 0;
            continue;
        }

        if (c == '\r' || c == '\n') {
            line_buf_[line_len_] = '\0';
            if (line_len_ > 0 || esc_cal_step_ == 2) {
                handle_line(line_buf_, now_ms);
            }
            line_len_ = 0;
        } else if ((c == '\b' || c == 0x7F) && line_len_ > 0) {
            --line_len_;
        } else if (c >= 32 && c < 127 && line_len_ + 1 < sizeof(line_buf_)) {
            line_buf_[line_len_++] = c;
        }
    }
}

void BenchCli::handle_line(char* line, uint32_t now_ms) noexcept {
    // Etapa 2 da calibração dos ESCs: piloto pressionou ENTER p/ descer de 2000 us p/ 1000 us!
    if (esc_cal_step_ == 2) {
        BenchMotorOverride low_ovr{};
        low_ovr.mode = BenchPwmMode::ESC_CAL_US;
        low_ovr.raw_us = 1000;
        if (hooks_.set_motor_override) {
            hooks_.set_motor_override(low_ovr);
        }
        esc_cal_step_ = 0;
        serial_->println("\n[ESC_CAL PASSO 2/2] Pulso reduzido p/ 1000 us (MINIMO)!");
        serial_->println("  -> Aguarde os bipes longos de confirmacao dos 4 ESCs e desconecte a LiPo.");
        stop_any_motor_override();
        return;
    }

    // Etapa 1 da calibração dos ESCs: exige digitar "CONFIRM"
    if (esc_cal_step_ == 1) {
        if (std::strcmp(line, "CONFIRM") == 0) {
            BenchMotorOverride hi_ovr{};
            hi_ovr.mode = BenchPwmMode::ESC_CAL_US;
            hi_ovr.raw_us = 2000;
            if (hooks_.set_motor_override) {
                hooks_.set_motor_override(hi_ovr);
            }
            esc_cal_step_ = 2;
            serial_->println("\n[ESC_CAL PASSO 1/2] Enviando 2000 us (MAXIMO) nos 4 canais!");
            serial_->println("  1. Conecte AGORA a bateria LiPo 3S na PDB (SEM HELICES!).");
            serial_->println("  2. Assim que ouvir os 2 bipes curtos ('bi-bi'), aperte [ENTER] imediatamente"
                             " (nao espere >3s para nao entrar em modo programacao do ESC!).");
        } else {
            esc_cal_step_ = 0;
            serial_->println("[ESC_CAL] Cancelado.");
        }
        return;
    }

    char* cmd = std::strtok(line, " \t");
    if (!cmd) return;

    if (std::strcmp(cmd, "help") == 0 || std::strcmp(cmd, "?") == 0) {
        print_help();
    } else if (std::strcmp(cmd, "status") == 0) {
        print_status();
    } else if (std::strcmp(cmd, "stop") == 0) {
        stop_any_motor_override();
        imu_stream_active_ = false;
        serial_->println("[OK] Saidas de bancada zeradas (1000 us).");
    } else if (std::strcmp(cmd, "imu_test") == 0) {
        serial_->println("\n=== PORTAO 5: VERIFICACAO DE EIXOS FRD DA IMU (10s) ===");
        serial_->println("  1. Incline p/ DIREITA       -> Roll DEVE ficar POSITIVO (+)");
        serial_->println("  2. Incline NARIZ P/ CIMA    -> Pitch DEVE ficar POSITIVO (+)");
        serial_->println("  3. Gire NARIZ SENT. HORARIO -> Yaw/GyroZ DEVE ficar POSITIVO (+)");
        imu_stream_active_ = true;
        imu_stream_end_ms_ = now_ms + 10000u;
        last_imu_print_ms_ = 0;
    } else if (std::strcmp(cmd, "imu_cal") == 0) {
        if (hooks_.trigger_gyro_cal && hooks_.trigger_gyro_cal()) {
            serial_->println("[OK] Recalibracao do giroscopio iniciada! Mantenha o drone 100% imovel por 2s...");
        } else {
            serial_->println("[ERRO] So e permitido recalibrar giroscopio quando DISARMED!");
        }
    } else if (std::strcmp(cmd, "imu_orient") == 0) {
        char* arg = std::strtok(nullptr, " \t");
        if (!arg) {
            serial_->println("Uso: imu_orient <0|90|180|270> (rotacao horaria da seta +X da IMU na placa)");
            return;
        }
        const int deg = std::atoi(arg);
        ImuMountRotation rot = ImuMountRotation::IMU_ROT_0_DEG;
        if (deg == 0)        rot = ImuMountRotation::IMU_ROT_0_DEG;
        else if (deg == 90)  rot = ImuMountRotation::IMU_ROT_90_DEG;
        else if (deg == 180) rot = ImuMountRotation::IMU_ROT_180_DEG;
        else if (deg == 270) rot = ImuMountRotation::IMU_ROT_270_DEG;
        else {
            serial_->println("[ERRO] Valores validos: 0, 90, 180 ou 270.");
            return;
        }
        if (hooks_.set_imu_rotation && hooks_.set_imu_rotation(rot)) {
            serial_->printf("[OK] Orientacao da IMU salva na NVS: %d graus! Recalibrando horizonte...\n", deg);
        } else {
            serial_->println("[ERRO] Desarme o drone antes de mudar a orientacao da IMU.");
        }
    } else if (std::strcmp(cmd, "baro_tare") == 0) {
        if (hooks_.tare_baro) {
            hooks_.tare_baro();
            serial_->println("[OK] Referencia de altitude do BMP280 zerada no solo.");
        }
    } else if (std::strcmp(cmd, "bench") == 0) {
        char* arg = std::strtok(nullptr, " \t");
        if (!arg || (std::strcmp(arg, "on") != 0 && std::strcmp(arg, "off") != 0)) {
            serial_->println("Uso: bench <on|off>");
            return;
        }
        const bool want_on = (std::strcmp(arg, "on") == 0);
        if (!want_on) {
            stop_any_motor_override();
        }
        if (hooks_.set_bench_mode && hooks_.set_bench_mode(want_on)) {
            serial_->printf("[OK] Modo BENCH_DIAGNOSTIC %s.\n", want_on ? "ATIVADO (Armamento de voo bloqueado!)" : "DESATIVADO");
        } else {
            serial_->println("[NEGADO] Desarme o drone e deixe a chave ARM do controle em OFF antes de entrar em BENCH!");
        }
    } else if (std::strcmp(cmd, "motor_test") == 0) {
        BenchStatusView st{};
        if (hooks_.read_status) hooks_.read_status(st);
        if (st.telem.state != SystemState::BENCH_DIAGNOSTIC) {
            serial_->println("[BLOQUEADO] Digite primeiro `bench on` (COM AS HELICES REMOVIDAS!).");
            return;
        }
        char* m_str = std::strtok(nullptr, " \t");
        char* p_str = std::strtok(nullptr, " \t");
        if (!m_str || !p_str) {
            serial_->println("Uso: motor_test <1..4> <0..25>  (Ex: motor_test 1 8)");
            return;
        }
        const int m = std::atoi(m_str);
        const float pct = std::clamp(static_cast<float>(std::atof(p_str)), 0.0f, 25.0f);
        if (m < 1 || m > 4) {
            serial_->println("[ERRO] Motor deve ser 1 (FR CCW), 2 (RR CW), 3 (RL CCW) ou 4 (FL CW).");
            return;
        }
        const char* desc[4] = {
            "M1 Frente-Direita (DEVE girar ANTI-HORARIO / CCW)",
            "M2 Traseira-Direita (DEVE girar HORARIO / CW)",
            "M3 Traseira-Esquerda (DEVE girar ANTI-HORARIO / CCW)",
            "M4 Frente-Esquerda (DEVE girar HORARIO / CW)"
        };
        BenchMotorOverride ovr{};
        ovr.mode = BenchPwmMode::SINGLE_MOTOR;
        ovr.motor_1to4 = static_cast<uint8_t>(m);
        ovr.pct_0to25 = pct;
        ovr.expire_ms = now_ms + 3000u;
        if (hooks_.set_motor_override) {
            hooks_.set_motor_override(ovr);
        }
        motor_run_active_ = true;
        motor_run_end_ms_ = now_ms + 3000u;
        serial_->printf("[PORTAO 6] Acionando %s a %.1f%% por 3.0s (aperte espaco p/ cortar!)...\n",
                        desc[m - 1], static_cast<double>(pct));
    } else if (std::strcmp(cmd, "esc_cal") == 0) {
        BenchStatusView st{};
        if (hooks_.read_status) hooks_.read_status(st);
        if (st.telem.state != SystemState::BENCH_DIAGNOSTIC) {
            serial_->println("[BLOQUEADO] Digite primeiro `bench on`.");
            return;
        }
        serial_->println("\n[ATENCAO MAXIMA] Calibracao de Curso dos 4 ESCs (1000..2000 us)!");
        serial_->println("  -> REMOVA TODAS AS 4 HELICES E DESCONECTE A BATERIA LIPO AGORA!");
        serial_->println("  -> Digite exatamente CONFIRM e aperte Enter para iniciar:");
        esc_cal_step_ = 1;
    } else if (std::strcmp(cmd, "params") == 0) {
        if (!hooks_.get_params) return;
        const proto::FlightParams p = hooks_.get_params();
        serial_->println("\n=== PARAMETROS DE VOO ATUAIS ===");
        for (uint8_t i = 0; i < proto::PARAM_COUNT; ++i) {
            float v = 0.0f;
            (void)proto::get_param_by_index(p, i, v);
            serial_->printf("  [%2u] %-16s = %.4f  (lim: %.2f .. %.2f)\n",
                            static_cast<unsigned>(i),
                            proto::kParamTable[i].name,
                            static_cast<double>(v),
                            static_cast<double>(proto::kParamTable[i].min_val),
                            static_cast<double>(proto::kParamTable[i].max_val));
        }
    } else if (std::strcmp(cmd, "set") == 0) {
        char* name = std::strtok(nullptr, " \t");
        char* val_s = std::strtok(nullptr, " \t");
        if (!name || !val_s || !hooks_.set_param || !hooks_.get_params) {
            serial_->println("Uso: set <NOME_PARAM> <VALOR>  (Ex: set RATE_RP_KP 0.13)");
            return;
        }
        const int idx = proto::find_param_by_name(name);
        if (idx < 0) {
            serial_->printf("[ERRO] Parametro '%s' desconhecido. Use `params`.\n", name);
            return;
        }
        const float val = static_cast<float>(std::atof(val_s));
        if (hooks_.set_param(static_cast<uint8_t>(idx), val)) {
            float applied = 0.0f;
            (void)proto::get_param_by_index(hooks_.get_params(), static_cast<uint8_t>(idx), applied);
            serial_->printf("[OK] %s = %.4f (use `save` p/ gravar na NVS!)\n",
                            proto::kParamTable[idx].name, static_cast<double>(applied));
        }
    } else if (std::strcmp(cmd, "save") == 0) {
        if (hooks_.save_params_nvs && hooks_.save_params_nvs()) {
            serial_->println("[OK] Parametros gravados permanentemente na Flash NVS!");
        }
    } else if (std::strcmp(cmd, "nvs_reset") == 0) {
        if (hooks_.reset_params_nvs && hooks_.reset_params_nvs()) {
            serial_->println("[OK] Parametros restaurados para os valores de fabrica!");
        } else {
            serial_->println("[ERRO] Desarme o drone antes de restaurar padroes.");
        }
    } else if (std::strcmp(cmd, "bb_info") == 0 && blackbox_) {
        const float secs = static_cast<float>(blackbox_->stored_frames()) /
                           static_cast<float>(blackbox_->sample_rate_hz());
        serial_->printf("[BLACKBOX] Armazenamento: %s | Gravados: %u / %u quadros (%.1f s @ %u Hz)\n",
                        blackbox_->using_psram() ? "Octal PSRAM (6.5 MB)" : "SRAM Interna",
                        static_cast<unsigned>(blackbox_->stored_frames()),
                        static_cast<unsigned>(blackbox_->capacity_frames()),
                        static_cast<double>(secs),
                        static_cast<unsigned>(blackbox_->sample_rate_hz()));
    } else if (std::strcmp(cmd, "bb_clear") == 0 && blackbox_) {
        blackbox_->clear();
        serial_->println("[BLACKBOX] Memoria limpa (0 quadros).");
    } else if (std::strcmp(cmd, "bb_bench_rec") == 0 && blackbox_) {
        char* arg = std::strtok(nullptr, " \t");
        const bool en = (arg && std::strcmp(arg, "on") == 0);
        blackbox_->set_bench_recording(en);
        serial_->printf("[BLACKBOX] Gravacao forcada em bancada: %s\n", en ? "ON" : "OFF");
    } else if (std::strcmp(cmd, "bb_dump_csv") == 0 && blackbox_) {
        BenchStatusView st{};
        if (hooks_.read_status) hooks_.read_status(st);
        if (st.telem.motors.armed) {
            serial_->println("[ERRO] Desarme o drone antes de exportar a Blackbox!");
            return;
        }
        (void)blackbox_->dump_csv(*serial_);
    } else if (std::strcmp(cmd, "bb_dump_bin") == 0 && blackbox_) {
        BenchStatusView st{};
        if (hooks_.read_status) hooks_.read_status(st);
        if (st.telem.motors.armed) {
            serial_->println("[ERRO] Desarme o drone antes de exportar a Blackbox!");
            return;
        }
        (void)blackbox_->dump_binary(*serial_);
    } else {
        serial_->printf("[CLI] Comando '%s' desconhecido. Digite `help`.\n", cmd);
    }
}

void BenchCli::print_help() noexcept {
    serial_->println("\n================ QUAD-450 ESP32-S3 BENCH CLI ================");
    serial_->println("  status                    - Diagnostico completo (IMU, Baro, VBAT, RC, PreArm)");
    serial_->println("  imu_test                  - [Portao 5] Mostra Roll/Pitch/Yaw ao vivo por 10s");
    serial_->println("  imu_orient <0|90|180|270> - Ajusta rotacao da IMU na placa 7x9cm (salva NVS)");
    serial_->println("  imu_cal                   - Recalibra zero do giroscopio (manter imovel 2s)");
    serial_->println("  baro_tare                 - Zera altitude relativa do BMP280 no solo");
    serial_->println("  bench <on|off>            - Entra/sai do modo BENCH_DIAGNOSTIC");
    serial_->println("  motor_test <1..4> <0..25> - [Portao 6] Gira 1 motor s/ helice por 3s");
    serial_->println("  esc_cal                   - Assistente seguro de calibracao 2000->1000us dos ESCs");
    serial_->println("  params | set <K> <V>      - Lista ou altera ganhos PID/Filtros ao vivo");
    serial_->println("  save   | nvs_reset        - Salva na Flash NVS ou restaura padroes");
    serial_->println("  bb_info | bb_clear        - Status ou limpeza da Caixa-Preta PSRAM (6.5 MB)");
    serial_->println("  bb_bench_rec <on|off>     - Liga gravacao da Blackbox mesmo desarmado");
    serial_->println("  bb_dump_csv | bb_dump_bin - Exporta Blackbox p/ tools/analyze_blackbox.py");
    serial_->println("=============================================================");
}

void BenchCli::print_prearm_decoded(uint16_t flags) noexcept {
    if (flags == PREARM_OK) {
        serial_->println("  Pre-Arm Check : [PRONTO PARA ARMAR (0x0000)]");
        return;
    }
    serial_->printf("  Pre-Arm Check : [BLOQUEADO (0x%04X)] ->", static_cast<unsigned>(flags));
    if (flags & ERR_IMU_UNHEALTHY)      serial_->print(" IMU_FALHA");
    if (flags & ERR_GYRO_UNCALIBRATED)  serial_->print(" GYRO_NAO_CALIBRADO");
    if (flags & ERR_RC_LINK_LOST)       serial_->print(" SEM_RADIO_RC");
    if (flags & ERR_ARM_SWITCH_BOOT_ON) serial_->print(" CICLAR_CHAVE_ARM_OFF");
    if (flags & ERR_THROTTLE_NOT_ZERO)  serial_->print(" BAIXAR_THROTTLE");
    if (flags & ERR_TILT_TOO_HIGH)      serial_->print(" NIVELAR_DRONE(<10deg)");
    if (flags & ERR_BATTERY_LOW)        serial_->print(" BATERIA_BAIXA(<10.8V)");
    if (flags & ERR_BENCH_MODE_ACTIVE)  serial_->print(" MODO_BENCH_ATIVO");
    serial_->println();
}

void BenchCli::print_status() noexcept {
    if (!hooks_.read_status) return;
    BenchStatusView st{};
    hooks_.read_status(st);

    serial_->println("\n---------------- STATUS CONTROLADORA DRONE_FC ----------------");
    serial_->printf("  Estado FSM    : %s | Armado: %s\n",
                    state_name(st.telem.state),
                    st.telem.motors.armed ? "SIM" : "NAO");
    print_prearm_decoded(st.telem.prearm_flags);
    serial_->printf("  IMU SPI       : %s (WHO_AM_I=0x%02X, Rot=%d deg) | INT1=%u Hz | Healthy=%d\n",
                    st.imu_name,
                    static_cast<unsigned>(st.imu_whoami),
                    rotation_to_deg(st.imu_rot),
                    static_cast<unsigned>(st.int1_hz),
                    st.imu.healthy ? 1 : 0);
    serial_->printf("  Loop Core 1   : dt=%u us | Carga CPU=%u%%\n",
                    static_cast<unsigned>(st.telem.loop_dt_us),
                    static_cast<unsigned>(st.telem.cpu_load_pct));
    serial_->printf("  Atitude FRD   : Roll=%+6.2f deg | Pitch=%+6.2f deg | Yaw=%+6.2f deg\n",
                    static_cast<double>(st.telem.attitude.roll_deg()),
                    static_cast<double>(st.telem.attitude.pitch_deg()),
                    static_cast<double>(st.telem.attitude.yaw_deg()));
    serial_->printf("  Baro BMP280   : %s | Alt=%+6.2f m | Press=%.0f Pa | Temp=%.1f C\n",
                    st.baro_ok ? "OK" : "AUSENTE",
                    static_cast<double>(st.baro.altitude_m),
                    static_cast<double>(st.baro.pressure_pa),
                    static_cast<double>(st.baro.temperature_c));
    serial_->printf("  Bateria 3S    : %.2f V | Motores: [M1:%.0f%% M2:%.0f%% M3:%.0f%% M4:%.0f%%]\n",
                    static_cast<double>(st.telem.vbat_v),
                    static_cast<double>(st.telem.motors.throttle[0] * 100.0f),
                    static_cast<double>(st.telem.motors.throttle[1] * 100.0f),
                    static_cast<double>(st.telem.motors.throttle[2] * 100.0f),
                    static_cast<double>(st.telem.motors.throttle[3] * 100.0f));
    serial_->printf("  Radio ESP-NOW : %s (RSSI=%d dBm) | Thr=%.0f%% R=%+.2f P=%+.2f Y=%+.2f | ARM_SW=%s\n",
                    st.rc.link_alive ? "CONECTADO" : "SEM_LINK",
                    static_cast<int>(st.rssi_dbm),
                    static_cast<double>(st.rc.throttle * 100.0f),
                    static_cast<double>(st.rc.roll),
                    static_cast<double>(st.rc.pitch),
                    static_cast<double>(st.rc.yaw),
                    st.rc.arm_switch ? "ON" : "OFF");
    if (blackbox_) {
        serial_->printf("  Blackbox PSRAM: %u / %u quadros gravados\n",
                        static_cast<unsigned>(blackbox_->stored_frames()),
                        static_cast<unsigned>(blackbox_->capacity_frames()));
    }
    serial_->println("--------------------------------------------------------------");
}

} // namespace fc::hal
