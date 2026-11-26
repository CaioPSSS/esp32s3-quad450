#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include "flight_pipeline.h"
#include "mavlink_v2_min.h"
#include "quad_physics_6dof.h"

using namespace fc;
using namespace fc::mav;
using namespace fc::proto;
using namespace fc::sim;

namespace {

// Coordenadas Home padrão p/ mapa satélite no QGroundControl (Campo de Testes — São José dos Campos, BR)
constexpr double kHomeLatDeg = -23.204500;
constexpr double kHomeLonDeg = -45.879200;
constexpr float  kHomeAltMsl = 600.0f;

class UdpMavlinkEndpoint {
public:
    bool open(const char* target_ip, uint16_t target_port, uint16_t local_bind_port = 14556) {
        sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            return false;
        }
        const int flags = ::fcntl(sock_, F_GETFL, 0);
        ::fcntl(sock_, F_SETFL, flags | O_NONBLOCK);

        int reuse = 1;
        ::setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_addr.s_addr = htonl(INADDR_ANY);
        local.sin_port = htons(local_bind_port);
        if (::bind(sock_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
            // Porta ocupada -> deixa o kernel escolher porta efêmera automaticamente
            local.sin_port = 0;
            (void)::bind(sock_, reinterpret_cast<sockaddr*>(&local), sizeof(local));
        }

        gcs_addr_.sin_family = AF_INET;
        gcs_addr_.sin_port = htons(target_port);
        ::inet_pton(AF_INET, target_ip, &gcs_addr_.sin_addr);
        return true;
    }

    ~UdpMavlinkEndpoint() {
        if (sock_ >= 0) {
            ::close(sock_);
        }
    }

    template <typename MsgT>
    void send_msg(uint32_t msgid, const MsgT& payload) {
        uint8_t frame[280];
        const size_t n = encode_mavlink_v2(msgid, 1, 1, tx_seq_, payload, frame);
        if (n > 0 && sock_ >= 0) {
            (void)::sendto(sock_, frame, n, 0,
                           reinterpret_cast<const sockaddr*>(&gcs_addr_), sizeof(gcs_addr_));
        }
    }

    void send_status_text(uint8_t severity, const char* text) {
        MsgStatusText st{};
        st.severity = severity;
        std::snprintf(st.text, sizeof(st.text), "%s", text ? text : "");
        send_msg(MSG_ID_STATUSTEXT, st);
    }

    bool poll_incoming(ParsedMavMessage& out_msg) {
        if (rx_pos_ < rx_len_) {
            while (rx_pos_ < rx_len_) {
                if (parser_.push_byte(rx_buf_[rx_pos_++], out_msg)) {
                    return true;
                }
            }
        }
        if (sock_ < 0) {
            return false;
        }
        sockaddr_in sender{};
        socklen_t slen = sizeof(sender);
        const ssize_t r = ::recvfrom(sock_, rx_buf_, sizeof(rx_buf_), 0,
                                     reinterpret_cast<sockaddr*>(&sender), &slen);
        if (r > 0) {
            // Responde na porta de origem do QGroundControl caso ele use porta dinâmica
            gcs_addr_ = sender;
            rx_len_ = static_cast<size_t>(r);
            rx_pos_ = 0;
            while (rx_pos_ < rx_len_) {
                if (parser_.push_byte(rx_buf_[rx_pos_++], out_msg)) {
                    return true;
                }
            }
        }
        return false;
    }

private:
    int sock_ = -1;
    sockaddr_in gcs_addr_{};
    uint8_t tx_seq_ = 0;
    MavlinkV2Parser parser_{};
    uint8_t rx_buf_[512] = {};
    size_t rx_len_ = 0;
    size_t rx_pos_ = 0;
};

void send_single_param(UdpMavlinkEndpoint& mav, const FlightParams& params, uint8_t idx) {
    float val = 0.0f;
    if (!get_param_by_index(params, idx, val)) {
        return;
    }
    MsgParamValue pv{};
    pv.param_value = val;
    pv.param_count = PARAM_COUNT;
    pv.param_index = idx;
    const size_t nlen = std::strlen(kParamTable[idx].name);
    std::memcpy(pv.param_id, kParamTable[idx].name, std::min<size_t>(nlen, 16u));
    pv.param_type = MAV_PARAM_TYPE_REAL32;
    mav.send_msg(MSG_ID_PARAM_VALUE, pv);
}

const char* state_name(SystemState s) {
    switch (s) {
        case SystemState::BOOT_CALIBRATING: return "BOOT_CALIBRATING";
        case SystemState::DISARMED:         return "DISARMED";
        case SystemState::BENCH_DIAGNOSTIC: return "BENCH_DIAGNOSTIC";
        case SystemState::ARMED_IDLE:       return "ARMED_IDLE";
        case SystemState::FLYING:           return "FLYING";
        case SystemState::FAILSAFE_DESCEND: return "FAILSAFE_DESCEND";
        case SystemState::FAILSAFE_LOCKED:  return "FAILSAFE_LOCKED";
    }
    return "UNKNOWN";
}

} // namespace

int main(int argc, char** argv) {
    bool headless_demo = false;
    float max_duration_s = 0.0f; // 0 = infinito em tempo real
    uint16_t gcs_port = 14550;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--headless-demo") == 0) {
            headless_demo = true;
            if (max_duration_s <= 0.0f) max_duration_s = 16.0f;
        } else if (std::strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
            max_duration_s = std::strtof(argv[++i], nullptr);
        } else if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            gcs_port = static_cast<uint16_t>(std::atoi(argv[++i]));
        }
    }

    UdpMavlinkEndpoint mav;
    mav.open("127.0.0.1", gcs_port);

    FlightParams params{};
    FlightPipeline pipeline;
    pipeline.init(params, 1000.0f, 1000); // 1.0s de calibração estacionária no boot

    QuadPhysics6DoF physics;
    physics.reset(2.0f, -1.5f); // Inicializa em rampa leve de grama (+2° Roll, -1.5° Pitch)

    RcCommand rc{};
    rc.throttle = 0.0f;
    rc.arm_switch = false;
    rc.mode = FlightMode::STABILIZE;
    rc.link_alive = true;
    rc.timestamp_us = 0;

    BaroSample last_baro{};
    SystemState prev_state = SystemState::BOOT_CALIBRATING;
    uint64_t step_idx = 0;
    constexpr float dt_s = 0.001f;

    std::printf("=================================================================\n");
    std::printf(" ESP32-S3 Quad-450 SITL 6-DoF Simulator | MAVLink v2 UDP -> :%u\n", gcs_port);
    std::printf(" Modo: %s\n", headless_demo ? "Roteiro Fase 1 Completa (ALT_HOLD + Pouso Failsafe Baro)"
                                                : "Tempo Real Interativo (Abra o QGroundControl!)");
    std::printf("=================================================================\n");

    auto next_wake = std::chrono::steady_clock::now();

    while (max_duration_s <= 0.0f || (static_cast<float>(step_idx) * dt_s) <= max_duration_s) {
        const uint32_t now_us = static_cast<uint32_t>(step_idx * 1000ULL);
        const uint32_t now_ms = static_cast<uint32_t>(step_idx);
        const float t_s = static_cast<float>(step_idx) * dt_s;

        // Roteiro autônomo se --headless-demo estiver ativo
        if (headless_demo) {
            rc.timestamp_us = now_us;
            rc.link_alive = true;
            if (t_s < 1.3f) {
                rc.arm_switch = false;
                rc.mode = FlightMode::ALT_HOLD;
                rc.throttle = 0.0f;
            } else if (t_s < 1.7f) {
                rc.arm_switch = true;  // Arma em ARMED_IDLE (hélices em idle 6% no chão!)
                rc.mode = FlightMode::ALT_HOLD;
                rc.throttle = 0.0f;
            } else if (t_s < 3.7f) {
                rc.arm_switch = true;  // Sobe em ALT_HOLD (stick acima do centro: 82%)
                rc.mode = FlightMode::ALT_HOLD;
                rc.throttle = 0.82f;
                rc.roll = 0.0f;
            } else if (t_s < 6.0f) {
                rc.throttle = 0.50f;   // Stick no centro (50%) -> trava altura em ALT_HOLD!
                rc.roll = 0.45f;       // Faz curva inclinada mantendo a altura!
            } else if (t_s < 7.5f) {
                rc.throttle = 0.50f;   // Nivela de volta cravado na mesma altura!
                rc.roll = 0.0f;
            } else {
                // Simula perda de pacotes ESP-NOW -> FAILSAFE_DESCEND (-0.65 -> -0.35 m/s flare)
                // -> Toque automático no solo -> FAILSAFE_LOCKED (desliga hélices na grama)!
                rc.link_alive = false;
            }
        }

        // Processa comandos MAVLink vindos do QGroundControl / pymavlink
        ParsedMavMessage rx{};
        while (mav.poll_incoming(rx)) {
            if (rx.msgid == MSG_ID_PARAM_REQUEST_LIST) {
                for (uint8_t idx = 0; idx < PARAM_COUNT; ++idx) {
                    send_single_param(mav, params, idx);
                }
            } else if (rx.msgid == MSG_ID_PARAM_REQUEST_READ) {
                const auto req = rx.unpack<MsgParamRequestRead>();
                int idx = req.param_index;
                if (idx < 0) {
                    idx = find_param_by_name(req.param_id);
                }
                if (idx >= 0 && idx < PARAM_COUNT) {
                    send_single_param(mav, params, static_cast<uint8_t>(idx));
                }
            } else if (rx.msgid == MSG_ID_PARAM_SET) {
                const auto ps = rx.unpack<MsgParamSet>();
                const int idx = find_param_by_name(ps.param_id);
                if (idx >= 0 && set_param_by_index(params, static_cast<uint8_t>(idx), ps.param_value)) {
                    pipeline.apply_params_live(params);
                    send_single_param(mav, params, static_cast<uint8_t>(idx));
                }
            } else if (rx.msgid == MSG_ID_MANUAL_CONTROL) {
                const auto mc = rx.unpack<MsgManualControl>();
                rc.pitch = std::clamp(static_cast<float>(mc.x) / 1000.0f, -1.0f, 1.0f);
                rc.roll  = std::clamp(static_cast<float>(mc.y) / 1000.0f, -1.0f, 1.0f);
                rc.throttle = std::clamp(static_cast<float>(mc.z) / 1000.0f, 0.0f, 1.0f);
                rc.yaw   = std::clamp(static_cast<float>(mc.r) / 1000.0f, -1.0f, 1.0f);
                rc.timestamp_us = now_us;
                rc.link_alive = true;
            } else if (rx.msgid == MSG_ID_COMMAND_LONG) {
                const auto cmd = rx.unpack<MsgCommandLong>();
                MsgCommandAck ack{};
                ack.command = cmd.command;
                ack.target_system = rx.sysid;
                ack.target_component = rx.compid;

                if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                    rc.arm_switch = (cmd.param1 > 0.5f);
                    rc.timestamp_us = now_us;
                    rc.link_alive = true;
                    ack.result = MAV_RESULT_ACCEPTED;
                } else if (cmd.command == MAV_CMD_PREFLIGHT_CALIBRATION) {
                    ack.result = pipeline.request_gyro_recalibration()
                                     ? MAV_RESULT_ACCEPTED
                                     : MAV_RESULT_DENIED;
                } else {
                    ack.result = MAV_RESULT_UNSUPPORTED;
                }
                mav.send_msg(MSG_ID_COMMAND_ACK, ack);
            }
        }

        if (!headless_demo) {
            // Mantém link RC virtual vivo em modo interativo de QGC
            rc.timestamp_us = now_us;
            rc.link_alive = true;
        }

        // Passo 1000 Hz: Amostra IMU (1000 Hz) + BMP280 (40 Hz) -> FlightPipeline -> Física 6-DoF
        const ImuSample imu = physics.sample_imu(now_us);
        if (step_idx % 25ULL == 0ULL) {
            last_baro = physics.sample_baro(now_us);
        }
        const PipelineStepOutput step_out = pipeline.step_1khz(
            imu, rc, physics.truth().vbat_loaded_v, now_us, dt_s, last_baro);
        physics.step(step_out.motors, dt_s);

        const SimTruthState& tr = physics.truth();

        // Notifica QGroundControl e console a cada transição de estado
        if (step_out.telemetry.state != prev_state) {
            char msg[50];
            std::snprintf(msg, sizeof(msg), "Estado: %s (Alt=%.1fm)",
                          state_name(step_out.telemetry.state), tr.altitude_m());
            mav.send_status_text(MAV_SEVERITY_NOTICE, msg);
            std::printf("[t=%5.2fs] %s | VBAT=%.2fV | PreArm=0x%02X\n",
                        t_s, msg, tr.vbat_loaded_v, step_out.telemetry.prearm_flags);
            prev_state = step_out.telemetry.state;
        }

        // Streams periódicos MAVLink v2:
        // 1 Hz: HEARTBEAT
        if (step_idx % 1000ULL == 0ULL) {
            MsgHeartbeat hb{};
            hb.custom_mode = static_cast<uint32_t>(step_out.telemetry.mode);
            hb.base_mode = MAV_MODE_FLAG_CUSTOM_MODE | MAV_MODE_FLAG_STABILIZE | MAV_MODE_FLAG_MANUAL_INPUT;
            if (step_out.motors.armed) {
                hb.base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
            }
            hb.system_status = (step_out.telemetry.state == SystemState::BOOT_CALIBRATING)
                                   ? MAV_STATE_CALIBRATING
                                   : (step_out.motors.armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
            mav.send_msg(MSG_ID_HEARTBEAT, hb);
        }

        // 2 Hz: SYS_STATUS + log resumido em modo headless
        if (step_idx % 500ULL == 0ULL) {
            MsgSysStatus sys{};
            constexpr uint32_t kSensors3dGyroAccelBaro = 0x01u | 0x02u | 0x08u;
            sys.onboard_control_sensors_present = kSensors3dGyroAccelBaro;
            sys.onboard_control_sensors_enabled = kSensors3dGyroAccelBaro;
            sys.onboard_control_sensors_health  = kSensors3dGyroAccelBaro;
            sys.load = 70; // ~7.0% CPU Core 1
            sys.voltage_battery = static_cast<uint16_t>(tr.vbat_loaded_v * 1000.0f);
            sys.current_battery = static_cast<int16_t>(tr.total_current_a * 100.0f);
            sys.battery_remaining = static_cast<int8_t>(
                std::clamp((tr.vbat_loaded_v - 10.2f) / (12.6f - 10.2f) * 100.0f, 0.0f, 100.0f));
            mav.send_msg(MSG_ID_SYS_STATUS, sys);

            if (headless_demo) {
                std::printf("  t=%4.1fs | %-16s | Alt=%5.2fm Vz=%+5.2fm/s | Roll=%+5.1f° Pitch=%+5.1f° | M=[%3.0f %3.0f %3.0f %3.0f]%%\n",
                            t_s, state_name(step_out.telemetry.state),
                            tr.altitude_m(), tr.climb_rate_ms(),
                            step_out.telemetry.attitude.roll_deg(),
                            step_out.telemetry.attitude.pitch_deg(),
                            step_out.motors.throttle[0] * 100.0f,
                            step_out.motors.throttle[1] * 100.0f,
                            step_out.motors.throttle[2] * 100.0f,
                            step_out.motors.throttle[3] * 100.0f);
            }
        }

        // 25 Hz: ATTITUDE
        if (step_idx % 40ULL == 0ULL) {
            MsgAttitude att{};
            att.time_boot_ms = now_ms;
            att.roll  = step_out.telemetry.attitude.roll_rad;
            att.pitch = step_out.telemetry.attitude.pitch_rad;
            att.yaw   = step_out.telemetry.attitude.yaw_rad;
            att.rollspeed  = tr.omega_body_rads.x;
            att.pitchspeed = tr.omega_body_rads.y;
            att.yawspeed   = tr.omega_body_rads.z;
            mav.send_msg(MSG_ID_ATTITUDE, att);
        }

        // 10 Hz: GLOBAL_POSITION_INT, VFR_HUD e SERVO_OUTPUT_RAW
        if (step_idx % 100ULL == 0ULL) {
            constexpr double kMetersPerDegLat = 111320.0;
            const double meters_per_deg_lon = kMetersPerDegLat * std::cos(kHomeLatDeg * 3.141592653589793 / 180.0);

            MsgGlobalPositionInt gp{};
            gp.time_boot_ms = now_ms;
            gp.lat = static_cast<int32_t>((kHomeLatDeg + tr.pos_ned_m.x / kMetersPerDegLat) * 1e7);
            gp.lon = static_cast<int32_t>((kHomeLonDeg + tr.pos_ned_m.y / meters_per_deg_lon) * 1e7);
            gp.alt = static_cast<int32_t>((kHomeAltMsl + tr.altitude_m()) * 1000.0f);
            gp.relative_alt = static_cast<int32_t>(tr.altitude_m() * 1000.0f);
            gp.vx = static_cast<int16_t>(tr.vel_ned_ms.x * 100.0f);
            gp.vy = static_cast<int16_t>(tr.vel_ned_ms.y * 100.0f);
            gp.vz = static_cast<int16_t>(tr.vel_ned_ms.z * 100.0f);
            float hdg_deg = step_out.telemetry.attitude.yaw_deg();
            if (hdg_deg < 0.0f) hdg_deg += 360.0f;
            gp.hdg = static_cast<uint16_t>(hdg_deg * 100.0f);
            mav.send_msg(MSG_ID_GLOBAL_POSITION_INT, gp);

            MsgVfrHud hud{};
            hud.groundspeed = std::hypot(tr.vel_ned_ms.x, tr.vel_ned_ms.y);
            hud.airspeed = hud.groundspeed;
            hud.alt = tr.altitude_m();
            hud.climb = tr.climb_rate_ms();
            hud.heading = static_cast<int16_t>(hdg_deg);
            hud.throttle = static_cast<uint16_t>(rc.throttle * 100.0f);
            mav.send_msg(MSG_ID_VFR_HUD, hud);

            MsgServoOutputRaw srv{};
            srv.time_usec = now_us;
            srv.servo1_raw = QuadXMixer::to_pwm_us(step_out.motors.throttle[0], step_out.motors.armed);
            srv.servo2_raw = QuadXMixer::to_pwm_us(step_out.motors.throttle[1], step_out.motors.armed);
            srv.servo3_raw = QuadXMixer::to_pwm_us(step_out.motors.throttle[2], step_out.motors.armed);
            srv.servo4_raw = QuadXMixer::to_pwm_us(step_out.motors.throttle[3], step_out.motors.armed);
            mav.send_msg(MSG_ID_SERVO_OUTPUT_RAW, srv);
        }

        ++step_idx;
        if (!headless_demo) {
            next_wake += std::chrono::microseconds(1000);
            std::this_thread::sleep_until(next_wake);
        }
    }

    return 0;
}
