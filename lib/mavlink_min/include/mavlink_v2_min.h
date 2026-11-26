#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace fc::mav {

// ============================================================================
// Implementação Header-Only Oficialmente Compatível com MAVLink v2 (common.xml)
// Validada byte-a-byte contra pymavlink v2.0 (X.25 CRC + CRC_EXTRA + Zero-Trim)
// ============================================================================

constexpr uint8_t kMavlinkV2Stx = 0xFD;
constexpr size_t  kMaxPayloadLen = 255;

// Constantes padrão MAVLink (common.xml)
constexpr uint8_t MAV_TYPE_QUADROTOR         = 2;
constexpr uint8_t MAV_AUTOPILOT_GENERIC      = 0;
constexpr uint8_t MAV_MODE_FLAG_CUSTOM_MODE  = 1;
constexpr uint8_t MAV_MODE_FLAG_MANUAL_INPUT = 64;
constexpr uint8_t MAV_MODE_FLAG_STABILIZE    = 16;
constexpr uint8_t MAV_MODE_FLAG_SAFETY_ARMED = 128;

constexpr uint8_t MAV_STATE_UNINIT      = 0;
constexpr uint8_t MAV_STATE_BOOT        = 1;
constexpr uint8_t MAV_STATE_CALIBRATING = 2;
constexpr uint8_t MAV_STATE_STANDBY     = 3;
constexpr uint8_t MAV_STATE_ACTIVE      = 4;
constexpr uint8_t MAV_STATE_CRITICAL    = 5;
constexpr uint8_t MAV_STATE_EMERGENCY   = 6;

constexpr uint8_t MAV_PARAM_TYPE_REAL32 = 9;

constexpr uint8_t MAV_SEVERITY_WARNING  = 4;
constexpr uint8_t MAV_SEVERITY_NOTICE   = 5;
constexpr uint8_t MAV_SEVERITY_INFO     = 6;

constexpr uint16_t MAV_CMD_NAV_RETURN_TO_LAUNCH  = 20;
constexpr uint16_t MAV_CMD_PREFLIGHT_CALIBRATION = 241;
constexpr uint16_t MAV_CMD_COMPONENT_ARM_DISARM  = 400;

constexpr uint8_t MAV_RESULT_ACCEPTED = 0;
constexpr uint8_t MAV_RESULT_DENIED   = 2;
constexpr uint8_t MAV_RESULT_UNSUPPORTED = 3;

// IDs e CRC_EXTRA oficiais (common.xml)
enum MsgId : uint32_t {
    MSG_ID_HEARTBEAT          = 0,
    MSG_ID_SYS_STATUS         = 1,
    MSG_ID_PARAM_REQUEST_READ = 20,
    MSG_ID_PARAM_REQUEST_LIST = 21,
    MSG_ID_PARAM_VALUE        = 22,
    MSG_ID_PARAM_SET          = 23,
    MSG_ID_ATTITUDE           = 30,
    MSG_ID_GLOBAL_POSITION_INT= 33,
    MSG_ID_SERVO_OUTPUT_RAW   = 36,
    MSG_ID_MANUAL_CONTROL     = 69,
    MSG_ID_VFR_HUD            = 74,
    MSG_ID_COMMAND_LONG       = 76,
    MSG_ID_COMMAND_ACK        = 77,
    MSG_ID_STATUSTEXT         = 253
};

inline int get_crc_extra(uint32_t msgid) noexcept {
    switch (msgid) {
        case MSG_ID_HEARTBEAT:           return 50;
        case MSG_ID_SYS_STATUS:          return 124;
        case MSG_ID_PARAM_REQUEST_READ:  return 214;
        case MSG_ID_PARAM_REQUEST_LIST:  return 159;
        case MSG_ID_PARAM_VALUE:         return 220;
        case MSG_ID_PARAM_SET:           return 168;
        case MSG_ID_ATTITUDE:            return 39;
        case MSG_ID_GLOBAL_POSITION_INT: return 104;
        case MSG_ID_SERVO_OUTPUT_RAW:    return 222;
        case MSG_ID_MANUAL_CONTROL:      return 243;
        case MSG_ID_VFR_HUD:             return 20;
        case MSG_ID_COMMAND_LONG:        return 152;
        case MSG_ID_COMMAND_ACK:         return 143;
        case MSG_ID_STATUSTEXT:          return 83;
        default:                         return -1;
    }
}

#pragma pack(push, 1)

struct MsgHeartbeat { // id=0, crc=50, size=9
    uint32_t custom_mode = 0;
    uint8_t  type = MAV_TYPE_QUADROTOR;
    uint8_t  autopilot = MAV_AUTOPILOT_GENERIC;
    uint8_t  base_mode = 0;
    uint8_t  system_status = MAV_STATE_STANDBY;
    uint8_t  mavlink_version = 3;
};
static_assert(sizeof(MsgHeartbeat) == 9, "MsgHeartbeat size mismatch");

struct MsgSysStatus { // id=1, crc=124, size=43
    uint32_t onboard_control_sensors_present = 0;
    uint32_t onboard_control_sensors_enabled = 0;
    uint32_t onboard_control_sensors_health  = 0;
    uint16_t load = 0;            // 0..1000 (0.1%)
    uint16_t voltage_battery = 0; // mV
    int16_t  current_battery = -1;// 10 mA (-1 = não medido)
    uint16_t drop_rate_comm = 0;
    uint16_t errors_comm = 0;
    uint16_t errors_count1 = 0;
    uint16_t errors_count2 = 0;
    uint16_t errors_count3 = 0;
    uint16_t errors_count4 = 0;
    int8_t   battery_remaining = -1; // %
    uint32_t onboard_control_sensors_present_extended = 0;
    uint32_t onboard_control_sensors_enabled_extended = 0;
    uint32_t onboard_control_sensors_health_extended  = 0;
};
static_assert(sizeof(MsgSysStatus) == 43, "MsgSysStatus size mismatch");

struct MsgParamRequestRead { // id=20, crc=214, size=20
    int16_t param_index = -1;
    uint8_t target_system = 1;
    uint8_t target_component = 1;
    char    param_id[16] = {};
};
static_assert(sizeof(MsgParamRequestRead) == 20, "MsgParamRequestRead size mismatch");

struct MsgParamRequestList { // id=21, crc=159, size=2
    uint8_t target_system = 1;
    uint8_t target_component = 1;
};
static_assert(sizeof(MsgParamRequestList) == 2, "MsgParamRequestList size mismatch");

struct MsgParamValue { // id=22, crc=220, size=25
    float    param_value = 0.0f;
    uint16_t param_count = 0;
    uint16_t param_index = 0;
    char     param_id[16] = {};
    uint8_t  param_type = MAV_PARAM_TYPE_REAL32;
};
static_assert(sizeof(MsgParamValue) == 25, "MsgParamValue size mismatch");

struct MsgParamSet { // id=23, crc=168, size=23
    float   param_value = 0.0f;
    uint8_t target_system = 1;
    uint8_t target_component = 1;
    char    param_id[16] = {};
    uint8_t param_type = MAV_PARAM_TYPE_REAL32;
};
static_assert(sizeof(MsgParamSet) == 23, "MsgParamSet size mismatch");

struct MsgAttitude { // id=30, crc=39, size=28
    uint32_t time_boot_ms = 0;
    float    roll = 0.0f;
    float    pitch = 0.0f;
    float    yaw = 0.0f;
    float    rollspeed = 0.0f;
    float    pitchspeed = 0.0f;
    float    yawspeed = 0.0f;
};
static_assert(sizeof(MsgAttitude) == 28, "MsgAttitude size mismatch");

struct MsgGlobalPositionInt { // id=33, crc=104, size=28
    uint32_t time_boot_ms = 0;
    int32_t  lat = 0;          // degE7
    int32_t  lon = 0;          // degE7
    int32_t  alt = 0;          // mm MSL
    int32_t  relative_alt = 0; // mm acima de Home
    int16_t  vx = 0;           // cm/s Norte
    int16_t  vy = 0;           // cm/s Leste
    int16_t  vz = 0;           // cm/s Baixo (Down)
    uint16_t hdg = 65535;      // cdeg
};
static_assert(sizeof(MsgGlobalPositionInt) == 28, "MsgGlobalPositionInt size mismatch");

struct MsgServoOutputRaw { // id=36, crc=222, size=37
    uint32_t time_usec = 0;
    uint16_t servo1_raw = 1000;
    uint16_t servo2_raw = 1000;
    uint16_t servo3_raw = 1000;
    uint16_t servo4_raw = 1000;
    uint16_t servo5_raw = 0;
    uint16_t servo6_raw = 0;
    uint16_t servo7_raw = 0;
    uint16_t servo8_raw = 0;
    uint8_t  port = 0;
    uint16_t servo9_raw = 0;
    uint16_t servo10_raw = 0;
    uint16_t servo11_raw = 0;
    uint16_t servo12_raw = 0;
    uint16_t servo13_raw = 0;
    uint16_t servo14_raw = 0;
    uint16_t servo15_raw = 0;
    uint16_t servo16_raw = 0;
};
static_assert(sizeof(MsgServoOutputRaw) == 37, "MsgServoOutputRaw size mismatch");

struct MsgManualControl { // id=69, crc=243, size=30
    int16_t  x = 0;       // Pitch [-1000..1000]
    int16_t  y = 0;       // Roll  [-1000..1000]
    int16_t  z = 0;       // Throttle [0..1000]
    int16_t  r = 0;       // Yaw   [-1000..1000]
    uint16_t buttons = 0;
    uint8_t  target = 1;
    uint16_t buttons2 = 0;
    uint8_t  enabled_extensions = 0;
    int16_t  s = 0;
    int16_t  t = 0;
    int16_t  aux1 = 0;
    int16_t  aux2 = 0;
    int16_t  aux3 = 0;
    int16_t  aux4 = 0;
    int16_t  aux5 = 0;
    int16_t  aux6 = 0;
};
static_assert(sizeof(MsgManualControl) == 30, "MsgManualControl size mismatch");

struct MsgVfrHud { // id=74, crc=20, size=20
    float    airspeed = 0.0f;
    float    groundspeed = 0.0f;
    float    alt = 0.0f;
    float    climb = 0.0f;
    int16_t  heading = 0;
    uint16_t throttle = 0; // 0..100%
};
static_assert(sizeof(MsgVfrHud) == 20, "MsgVfrHud size mismatch");

struct MsgCommandLong { // id=76, crc=152, size=33
    float    param1 = 0.0f;
    float    param2 = 0.0f;
    float    param3 = 0.0f;
    float    param4 = 0.0f;
    float    param5 = 0.0f;
    float    param6 = 0.0f;
    float    param7 = 0.0f;
    uint16_t command = 0;
    uint8_t  target_system = 1;
    uint8_t  target_component = 1;
    uint8_t  confirmation = 0;
};
static_assert(sizeof(MsgCommandLong) == 33, "MsgCommandLong size mismatch");

struct MsgCommandAck { // id=77, crc=143, size=10
    uint16_t command = 0;
    uint8_t  result = MAV_RESULT_ACCEPTED;
    uint8_t  progress = 0;
    int32_t  result_param2 = 0;
    uint8_t  target_system = 255;
    uint8_t  target_component = 190;
};
static_assert(sizeof(MsgCommandAck) == 10, "MsgCommandAck size mismatch");

struct MsgStatusText { // id=253, crc=83, size=54
    uint8_t  severity = MAV_SEVERITY_INFO;
    char     text[50] = {};
    uint16_t id = 0;
    uint8_t  chunk_seq = 0;
};
static_assert(sizeof(MsgStatusText) == 54, "MsgStatusText size mismatch");

#pragma pack(pop)

// Algoritmo oficial CRC-16/MCRF4XX (X.25) do MAVLink
inline void crc_accumulate(uint8_t data, uint16_t& crc) noexcept {
    uint8_t tmp = data ^ static_cast<uint8_t>(crc & 0xFFu);
    tmp ^= static_cast<uint8_t>(tmp << 4);
    crc = static_cast<uint16_t>(
        (crc >> 8) ^
        (static_cast<uint16_t>(tmp) << 8) ^
        (static_cast<uint16_t>(tmp) << 3) ^
        (static_cast<uint16_t>(tmp) >> 4));
}

inline uint16_t crc_calculate(const uint8_t* buf, size_t len, uint8_t crc_extra) noexcept {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc_accumulate(buf[i], crc);
    }
    crc_accumulate(crc_extra, crc);
    return crc;
}

// Serializa qualquer struct MAVLink v2 com truncamento de zeros finais (padrão MAVLink 2)
template <typename PayloadT>
inline size_t encode_mavlink_v2(uint32_t msgid,
                                uint8_t sysid,
                                uint8_t compid,
                                uint8_t& io_seq,
                                const PayloadT& payload,
                                uint8_t* out_frame_280) noexcept {
    const int crc_extra = get_crc_extra(msgid);
    if (crc_extra < 0 || !out_frame_280) {
        return 0;
    }

    const auto* raw = reinterpret_cast<const uint8_t*>(&payload);
    size_t len = sizeof(PayloadT);
    // MAVLink v2 trunca zeros à direita (mantendo no mínimo 1 byte se sizeof > 0)
    while (len > 1 && raw[len - 1] == 0u) {
        --len;
    }

    out_frame_280[0] = kMavlinkV2Stx;
    out_frame_280[1] = static_cast<uint8_t>(len);
    out_frame_280[2] = 0u; // incompat_flags
    out_frame_280[3] = 0u; // compat_flags
    out_frame_280[4] = io_seq++;
    out_frame_280[5] = sysid;
    out_frame_280[6] = compid;
    out_frame_280[7] = static_cast<uint8_t>(msgid & 0xFFu);
    out_frame_280[8] = static_cast<uint8_t>((msgid >> 8) & 0xFFu);
    out_frame_280[9] = static_cast<uint8_t>((msgid >> 16) & 0xFFu);

    std::memcpy(&out_frame_280[10], raw, len);

    const uint16_t crc = crc_calculate(&out_frame_280[1], 9u + len, static_cast<uint8_t>(crc_extra));
    out_frame_280[10 + len] = static_cast<uint8_t>(crc & 0xFFu);
    out_frame_280[11 + len] = static_cast<uint8_t>((crc >> 8) & 0xFFu);
    return 12u + len;
}

struct ParsedMavMessage {
    uint32_t msgid = 0;
    uint8_t  sysid = 0;
    uint8_t  compid = 0;
    uint8_t  seq = 0;
    uint8_t  payload_len = 0;
    uint8_t  payload[kMaxPayloadLen] = {}; // Zero-padded para leitura segura de extensões v2!

    template <typename T>
    T unpack() const noexcept {
        T out{};
        const size_t n = (sizeof(T) < kMaxPayloadLen) ? sizeof(T) : kMaxPayloadLen;
        std::memcpy(&out, payload, n);
        return out;
    }
};

// Parser byte-a-byte sem alocação dinâmica (suporta pacotes com e sem assinatura final de 13B)
class MavlinkV2Parser {
public:
    bool push_byte(uint8_t b, ParsedMavMessage& out_msg) noexcept {
        switch (state_) {
            case State::IDLE:
                if (b == kMavlinkV2Stx) {
                    hdr_[0] = b;
                    idx_ = 1;
                    state_ = State::HEADER;
                }
                break;

            case State::HEADER:
                hdr_[idx_++] = b;
                if (idx_ == 10) {
                    payload_len_ = hdr_[1];
                    sig_len_ = (hdr_[2] & 0x01u) ? 13u : 0u; // MAVLINK_IFLAG_SIGNED
                    std::memset(payload_, 0, sizeof(payload_));
                    idx_ = 0;
                    state_ = (payload_len_ > 0) ? State::PAYLOAD : State::CRC1;
                }
                break;

            case State::PAYLOAD:
                payload_[idx_++] = b;
                if (idx_ >= payload_len_) {
                    state_ = State::CRC1;
                }
                break;

            case State::CRC1:
                rx_crc_lo_ = b;
                state_ = State::CRC2;
                break;

            case State::CRC2: {
                const uint16_t rx_crc = static_cast<uint16_t>(rx_crc_lo_) |
                                        (static_cast<uint16_t>(b) << 8);
                if (sig_len_ > 0) {
                    saved_crc_ = rx_crc;
                    idx_ = 0;
                    state_ = State::SIGNATURE;
                    return false;
                }
                state_ = State::IDLE;
                return validate_and_emit(rx_crc, out_msg);
            }

            case State::SIGNATURE:
                if (++idx_ >= sig_len_) {
                    state_ = State::IDLE;
                    return validate_and_emit(saved_crc_, out_msg);
                }
                break;
        }
        return false;
    }

private:
    enum class State : uint8_t { IDLE, HEADER, PAYLOAD, CRC1, CRC2, SIGNATURE };

    bool validate_and_emit(uint16_t rx_crc, ParsedMavMessage& out_msg) noexcept {
        const uint32_t msgid = static_cast<uint32_t>(hdr_[7]) |
                               (static_cast<uint32_t>(hdr_[8]) << 8) |
                               (static_cast<uint32_t>(hdr_[9]) << 16);
        const int crc_extra = get_crc_extra(msgid);
        if (crc_extra < 0) {
            return false; // Mensagem desconhecida (ex.: PING/TIMESYNC ignorados com segurança)
        }

        uint16_t calc = 0xFFFFu;
        for (size_t i = 1; i < 10; ++i) {
            crc_accumulate(hdr_[i], calc);
        }
        for (size_t i = 0; i < payload_len_; ++i) {
            crc_accumulate(payload_[i], calc);
        }
        crc_accumulate(static_cast<uint8_t>(crc_extra), calc);

        if (calc != rx_crc) {
            return false;
        }

        out_msg.msgid = msgid;
        out_msg.seq = hdr_[4];
        out_msg.sysid = hdr_[5];
        out_msg.compid = hdr_[6];
        out_msg.payload_len = payload_len_;
        std::memset(out_msg.payload, 0, sizeof(out_msg.payload));
        std::memcpy(out_msg.payload, payload_, payload_len_);
        return true;
    }

    State state_ = State::IDLE;
    uint8_t hdr_[10] = {};
    uint8_t payload_[kMaxPayloadLen] = {};
    uint8_t payload_len_ = 0;
    uint8_t sig_len_ = 0;
    uint8_t idx_ = 0;
    uint8_t rx_crc_lo_ = 0;
    uint16_t saved_crc_ = 0;
};

} // namespace fc::mav
