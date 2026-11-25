#pragma once

#include <cstddef>
#include <cstdint>

namespace fc::proto {

constexpr uint8_t kPacketMagic = 0xE3;

enum PacketType : uint8_t {
    PKT_RC_COMMAND = 0xA1,
    PKT_TELEM_FAST = 0xB1,
    PKT_PARAM_SYNC = 0xC1,
    PKT_GPS_NAV    = 0xD1
};

enum ParamOp : uint8_t {
    PARAM_OP_GET      = 0,
    PARAM_OP_SET      = 1,
    PARAM_OP_ACK      = 2,
    PARAM_OP_SAVE_NVS = 3
};

constexpr uint8_t kArmMagicValue    = 0x5A;
constexpr uint8_t kDisarmMagicValue = 0x00;

#pragma pack(push, 1)

// TX -> Drone @ 100 Hz (20 bytes exatos)
struct RcCommandPkt {
    uint8_t  magic = kPacketMagic;
    uint8_t  type  = PKT_RC_COMMAND;
    uint16_t seq   = 0;
    uint16_t thr_permil   = 0;     // 0 .. 1000
    int16_t  roll_permil  = 0;     // -1000 .. +1000
    int16_t  pitch_permil = 0;     // -1000 .. +1000
    int16_t  yaw_permil   = 0;     // -1000 .. +1000
    uint8_t  arm_magic    = kDisarmMagicValue;
    uint8_t  flight_mode  = 0;
    uint8_t  aux_flags    = 0;
    uint8_t  reserved[3]  = {0, 0, 0};
    uint16_t crc16        = 0;
};
static_assert(sizeof(RcCommandPkt) == 20, "RcCommandPkt deve ter exatos 20 bytes");

// Drone -> TX @ 20 Hz (34 bytes exatos)
struct TelemFastPkt {
    uint8_t  magic = kPacketMagic;
    uint8_t  type  = PKT_TELEM_FAST;
    uint16_t seq   = 0;
    uint8_t  state = 0;
    uint8_t  flight_mode  = 0;
    uint16_t prearm_flags = 0;
    int16_t  roll_deg10   = 0;
    int16_t  pitch_deg10  = 0;
    int16_t  yaw_deg10    = 0;
    int32_t  alt_cm       = 0;
    int16_t  vz_cms       = 0;
    uint16_t vbat_mv      = 0;
    uint8_t  motor_pct[4] = {0, 0, 0, 0};
    int8_t   drone_rssi_dbm = -100;
    uint8_t  cpu_load_pct = 0;
    uint16_t loop_dt_us   = 1000;
    uint16_t ack_seq      = 0;
    uint16_t crc16        = 0;
};
static_assert(sizeof(TelemFastPkt) == 34, "TelemFastPkt deve ter exatos 34 bytes");

// Bidirecional sob demanda c/ ACK (16 bytes exatos)
struct ParamSyncPkt {
    uint8_t  magic     = kPacketMagic;
    uint8_t  type      = PKT_PARAM_SYNC;
    uint8_t  req_id    = 0;
    uint8_t  op        = PARAM_OP_GET;
    uint8_t  param_idx = 0;
    uint8_t  status    = 0;
    uint16_t reserved  = 0;
    float    value     = 0.0f;
    uint16_t padding   = 0;
    uint16_t crc16     = 0;
};
static_assert(sizeof(ParamSyncPkt) == 16, "ParamSyncPkt deve ter exatos 16 bytes");

// Bidirecional @ 5 Hz na Fase 2 GPS (28 bytes exatos)
struct GpsNavPkt {
    uint8_t  magic = kPacketMagic;
    uint8_t  type  = PKT_GPS_NAV;
    uint16_t seq   = 0;
    int32_t  lat_e7 = 0;
    int32_t  lon_e7 = 0;
    int32_t  alt_msl_mm = 0;
    uint16_t home_dist_dm = 0;
    uint16_t ground_speed_cms = 0;
    int16_t  heading_deg10 = 0;
    uint8_t  sats = 0;
    uint8_t  fix_type = 0;
    uint8_t  hdop_10 = 99;
    uint8_t  nav_cmd = 0;
    uint16_t crc16 = 0;
};
static_assert(sizeof(GpsNavPkt) == 28, "GpsNavPkt deve ter exatos 28 bytes");

#pragma pack(pop)

// CRC-16-CCITT (polinômio 0x1021, init 0xFFFF)
inline uint16_t crc16_ccitt(const uint8_t* data, size_t len) noexcept {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000u) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

template <typename Pkt>
inline void seal_packet(Pkt& pkt) noexcept {
    pkt.magic = kPacketMagic;
    pkt.crc16 = crc16_ccitt(reinterpret_cast<const uint8_t*>(&pkt), sizeof(Pkt) - sizeof(uint16_t));
}

template <typename Pkt>
inline bool verify_packet(const Pkt& pkt, uint8_t expected_type) noexcept {
    if (pkt.magic != kPacketMagic || pkt.type != expected_type) {
        return false;
    }
    const uint16_t calc = crc16_ccitt(reinterpret_cast<const uint8_t*>(&pkt), sizeof(Pkt) - sizeof(uint16_t));
    return calc == pkt.crc16;
}

} // namespace fc::proto
