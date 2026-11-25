#include <atomic>
#include <limits>
#include <thread>
#include <unity.h>

#include "fc_types.h"
#include "param_registry.h"
#include "radio_packets.h"
#include "seqlock.h"

using namespace fc;
using namespace fc::proto;
using namespace fc::sync;

void setUp() {}
void tearDown() {}

void test_vec3_operations() {
    const Vec3f x_axis{1.0f, 0.0f, 0.0f};
    const Vec3f y_axis{0.0f, 1.0f, 0.0f};
    const Vec3f z_axis = x_axis.cross(y_axis); // Regra da mão direita FRD: X cross Y = +Z

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, z_axis.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, z_axis.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, z_axis.z);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, x_axis.dot(y_axis));
}

void test_seqlock_read_write_consistency() {
    RcCommand initial{};
    initial.throttle = 0.0f;
    SeqLock<RcCommand> lock(initial);

    TEST_ASSERT_EQUAL_UINT32(0u, lock.sequence());

    RcCommand cmd{};
    cmd.throttle = 0.42f;
    cmd.roll = -0.25f;
    cmd.pitch = 0.15f;
    cmd.arm_switch = true;
    cmd.link_alive = true;

    lock.write(cmd);
    TEST_ASSERT_EQUAL_UINT32(2u, lock.sequence());

    const RcCommand out = lock.read();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.42f, out.throttle);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.25f, out.roll);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.15f, out.pitch);
    TEST_ASSERT_TRUE(out.arm_switch);
    TEST_ASSERT_TRUE(out.link_alive);
}

// Stress-test concorrente simulando Core 0 escrevendo e Core 1 lendo 50.000 vezes:
// garante zero leituras rasgadas (torn reads)!
void test_seqlock_multithreaded_no_torn_reads() {
    struct CoherentPayload {
        uint32_t a = 0;
        uint32_t b = 0;
        uint32_t c = 0;
        uint32_t inv_a = ~0u;
    };

    SeqLock<CoherentPayload> shared{};
    std::atomic<bool> writer_done{false};
    std::atomic<uint32_t> torn_count{0};

    std::thread writer([&]() {
        for (uint32_t i = 1; i <= 50000u; ++i) {
            CoherentPayload p{i, i * 3u, i * 7u, ~i};
            shared.write(p);
        }
        writer_done.store(true, std::memory_order_release);
    });

    std::thread reader([&]() {
        while (!writer_done.load(std::memory_order_acquire)) {
            const CoherentPayload snap = shared.read();
            if (snap.b != snap.a * 3u || snap.c != snap.a * 7u || snap.inv_a != ~snap.a) {
                torn_count.fetch_add(1u, std::memory_order_relaxed);
            }
        }
    });

    writer.join();
    reader.join();
    TEST_ASSERT_EQUAL_UINT32(0u, torn_count.load());
}

void test_radio_packet_crc_and_sizes() {
    TEST_ASSERT_EQUAL_UINT32(20u, sizeof(RcCommandPkt));
    TEST_ASSERT_EQUAL_UINT32(34u, sizeof(TelemFastPkt));
    TEST_ASSERT_EQUAL_UINT32(16u, sizeof(ParamSyncPkt));
    TEST_ASSERT_EQUAL_UINT32(28u, sizeof(GpsNavPkt));

    RcCommandPkt pkt{};
    pkt.seq = 101;
    pkt.thr_permil = 380;
    pkt.roll_permil = -120;
    pkt.arm_magic = kArmMagicValue;
    seal_packet(pkt);

    TEST_ASSERT_TRUE(verify_packet(pkt, PKT_RC_COMMAND));
    TEST_ASSERT_FALSE(verify_packet(pkt, PKT_TELEM_FAST));

    // Corromper 1 único bit no payload deve invalidar o CRC16!
    pkt.thr_permil ^= 0x0001u;
    TEST_ASSERT_FALSE(verify_packet(pkt, PKT_RC_COMMAND));
}

void test_param_table_bounds_and_clamping() {
    FlightParams params{};
    for (uint8_t i = 0; i < PARAM_COUNT; ++i) {
        float val = 0.0f;
        TEST_ASSERT_TRUE(get_param_by_index(params, i, val));
        TEST_ASSERT_TRUE(val >= kParamTable[i].min_val);
        TEST_ASSERT_TRUE(val <= kParamTable[i].max_val);
    }

    // Busca por nome MAVLink e clamp contra valores perigosos ou NaN
    const int idx = find_param_by_name("MAX_TILT_DEG");
    TEST_ASSERT_EQUAL_INT(PARAM_MAX_TILT_DEG, idx);

    TEST_ASSERT_TRUE(set_param_by_index(params, PARAM_MAX_TILT_DEG, 999.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 55.0f, params.max_tilt_deg); // Clampado no teto seguro!

    TEST_ASSERT_FALSE(set_param_by_index(params, PARAM_MAX_TILT_DEG,
                                         std::numeric_limits<float>::quiet_NaN()));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 55.0f, params.max_tilt_deg);
}

int main(int /*argc*/, char** /*argv*/) {
    UNITY_BEGIN();
    RUN_TEST(test_vec3_operations);
    RUN_TEST(test_seqlock_read_write_consistency);
    RUN_TEST(test_seqlock_multithreaded_no_torn_reads);
    RUN_TEST(test_radio_packet_crc_and_sizes);
    RUN_TEST(test_param_table_bounds_and_clamping);
    return UNITY_END();
}
