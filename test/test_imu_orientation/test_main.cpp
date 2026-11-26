#include <cmath>
#include <unity.h>

#include "imu_orientation.h"
#include "mahony_ahrs.h"

using namespace fc;

void setUp() {}
void tearDown() {}

// Auxiliar: dado um vetor físico no referencial FLU alinhado ao DRONE (X=Frente, Y=Esq, Z=Cima),
// calcula o que um chip MEMS girado em `rot` (sentido horário visto de cima) leria nos pinos internos.
static Vec3f simulate_rotated_chip_flu(const Vec3f& drone_flu, ImuMountRotation rot) {
    switch (rot) {
        case ImuMountRotation::IMU_ROT_0_DEG:
            return {drone_flu.x, drone_flu.y, drone_flu.z};
        case ImuMountRotation::IMU_ROT_90_DEG:
            // +X_chip aponta p/ Direita (-Y_flu), +Y_chip aponta p/ Frente (+X_flu)
            return {-drone_flu.y, drone_flu.x, drone_flu.z};
        case ImuMountRotation::IMU_ROT_180_DEG:
            return {-drone_flu.x, -drone_flu.y, drone_flu.z};
        case ImuMountRotation::IMU_ROT_270_DEG:
            // +X_chip aponta p/ Esquerda (+Y_flu), +Y_chip aponta p/ Trás (-X_flu)
            return {drone_flu.y, -drone_flu.x, drone_flu.z};
    }
    return drone_flu;
}

// 1. Verifica que nas 4 orientações (0/90/180/270°), o drone plano na mesa dá Roll=0, Pitch=0 e AccelUp=0!
void test_all_four_rotations_level_table_zero_attitude() {
    const ImuMountRotation rots[4] = {
        ImuMountRotation::IMU_ROT_0_DEG,
        ImuMountRotation::IMU_ROT_90_DEG,
        ImuMountRotation::IMU_ROT_180_DEG,
        ImuMountRotation::IMU_ROT_270_DEG
    };

    for (ImuMountRotation r : rots) {
        // Na mesa plana, a reação normal aponta para CIMA (+1g em +Z_chip)
        const Vec3f chip_acc = simulate_rotated_chip_flu({0.0f, 0.0f, kGravityMs2}, r);
        const Vec3f body_frd = map_chip_flu_to_body_frd(chip_acc, r);

        TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, body_frd.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, body_frd.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, -kGravityMs2, body_frd.z);

        MahonyAHRS ahrs;
        TEST_ASSERT_TRUE(ahrs.align_to_gravity(body_frd, 0.0f));
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ahrs.euler().roll_deg());
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ahrs.euler().pitch_deg());
        TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, ahrs.earth_accel_up_ms2());
    }
}

// 2. Verifica Portão 5 (Roll Direita +, Pitch Nariz Cima +, Yaw Horário +) nas 4 rotações de PCB!
void test_portao5_signs_roll_pitch_yaw_all_four_rotations() {
    const ImuMountRotation rots[4] = {
        ImuMountRotation::IMU_ROT_0_DEG,
        ImuMountRotation::IMU_ROT_90_DEG,
        ImuMountRotation::IMU_ROT_180_DEG,
        ImuMountRotation::IMU_ROT_270_DEG
    };

    for (ImuMountRotation r : rots) {
        // A) Inclinar p/ DIREITA em +25°: asa esquerda sobe (+Y_flu recebe +g*sin(25°))
        const float deg25 = 25.0f * kDegToRad;
        const Vec3f chip_roll_r = simulate_rotated_chip_flu(
            {0.0f, kGravityMs2 * std::sin(deg25), kGravityMs2 * std::cos(deg25)}, r);
        MahonyAHRS ahrs_r;
        TEST_ASSERT_TRUE(ahrs_r.align_to_gravity(map_chip_flu_to_body_frd(chip_roll_r, r), 0.0f));
        TEST_ASSERT_FLOAT_WITHIN(0.05f, +25.0f, ahrs_r.euler().roll_deg());

        // B) Inclinar NARIZ P/ CIMA em +20°: frente sobe (+X_flu recebe +g*sin(20°))
        const float deg20 = 20.0f * kDegToRad;
        const Vec3f chip_pitch_up = simulate_rotated_chip_flu(
            {kGravityMs2 * std::sin(deg20), 0.0f, kGravityMs2 * std::cos(deg20)}, r);
        MahonyAHRS ahrs_p;
        TEST_ASSERT_TRUE(ahrs_p.align_to_gravity(map_chip_flu_to_body_frd(chip_pitch_up, r), 0.0f));
        TEST_ASSERT_FLOAT_WITHIN(0.05f, +20.0f, ahrs_p.euler().pitch_deg());

        // C) Girar NARIZ NO SENTIDO HORÁRIO visto de cima a 90°/s:
        //    No referencial FLU (+Z p/ Cima), giro horário é NEGATIVO (-90°/s em Z_flu)!
        const Vec3f chip_yaw_cw = simulate_rotated_chip_flu({0.0f, 0.0f, -90.0f * kDegToRad}, r);
        const Vec3f gyro_frd = map_chip_flu_to_body_frd(chip_yaw_cw, r);
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, +90.0f * kDegToRad, gyro_frd.z);
    }
}

int main(int /*argc*/, char** /*argv*/) {
    UNITY_BEGIN();
    RUN_TEST(test_all_four_rotations_level_table_zero_attitude);
    RUN_TEST(test_portao5_signs_roll_pitch_yaw_all_four_rotations);
    return UNITY_END();
}
