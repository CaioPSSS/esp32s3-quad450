#pragma once

#include <cstdint>
#include "fc_types.h"

namespace fc {

/**
 * Orientação física de montagem do chip IMU (padrão MEMS FLU: +X=Frente, +Y=Esquerda, +Z=Cima
 * visto de topo na serigrafia do módulo) sobre a placa perfurada 7x9 cm em relação ao nariz do drone.
 *
 * Rotação medida em sentido HORÁRIO visto de cima:
 * - IMU_ROT_0_DEG   : Seta +X do módulo aponta p/ FRENTE do drone
 * - IMU_ROT_90_DEG  : Seta +X do módulo aponta p/ DIREITA do drone
 * - IMU_ROT_180_DEG : Seta +X do módulo aponta p/ TRÁS do drone
 * - IMU_ROT_270_DEG : Seta +X do módulo aponta p/ ESQUERDA do drone
 */
enum class ImuMountRotation : uint8_t {
    IMU_ROT_0_DEG   = 0,
    IMU_ROT_90_DEG  = 1,
    IMU_ROT_180_DEG = 2,
    IMU_ROT_270_DEG = 3
};

/**
 * Converte um vetor 3D medido no referencial nativo do chip MEMS (FLU: +X chip, +Y chip, +Z chip Up)
 * para o referencial aeronáutico estrito da controladora (FRD: +X Frente, +Y Direita, +Z Baixo).
 *
 * Propriedade matemática fundamental: det(R_flu_to_frd) = +1 (rotação própria de 180° em torno do
 * eixo horizontal), valendo identicamente tanto para Velocidade Angular (gyro_rads) quanto para
 * Força Específica (accel_ms2)!
 *
 * Em repouso plano na mesa (chip lendo [0, 0, +9.81 m/s²]):
 *   -> retorna [0, 0, -9.81 m/s²] em FRD (Roll = 0°, Pitch = 0° no MahonyAHRS).
 */
inline constexpr Vec3f map_chip_flu_to_body_frd(const Vec3f& chip_flu,
                                                ImuMountRotation rot = ImuMountRotation::IMU_ROT_0_DEG) noexcept {
    // Primeiro alinha os eixos horizontais do chip para FLU alinhado ao nariz do drone:
    float x_fwd = chip_flu.x;
    float y_left = chip_flu.y;

    switch (rot) {
        case ImuMountRotation::IMU_ROT_0_DEG:
            // +X_chip = Frente, +Y_chip = Esquerda
            x_fwd  =  chip_flu.x;
            y_left =  chip_flu.y;
            break;
        case ImuMountRotation::IMU_ROT_90_DEG:
            // +X_chip aponta p/ Direita (-Esquerda), +Y_chip aponta p/ Frente
            x_fwd  =  chip_flu.y;
            y_left = -chip_flu.x;
            break;
        case ImuMountRotation::IMU_ROT_180_DEG:
            // +X_chip aponta p/ Trás, +Y_chip aponta p/ Direita
            x_fwd  = -chip_flu.x;
            y_left = -chip_flu.y;
            break;
        case ImuMountRotation::IMU_ROT_270_DEG:
            // +X_chip aponta p/ Esquerda, +Y_chip aponta p/ Trás
            x_fwd  = -chip_flu.y;
            y_left =  chip_flu.x;
            break;
    }

    // Converte FLU (Frente, Esquerda, Cima) -> FRD (Frente, Direita, Baixo)
    return Vec3f{x_fwd, -y_left, -chip_flu.z};
}

} // namespace fc
