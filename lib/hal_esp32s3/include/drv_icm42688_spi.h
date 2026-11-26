#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <cstdint>

#include "fc_types.h"
#include "imu_orientation.h"

namespace fc::hal {

enum class ImuChipModel : uint8_t {
    NONE       = 0,
    ICM42688P  = 1,
    ICM42605   = 2,
    MPU6500    = 3, // Inclui MPU-6000 / MPU-6500 / MPU-9250 / ICM-20602 (mesmo mapa SPI!)
    BMI160     = 4
};

struct ImuSpiPins {
    int cs   = 10;
    int mosi = 11;
    int sck  = 12;
    int miso = 13;
    int int1 = 14;
};

/**
 * Driver SPI Determinístico de 10 MHz para IMU Principal (ICM-42688-P @ 1 kHz + INT1).
 *
 * Recursos de Engenharia:
 * - Auto-detecção de chip (`WHO_AM_I`): `ICM-42688-P` (0x47), `ICM-42605` (0x42),
 *   família `MPU-6500/9250/ICM-20602` (0x70/0x71/0x73/0x12/0x68) e `BMI160` (0xD1).
 * - Aplica a correção obrigatória de errata do datasheet do `ICM-42688-P`
 *   (`INT_CONFIG1` bit `INT_ASYNC_RESET = 0`) para pulso `INT1` confiável a 1000 Hz!
 * - Leitura burst de 12/14 bytes a 10 MHz (~16 µs) e conversão imediata `FLU -> FRD`.
 */
class ImuSpiDriver {
public:
    bool begin(SPIClass& spi,
               const ImuSpiPins& pins = ImuSpiPins{},
               ImuMountRotation rot = ImuMountRotation::IMU_ROT_0_DEG) noexcept;

    /**
     * Lê Acelerômetro (±16g -> m/s²) e Giroscópio (±2000 dps -> rad/s) via SPI @ 10 MHz
     * e devolve `ImuSample` já rotacionado para o referencial aeronáutico FRD.
     */
    ImuSample read_sample(uint32_t now_us) noexcept;

    void set_mount_rotation(ImuMountRotation rot) noexcept { rot_ = rot; }
    ImuMountRotation mount_rotation() const noexcept { return rot_; }

    bool is_detected() const noexcept { return model_ != ImuChipModel::NONE; }
    ImuChipModel model() const noexcept { return model_; }
    uint8_t who_am_i() const noexcept { return who_am_i_; }
    const char* chip_name() const noexcept;

private:
    uint8_t read_reg(uint8_t reg, uint32_t clock_hz = 1000000u) noexcept;
    void write_reg(uint8_t reg, uint8_t val, uint32_t clock_hz = 1000000u) noexcept;
    void read_burst(uint8_t start_reg, uint8_t* buf, size_t len, uint32_t clock_hz = 10000000u) noexcept;

    bool init_icm426xx() noexcept;
    bool init_mpu6500_family() noexcept;
    bool init_bmi160() noexcept;

    SPIClass* spi_ = nullptr;
    ImuSpiPins pins_{};
    ImuMountRotation rot_ = ImuMountRotation::IMU_ROT_0_DEG;
    ImuChipModel model_ = ImuChipModel::NONE;
    uint8_t who_am_i_ = 0x00;
};

} // namespace fc::hal
