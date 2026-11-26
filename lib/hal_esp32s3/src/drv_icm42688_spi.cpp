#include "drv_icm42688_spi.h"

#include <cmath>

namespace fc::hal {

namespace {

// Registradores ICM-42688-P / ICM-42605 (Bank 0)
constexpr uint8_t kIcmRegDeviceConfig     = 0x11;
constexpr uint8_t kIcmRegIntConfig        = 0x14;
constexpr uint8_t kIcmRegAccelDataX1      = 0x1F; // 0x1F..0x2A (6B Accel + 6B Gyro Big-Endian)
constexpr uint8_t kIcmRegPwrMgmt0         = 0x4E;
constexpr uint8_t kIcmRegGyroConfig0      = 0x4F;
constexpr uint8_t kIcmRegAccelConfig0     = 0x50;
constexpr uint8_t kIcmRegGyroAccelConfig0 = 0x52;
constexpr uint8_t kIcmRegIntConfig0       = 0x63;
constexpr uint8_t kIcmRegIntConfig1       = 0x64; // Bit 4 (INT_ASYNC_RESET) DEVE ser zerado!
constexpr uint8_t kIcmRegIntSource0       = 0x65;
constexpr uint8_t kIcmRegWhoAmI           = 0x75;
constexpr uint8_t kIcmRegBankSel          = 0x76;

// Registradores Família MPU-6500 / MPU-9250 / ICM-20602 / MPU-6000
constexpr uint8_t kMpuRegSmplrtDiv   = 0x19;
constexpr uint8_t kMpuRegConfig      = 0x1A;
constexpr uint8_t kMpuRegGyroConfig  = 0x1B;
constexpr uint8_t kMpuRegAccelConfig = 0x1C;
constexpr uint8_t kMpuRegAccelCfg2   = 0x1D;
constexpr uint8_t kMpuRegIntPinCfg   = 0x37;
constexpr uint8_t kMpuRegIntEnable   = 0x38;
constexpr uint8_t kMpuRegAccelXoutH  = 0x3B; // 0x3B..0x48 (6B Accel + 2B Temp + 6B Gyro Big-Endian)
constexpr uint8_t kMpuRegUserCtrl    = 0x6A;
constexpr uint8_t kMpuRegPwrMgmt1    = 0x6B;
constexpr uint8_t kMpuRegWhoAmI      = 0x75;

// Registradores BMI160 (Nota: leituras SPI no BMI160 têm 1 dummy byte inicial no protocolo Bosch?
// Não! O BMI160 NÃO tem dummy byte em SPI (só o BMI270/BMI088 têm); o BMI160 lê direto após addr|0x80!)
constexpr uint8_t kBmi160RegChipId    = 0x00; // 0xD1
constexpr uint8_t kBmi160RegDataGyroX = 0x0C; // 0x0C..0x17 (6B Gyro + 6B Accel Little-Endian!)
constexpr uint8_t kBmi160RegAccConf   = 0x40;
constexpr uint8_t kBmi160RegAccRange  = 0x41;
constexpr uint8_t kBmi160RegGyrConf   = 0x42;
constexpr uint8_t kBmi160RegGyrRange  = 0x43;
constexpr uint8_t kBmi160RegIntEn1    = 0x51;
constexpr uint8_t kBmi160RegIntOutCtrl= 0x53;
constexpr uint8_t kBmi160RegIntMap1   = 0x56;
constexpr uint8_t kBmi160RegCmd       = 0x7E;

// Escalas físicas (±16g = 2048 LSB/g; ±2000 dps = 16.4 LSB/(deg/s))
constexpr float kAccelScale16gMs2    = fc::kGravityMs2 / 2048.0f;
constexpr float kGyroScale2000DpsRad = fc::kDegToRad / 16.4f;

inline int16_t be16(uint8_t hi, uint8_t lo) noexcept {
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo));
}

inline int16_t le16(uint8_t lo, uint8_t hi) noexcept {
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo));
}

} // namespace

uint8_t ImuSpiDriver::read_reg(uint8_t reg, uint32_t clock_hz) noexcept {
    if (!spi_) return 0;
    spi_->beginTransaction(SPISettings(clock_hz, MSBFIRST, SPI_MODE3));
    digitalWrite(pins_.cs, LOW);
    spi_->transfer(static_cast<uint8_t>(reg | 0x80u));
    const uint8_t val = spi_->transfer(0x00);
    digitalWrite(pins_.cs, HIGH);
    spi_->endTransaction();
    return val;
}

void ImuSpiDriver::write_reg(uint8_t reg, uint8_t val, uint32_t clock_hz) noexcept {
    if (!spi_) return;
    spi_->beginTransaction(SPISettings(clock_hz, MSBFIRST, SPI_MODE3));
    digitalWrite(pins_.cs, LOW);
    spi_->transfer(static_cast<uint8_t>(reg & 0x7Fu));
    spi_->transfer(val);
    digitalWrite(pins_.cs, HIGH);
    spi_->endTransaction();
}

void ImuSpiDriver::read_burst(uint8_t start_reg, uint8_t* buf, size_t len, uint32_t clock_hz) noexcept {
    if (!spi_ || !buf || len == 0) return;
    spi_->beginTransaction(SPISettings(clock_hz, MSBFIRST, SPI_MODE3));
    digitalWrite(pins_.cs, LOW);
    spi_->transfer(static_cast<uint8_t>(start_reg | 0x80u));
    spi_->transferBytes(nullptr, buf, static_cast<uint32_t>(len));
    digitalWrite(pins_.cs, HIGH);
    spi_->endTransaction();
}

bool ImuSpiDriver::begin(SPIClass& spi, const ImuSpiPins& pins, ImuMountRotation rot) noexcept {
    spi_ = &spi;
    pins_ = pins;
    rot_ = rot;
    model_ = ImuChipModel::NONE;
    who_am_i_ = 0x00;

    pinMode(pins_.cs, OUTPUT);
    digitalWrite(pins_.cs, HIGH);
    if (pins_.int1 >= 0) {
        pinMode(pins_.int1, INPUT);
    }

    spi_->begin(pins_.sck, pins_.miso, pins_.mosi, pins_.cs);
    delay(10);

    // Pulso dummy em CS garante que sensores híbridos I2C/SPI (como BMI160/ICM) travem em modo SPI
    digitalWrite(pins_.cs, LOW);
    delayMicroseconds(20);
    digitalWrite(pins_.cs, HIGH);
    delay(5);

    // 1. Tenta detectar ICM-42688-P (0x47), ICM-42605 (0x42) ou família MPU-6500/9250/ICM-20602 em 0x75
    write_reg(kIcmRegBankSel, 0x00);
    delay(2);
    const uint8_t id75 = read_reg(kIcmRegWhoAmI);

    if (id75 == 0x47 || id75 == 0x42) {
        who_am_i_ = id75;
        model_ = (id75 == 0x47) ? ImuChipModel::ICM42688P : ImuChipModel::ICM42605;
        return init_icm426xx();
    }

    if (id75 == 0x70 || id75 == 0x71 || id75 == 0x73 || id75 == 0x12 || id75 == 0x68) {
        who_am_i_ = id75;
        model_ = ImuChipModel::MPU6500;
        return init_mpu6500_family();
    }

    // 2. Fallback para BMI160 SPI (CHIP_ID = 0xD1 no reg 0x00)
    (void)read_reg(0x7F); // Ativação formal SPI do BMI160 conforme datasheet Bosch
    delay(2);
    const uint8_t id00 = read_reg(kBmi160RegChipId);
    if (id00 == 0xD1) {
        who_am_i_ = id00;
        model_ = ImuChipModel::BMI160;
        return init_bmi160();
    }

    who_am_i_ = id75;
    return false;
}

bool ImuSpiDriver::init_icm426xx() noexcept {
    // Reset de software limpo
    write_reg(kIcmRegBankSel, 0x00);
    write_reg(kIcmRegDeviceConfig, 0x01);
    delay(10);
    write_reg(kIcmRegBankSel, 0x00);

    // INT1: Push-Pull (bit 1 = 1), Pulsed (bit 2 = 0), Active-High (bit 0 = 1) -> 0x03
    write_reg(kIcmRegIntConfig, 0x03);

    // ERRATA CRÍTICA DO DATASHEET ICM-42688-P (Seção 14.50):
    // O bit 4 (`INT_ASYNC_RESET`) de `INT_CONFIG1` (0x64) vem em 1 por padrão e DEVE ser
    // alterado para 0 para operação determinística do pino INT1 em ODR >= 1 kHz!
    uint8_t int_cfg1 = read_reg(kIcmRegIntConfig1);
    int_cfg1 &= static_cast<uint8_t>(~(1u << 4));
    write_reg(kIcmRegIntConfig1, int_cfg1);

    // Limpa flag Data-Ready automaticamente na leitura dos registradores de dados
    write_reg(kIcmRegIntConfig0, 0x00);

    // Giroscópio: ±2000 dps (FS_SEL=000) + ODR = 1 kHz (0110) -> 0x06
    write_reg(kIcmRegGyroConfig0, 0x06);

    // Acelerômetro: ±16 g (FS_SEL=000) + ODR = 1 kHz (0110) -> 0x06
    write_reg(kIcmRegAccelConfig0, 0x06);

    // Banda interna UI ampla (ODR/4 = ~250 Hz, mínima latência de fase) p/ deixar nosso Biquad filtrar!
    write_reg(kIcmRegGyroAccelConfig0, 0x11);

    // Roteia UI Data-Ready (bit 3) para o pino físico INT1 (GPIO14)
    write_reg(kIcmRegIntSource0, 0x08);

    // Liga Giroscópio e Acelerômetro em modo Low-Noise (bits [3:0] = 1111 = 0x0F)
    // Datasheet exige esperar >= 45 ms após ligar o Giroscópio!
    write_reg(kIcmRegPwrMgmt0, 0x0F);
    delay(55);

    return read_reg(kIcmRegWhoAmI) == who_am_i_;
}

bool ImuSpiDriver::init_mpu6500_family() noexcept {
    write_reg(kMpuRegPwrMgmt1, 0x80); // Device Reset
    delay(50);
    write_reg(kMpuRegPwrMgmt1, 0x01); // Auto-select PLL clock
    delay(15);
    write_reg(kMpuRegUserCtrl, 0x10); // I2C_IF_DIS = 1 (trava em modo SPI puro!)
    delay(5);

    write_reg(kMpuRegSmplrtDiv, 0x00);   // 1 kHz Sample Rate (1000 / (1 + 0))
    write_reg(kMpuRegConfig, 0x01);      // Gyro DLPF = 184 Hz (taxa base interna = 1 kHz)
    write_reg(kMpuRegGyroConfig, 0x18);  // ±2000 dps
    write_reg(kMpuRegAccelConfig, 0x18); // ±16 g
    write_reg(kMpuRegAccelCfg2, 0x01);   // Accel DLPF = 184 Hz @ 1 kHz

    // INT1: Push-pull, Active-High, pulso 50 µs, limpa em qualquer leitura (INT_ANYRD_2CLEAR)
    write_reg(kMpuRegIntPinCfg, 0x10);
    write_reg(kMpuRegIntEnable, 0x01);   // RAW_RDY_EN = 1
    delay(20);

    return read_reg(kMpuRegWhoAmI) == who_am_i_;
}

bool ImuSpiDriver::init_bmi160() noexcept {
    write_reg(kBmi160RegCmd, 0xB6); // Soft reset
    delay(20);
    (void)read_reg(0x7F);           // Reativa SPI após soft reset
    delay(5);

    write_reg(kBmi160RegCmd, 0x11); // Accel Normal Mode
    delay(10);
    write_reg(kBmi160RegCmd, 0x15); // Gyro Normal Mode
    delay(60);

    // Accel: ODR 800/1600 Hz -> 0x2B (800 Hz normal) ou 0x2C (1600 Hz), Range ±16g (0x0C)
    write_reg(kBmi160RegAccConf, 0x2B);
    write_reg(kBmi160RegAccRange, 0x0C);
    // Gyro: ODR 800 Hz (0x2B), Range ±2000 dps (0x00)
    write_reg(kBmi160RegGyrConf, 0x2B);
    write_reg(kBmi160RegGyrRange, 0x00);

    // INT1: Push-pull Active-High habilitado + Data-Ready mapeado p/ INT1
    write_reg(kBmi160RegIntOutCtrl, 0x0A);
    write_reg(kBmi160RegIntMap1, 0x80);
    write_reg(kBmi160RegIntEn1, 0x10);
    delay(10);

    return read_reg(kBmi160RegChipId) == 0xD1;
}

ImuSample ImuSpiDriver::read_sample(uint32_t now_us) noexcept {
    ImuSample sample{};
    sample.timestamp_us = now_us;

    if (model_ == ImuChipModel::NONE || !spi_) {
        sample.healthy = false;
        return sample;
    }

    int16_t ax_raw = 0, ay_raw = 0, az_raw = 0;
    int16_t gx_raw = 0, gy_raw = 0, gz_raw = 0;

    if (model_ == ImuChipModel::ICM42688P || model_ == ImuChipModel::ICM42605) {
        uint8_t raw[12] = {};
        read_burst(kIcmRegAccelDataX1, raw, 12, 10000000u);
        ax_raw = be16(raw[0],  raw[1]);
        ay_raw = be16(raw[2],  raw[3]);
        az_raw = be16(raw[4],  raw[5]);
        gx_raw = be16(raw[6],  raw[7]);
        gy_raw = be16(raw[8],  raw[9]);
        gz_raw = be16(raw[10], raw[11]);
    } else if (model_ == ImuChipModel::MPU6500) {
        uint8_t raw[14] = {};
        read_burst(kMpuRegAccelXoutH, raw, 14, 10000000u);
        ax_raw = be16(raw[0],  raw[1]);
        ay_raw = be16(raw[2],  raw[3]);
        az_raw = be16(raw[4],  raw[5]);
        // raw[6..7] = Temperatura interna
        gx_raw = be16(raw[8],  raw[9]);
        gy_raw = be16(raw[10], raw[11]);
        gz_raw = be16(raw[12], raw[13]);
    } else if (model_ == ImuChipModel::BMI160) {
        uint8_t raw[12] = {};
        read_burst(kBmi160RegDataGyroX, raw, 12, 10000000u);
        gx_raw = le16(raw[0],  raw[1]);
        gy_raw = le16(raw[2],  raw[3]);
        gz_raw = le16(raw[4],  raw[5]);
        ax_raw = le16(raw[6],  raw[7]);
        ay_raw = le16(raw[8],  raw[9]);
        az_raw = le16(raw[10], raw[11]);
    }

    // Proteção contra barramento SPI desconectado (MISO flutuando em 0x00 ou 0xFF em todos os bytes)
    const bool all_zero = (ax_raw == 0 && ay_raw == 0 && az_raw == 0 &&
                           gx_raw == 0 && gy_raw == 0 && gz_raw == 0);
    const bool all_ffff = (ax_raw == -1 && ay_raw == -1 && az_raw == -1 &&
                           gx_raw == -1 && gy_raw == -1 && gz_raw == -1);
    if (all_zero || all_ffff) {
        sample.healthy = false;
        return sample;
    }

    const Vec3f accel_flu{
        static_cast<float>(ax_raw) * kAccelScale16gMs2,
        static_cast<float>(ay_raw) * kAccelScale16gMs2,
        static_cast<float>(az_raw) * kAccelScale16gMs2
    };
    const Vec3f gyro_flu{
        static_cast<float>(gx_raw) * kGyroScale2000DpsRad,
        static_cast<float>(gy_raw) * kGyroScale2000DpsRad,
        static_cast<float>(gz_raw) * kGyroScale2000DpsRad
    };

    sample.accel_ms2 = map_chip_flu_to_body_frd(accel_flu, rot_);
    sample.gyro_rads = map_chip_flu_to_body_frd(gyro_flu, rot_);
    sample.healthy = std::isfinite(sample.accel_ms2.x) && std::isfinite(sample.gyro_rads.x);
    return sample;
}

const char* ImuSpiDriver::chip_name() const noexcept {
    switch (model_) {
        case ImuChipModel::ICM42688P: return "ICM-42688-P";
        case ImuChipModel::ICM42605:  return "ICM-42605";
        case ImuChipModel::MPU6500:   return "MPU-6500/9250/ICM20602";
        case ImuChipModel::BMI160:    return "BMI160";
        default:                      return "NAO_DETECTADO";
    }
}

} // namespace fc::hal
