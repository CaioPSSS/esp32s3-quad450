# ESP32-S3 Quad-450 DIY Flight Controller & Ground Station

Monorepo completo para quadricóptero 450 mm DIY com **ESP32-S3 N16R8 (WROOM-1U / IPEX)**:
- `lib/flight_core/` — Núcleo matemático e de controle determinístico em C++17 puro (zero dependência de hardware).
- `lib/shared_protocol/` — Pacotes binários compactos ESP-NOW LR (`CRC-16-CCITT`) e tabela de 15 parâmetros MAVLink.
- `lib/mavlink_min/` — Implementação portátil *header-only* MAVLink v2 (`common.xml`, validada contra `pymavlink`).
- `lib/hal_sync/` — Primitivas *lock-free* (`SeqLock<T>`) entre Core 0 e Core 1.
- `lib/hal_esp32s3/` — Drivers de hardware ESP32-S3 (`ICM-42688-P` SPI 10 MHz + `INT1` 1 kHz, `BMP280` I2C 400 kHz, `LEDC` 14-bit @ 400 Hz, `ESP-NOW Long Range`, `Blackbox` PSRAM 6.5 MB e `BenchCli`).
- `lib/sim_physics/` — Dinâmica 6-DoF realista do quadricóptero de 900 g (motores A2212, sag de bateria 3S, vibração 185 Hz).
- `src/sitl_pc/` — Simulador SITL com servidor MAVLink v2 UDP (`:14550`) para **QGroundControl**.
- `src/ground_tx/` — Firmware Dual-Core do Controle Caseiro (ADC1 16× oversampling, Wizard NVS, OLED SSD1306, Ponte USB-OTG MAVLink v2).
- `src/drone_fc/` — Firmware Dual-Core da Controladora de Voo (Core 1 @ 1000 Hz ISR `INT1` + Core 0 @ ESP-NOW LR / Baro / Blackbox PSRAM / CLI).

## Comandos Principais

```bash
# 1. Rodar todas as suítes de testes unitários + voo 6-DoF em malha fechada no PC
pio test -e native_test

# 2. Compilar e rodar demonstração de voo no Simulador SITL 6-DoF
pio run -e sitl_pc && ./.pio/build/sitl_pc/program --headless-demo

# 3. Rodar SITL em tempo real aguardando conexão do QGroundControl (UDP 127.0.0.1:14550)
./.pio/build/sitl_pc/program

# 4. Validar protocolo MAVLink v2 e analisador FFT da Blackbox
python3 tools/verify_sitl_mavlink.py
python3 tools/analyze_blackbox.py --self-test

# 5. Compilar / Gravar o firmware do Controle Caseiro (ESP32-S3)
pio run -e ground_tx
# pio run -e ground_tx -t upload

# 6. Compilar / Gravar o firmware da Controladora do Drone (ESP32-S3)
pio run -e drone_fc
# pio run -e drone_fc -t upload && pio device monitor -b 115200
```

## CLI Interativa de Bancada (`drone_fc` USB-C @ 115200 bps — Portões 5 a 8)

- `status` — Diagnóstico completo (`WHO_AM_I`, `INT1 Hz`, `dt_us`, atitude FRD, `BMP280`, `VBAT`, `RSSI`, `PreArmFlags`).
- `imu_test` / `imu_orient <0|90|180|270>` / `imu_cal` — Verificação de sinais FRD no **Portão 5** e calibração.
- `bench on` → `motor_test <1..4> <0..25>` — Teste individual de motores **sem hélices** no **Portão 6** (corta em 3 s ou ao pressionar qualquer tecla).
- `bench on` → `esc_cal` — Assistente guiado `2000 µs → 1000 µs` de calibração dos 4 ESCs (**sem hélices**).
- `bb_info` / `bb_dump_csv` / `bb_dump_bin` — Exportação da Caixa-Preta PSRAM de 6.5 MB para `python3 tools/analyze_blackbox.py`.
