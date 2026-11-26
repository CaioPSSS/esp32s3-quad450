# ESP32-S3 Quad-450 DIY Flight Controller & Ground Station (Fase 1 Completa — Sem GPS)

Monorepo completo para construção, simulação 6-DoF e voo de um quadricóptero **450 mm DIY** e seu **Controle Remoto Caseiro Long-Range**, baseados em **2× ESP32-S3-DevKitC-1 N16R8 (WROOM-1U c/ antena externa IPEX)** — **sem impressora 3D** e **sem PCB encomendada** (chassi híbrido madeira/alumínio + placa perfurada `7×9 cm` FR4).

## Documentação de Engenharia, Lista de Compras (BOM) e Montagem (`docs/`)

1. [**Validação de Engenharia FMEA & BOM Otimizada v2.0**](docs/validacao_engenharia_e_bom_otimizada.md) — Análise matemática/elétrica pré-compra (7 armadilhas de hardware mitigadas) + lista exata de peças e preços Brasil vs. AliExpress.
2. [**Manual de Montagem Passo a Passo Milimétrico**](docs/manual_montagem_detalhado_450.md) — Gabaritos de corte/furação (`120×120 mm`, braços `225 mm`), mapa ponto-a-ponto da placa `7×9 cm` FR4, construção do controle caseiro, **Bancada Gangorra 1-Eixo** e os **8 Portões de Validação**.
3. [**Plano Mestre de Arquitetura de Software**](docs/arquitetura_software_esp32s3.md) — Arquitetura Hexagonal, convenção estrita `FRD`/`NED`, tempo real Dual-Core *lock-free* (`SeqLock`), protocolo `ESP-NOW LR` + `MAVLink v2` e Caixa-Preta PSRAM.
4. [**Arquitetura Geral do Sistema**](docs/arquitetura_drone_esp32s3_450.md) & [**Pesquisa Panorâmica Inicial**](docs/pesquisa_drone_esp32.md).

---

## Estrutura do Código

- `lib/flight_core/` — Núcleo matemático e de controle determinístico em C++17 puro (`BiquadLPF`, `MahonyAHRS`, `VerticalEstimator` Accel-Z + Baro, `AltHoldController` c/ *flare* e detecção de pouso, `CascadedPID`, `QuadXMixer`, `SafetyStateMachine`, `GimbalMapper` e `FlightPipeline`).
- `lib/shared_protocol/` — Pacotes binários compactos ESP-NOW LR (`CRC-16-CCITT`) e tabela de **19 parâmetros MAVLink** ajustáveis em voo.
- `lib/mavlink_min/` — Implementação portátil *header-only* MAVLink v2 (`common.xml`, validada contra `pymavlink`).
- `lib/hal_sync/` — Primitivas *lock-free* (`SeqLock<T>`) entre Core 0 e Core 1.
- `lib/hal_esp32s3/` — Drivers de hardware ESP32-S3 (`ICM-42688-P` SPI 10 MHz + `INT1` 1 kHz, `BMP280` I2C 400 kHz, `LEDC` 14-bit @ 400 Hz, `ESP-NOW Long Range`, `Blackbox` PSRAM 6.5 MB e `BenchCli`).
- `lib/sim_physics/` — Dinâmica 6-DoF realista do quadricóptero de 900 g (motores A2212, sag de bateria 3S, vibração 185 Hz, BMP280).
- `src/sitl_pc/` — Simulador SITL com servidor MAVLink v2 UDP (`:14550`) para **QGroundControl**.
- `src/ground_tx/` — Firmware Dual-Core do Controle Caseiro (ADC1 16× oversampling, Wizard NVS, OLED SSD1306, Ponte USB-OTG MAVLink v2).
- `src/drone_fc/` — Firmware Dual-Core da Controladora de Voo (Core 1 @ 1000 Hz ISR `INT1` + Core 0 @ ESP-NOW LR / Baro / Blackbox PSRAM / CLI).

## Modos de Voo na Chave de 3 Posições (`SW2` — Fase 1)

1. **`STAB` (`STABILIZE`):** Autonivelamento `±35°` com *Throttle* 100% manual direto — ideal para **Portão 8 (Gangorra 1-Eixo)** e primeiros saltos rasantes.
2. **`ALTH` (`ALT_HOLD`):** Trava de altitude automática no centro do stick (`42%–58%`) fundindo `Earth Accel-Z` (`1000 Hz`) + `BMP280` (`40 Hz`), subida/descida proporcional (`±1.8 m/s`) e inclinação `±35°`.
3. **`CINE` (`ALT_CINE` Iniciante / Filmagem):** Trava de altitude suave (`±1.0 m/s`) com inclinação máxima reduzida a `±20°` e giro `Yaw` suave (`90°/s`). *(Slot já preparado no protocolo para receber `POS_HOLD` quando o GPS da Fase 2 for conectado ao header de 6 pinos).*
- **Failsafe Inteligente com Barômetro:** Perda de rádio `> 300 ms` nivela o drone, desce controlado a `-0.65 m/s`, faz *flare* macio a `-0.35 m/s` abaixo de `1.5 m` AGL e **desliga automaticamente os motores ao detectar toque firme na grama**, mantendo a sirene ativa.

## Comandos Principais (PlatformIO)

```bash
# 1. Rodar todas as 8 suítes de testes unitários + voo 6-DoF em malha fechada no PC
pio test -e native_test

# 2. Compilar e rodar demonstração Fase 1 no Simulador SITL 6-DoF (ALT_HOLD + Pouso Failsafe)
pio run -e sitl_pc && ./.pio/build/sitl_pc/program --headless-demo

# 3. Rodar SITL em tempo real aguardando conexão do QGroundControl (UDP 127.0.0.1:14550)
./.pio/build/sitl_pc/program

# 4. Validar protocolo MAVLink v2 (19 parâmetros) e analisador FFT da Blackbox
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
- `imu_test` / `imu_orient <0|90|180|270>` / `imu_cal` / `baro_tare` — Verificação de sinais FRD no **Portão 5** e calibração.
- `bench on` → `motor_test <1..4> <0..25>` — Teste individual de motores **sem hélices** no **Portão 6** (corta em 3 s ou ao pressionar qualquer tecla).
- `bench on` → `esc_cal` — Assistente guiado `2000 µs → 1000 µs` de calibração dos 4 ESCs (**sem hélices**).
- `bb_info` / `bb_dump_csv` / `bb_dump_bin` — Exportação da Caixa-Preta PSRAM de 6.5 MB para `python3 tools/analyze_blackbox.py`.
