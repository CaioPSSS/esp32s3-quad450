# ESP32-S3 Quad-450 DIY Flight Controller & Long-Range Ground Station (`v2.1` — Fase 1 Completa Sem GPS)

Monorepo completo para engenharia, simulação física 6-DoF, construção de bancada e voo real de um quadricóptero **450 mm DIY** e seu **Controle Remoto Caseiro Long-Range**, baseados em **2× `ESP32-S3-DevKitC-1 N16R8` (`WROOM-1U` c/ antena externa IPEX)** — **sem impressora 3D** e **sem PCB encomendada** (chassi híbrido em madeira `15×15 mm` ou alumínio `1/2"` + placa perfurada `70×90 mm` FR4).

## Suíte Unificada de Documentação (`docs/` — Revisão `v2.1`)

1. [**Validação de Engenharia Multidisciplinar, FMEA & BOM Definitiva v2.1**](docs/validacao_engenharia_e_bom_otimizada.md) — **Comece por aqui antes de comprar peças!** 14 armadilhas reais de engenharia mitigadas (geometria anti-colisão, reações de alavanca e ressonância modal dos braços, limite linear `<= 2,90 V` do `ADC1`, escolha da IMU `ICM-42688-P` SPI sem `BMI270`, buzzer `NPN Active-HIGH`, *link budget* Friis `250 kbps` de `1 km`), tabelas completas da **BOM v2.1** e checklist de inspeção ao receber os pacotes.
2. [**Manual de Montagem Milimétrico Passo a Passo v2.1**](docs/manual_montagem_detalhado_450.md) — Gabaritos exatos de furação (`120×120 mm` c/ dampers em `(±22, ±41) mm` sem colisão nos braços, braços `225 mm` c/ buchas internas), soldagem ponto-a-ponto (`39 kΩ / 10 kΩ` `VBAT`, topo `560 Ω + 10 µF` nos gimbals, `SN74HCT125N` 5V), **Bancada Gangorra 1-Eixo** e os **8 Portões de Validação** com critérios numéricos de multímetro.
3. [**Especificação Mestre de Arquitetura de Software v2.1**](docs/arquitetura_software_esp32s3.md) — Arquitetura Hexagonal C++17, isolamento estrito Dual-Core (`CommsCore0` / `UiCore0` no Core 0 e `Flight1kHz` / `RcTx100Hz` no Core 1), margens de estabilidade de Bode (`PM = 54,2°`, `GM = 14,3 dB`), proteções de pouso inclinado e falha de barômetro, protocolo `ESP-NOW LR` (`250 kbps`) e tabela dos **19 parâmetros MAVLink v2**.
4. [**Visão Geral Integrada do Sistema v2.1**](docs/arquitetura_drone_esp32s3_450.md) & [**Pesquisa Exploratória Inicial (Histórico Fase 0)**](docs/pesquisa_drone_esp32.md).

---

## Arquitetura do Código (`v2.1`)

- `lib/flight_core/` — Núcleo matemático e de controle em C++17 puro (`BiquadLPF`, `MahonyAHRS`, `VerticalEstimator` 3 estados, `AltHoldController` c/ *flare* e detecção de toque, `CascadedPID` c/ *bleed* de integrador, `QuadXMixer` c/ supressão de *Airmode* em pouso inclinado e PWM float, `SafetyStateMachine` c/ histerese de 3 pacotes, `GimbalMapper` e `FlightPipeline`).
- `lib/shared_protocol/` — Pacotes binários compactos `ESP-NOW LR` (`20B / 34B / 16B / 28B` + `CRC-16-CCITT`) e registro validado de **19 parâmetros MAVLink**.
- `lib/mavlink_min/` — Implementação portátil *header-only* **MAVLink v2** (`common.xml`, validada contra `pymavlink` e **QGroundControl**).
- `lib/hal_sync/` — Primitivas atômicas *lock-free* (`SeqLock<T>`) entre Core 0 e Core 1.
- `lib/hal_esp32s3/` — Drivers dedicados ESP32-S3 (`ICM-42688-P` SPI 10 MHz + `INT1` 1 kHz c/ fallback automático p/ `ICM-42605`/`MPU-6500`, `BMP280` I2C 400 kHz, `LEDC` 14-bit @ 400 Hz com resolução float de `0,1526 µs`, `ESP-NOW LR` puro @ `WIFI_PHY_RATE_LORA_250K` + RSSI filtrado por MAC, `Blackbox` PSRAM 6,5 MB e `BenchCli`).
- `lib/sim_physics/` & `src/sitl_pc/` — Dinâmica 6-DoF do Quad-450 + servidor MAVLink v2 UDP (`:14550`) para voar no **QGroundControl** antes de montar o hardware.
- `src/ground_tx/` & `src/drone_fc/` — Firmwares Dual-Core de produção do Controle Caseiro e da Controladora de Voo.

## Modos de Voo (`SW2`) e Segurança Soberana

1. **`STAB` (`STABILIZE`):** Autonivelamento `±35°` com *Throttle* manual direto — usado na **Gangorra 1-Eixo (Portão 8)** e primeiros voos.
2. **`ALTH` (`ALT_HOLD`):** Trava automática de altitude no centro do stick (`42%–58%`) fundindo `Earth Accel-Z` (`1000 Hz`) + `BMP280` (`40 Hz`), subida/descida proporcional (`±1,8 m/s`) e fallback *bumpless* automático caso o barômetro falhe em voo.
3. **`CINE` (`ALT_CINE` Iniciante / Filmagem):** Trava suave de altitude (`±1,0 m/s`), inclinação máxima reduzida a `±20°` e `Yaw` suave (`90°/s`) — slot já preparado para `POS_HOLD` na Fase 2 com GPS.
- **Failsafe Inteligente em 2 Estágios:** Perda de rádio `> 300 ms` nivela o chassi, desce controlado a `-0,65 m/s`, executa *flare* a `-0,35 m/s` abaixo de `1,5 m` AGL e **corta automaticamente os motores ao apoiar no solo**, exigindo 3 pacotes consecutivos para retomar voo ou apenas 1 pacote com `SW1 = SAFE` para corte imediato.

## Comandos Principais (PlatformIO)

```bash
# 1. Executar as 8 suítes de testes nativos (42 casos unitários + 6-DoF em malha fechada)
pio test -e native_test

# 2. Compilar e rodar missão completa no Simulador SITL 6-DoF (ALT_HOLD + Pouso Failsafe)
pio run -e sitl_pc && ./.pio/build/sitl_pc/program --headless-demo

# 3. Iniciar SITL em tempo real aguardando QGroundControl (UDP 127.0.0.1:14550)
./.pio/build/sitl_pc/program

# 4. Validar protocolo MAVLink v2 (19 parâmetros) e analisador FFT da Blackbox
python3 tools/verify_sitl_mavlink.py
python3 tools/analyze_blackbox.py --self-test

# 5. Compilar / Gravar Controle Caseiro (ground_tx) e Controladora de Voo (drone_fc)
pio run -e ground_tx -e drone_fc
# pio run -e ground_tx -t upload
# pio run -e drone_fc -t upload && pio device monitor -b 115200
```

## Console Interativo de Bancada (`drone_fc` USB-C @ `115200 bps` — Portões 5 a 8)

- `status` — Diagnóstico completo (`WHO_AM_I`, `INT1 Hz`, `dt_us`, atitude `FRD`, `BMP280`, `VBAT`, `RSSI`, `PreArmFlags`).
- `imu_test` / `imu_orient <0|90|180|270>` / `imu_cal` / `baro_tare` — Validação de eixos `FRD` no **Portão 5** e calibração.
- `bench on` → `motor_test <1..4> <0..25>` / `esc_cal` — Teste individual de motores (`3 s`, corta em qualquer tecla) e calibração de ESCs **sem hélices** no **Portão 6**.
- `params` / `set <NOME> <VAL>` / `save` / `nvs_reset` — Sintonia fina de ganhos e filtros (bloqueia gravação `NVS` caso armado).
- `bb_info` / `bb_dump_csv` / `bb_dump_bin` — Exportação da Caixa-Preta PSRAM (`6,5 MB`) para `python3 tools/analyze_blackbox.py`.
