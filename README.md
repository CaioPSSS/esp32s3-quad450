# ESP32-S3 Quad-450 DIY Flight Controller & Ground Station

Monorepo completo (Flight Core C++17 puro + Simulador SITL 6-DoF + Controladora de Voo ESP32-S3 + Controle Caseiro ESP-NOW LR / MAVLink v2).

## Estrutura do Projeto

- `lib/flight_core/` — Núcleo matemático e de controle determinístico em C++17 puro (zero dependência de hardware).
- `lib/shared_protocol/` — Pacotes binários compactos ESP-NOW (`CRC-16-CCITT`) e tabela de parâmetros MAVLink.
- `lib/hal_sync/` — Primitivas *lock-free* (`SeqLock<T>`) para sincronização entre Core 0 e Core 1.
- `src/drone_fc/` — Firmware da controladora de voo (`ESP32-S3-DevKitC-1 N16R8`).
- `src/ground_tx/` — Firmware do controle caseiro c/ gimbals RC, OLED e ponte USB-OTG MAVLink v2.
- `src/sitl_pc/` — Simulador físico 6-DoF no PC integrado ao QGroundControl via UDP `:14550`.
- `test/` — Suíte de testes unitários TDD executada nativamente no PC.

## Comandos Úteis

```bash
# Rodar toda a suíte de testes unitários no PC (< 1s)
pio test -e native_test

# Rodar apenas os testes da Sprint 1 (Filtro Biquad)
pio test -e native_test -f test_biquad
```
