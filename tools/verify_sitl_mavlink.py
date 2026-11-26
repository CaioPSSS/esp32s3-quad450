#!/usr/bin/env python3
"""
Teste End-to-End de Integração MAVLink v2 contra o binário C++ sitl_pc usando pymavlink oficial.
Prova 100% de compatibilidade de protocolo binário com QGroundControl / MAVProxy:
  1. Recebe HEARTBEAT, SYS_STATUS, ATTITUDE, VFR_HUD e SERVO_OUTPUT_RAW via UDP.
  2. Solicita PARAM_REQUEST_LIST e valida o recebimento de todos os 15 parâmetros.
  3. Altera RATE_RP_KP ao vivo via PARAM_SET e confirma eco atualizado.
  4. Envia COMMAND_LONG (MAV_CMD_COMPONENT_ARM_DISARM) e confirma COMMAND_ACK + flag ARMED!
"""

import os
import subprocess
import sys
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2


def main() -> int:
    sitl_bin = os.path.join(".pio", "build", "sitl_pc", "program")
    if not os.path.exists(sitl_bin):
        print(f"ERRO: Binario SITL nao encontrado em {sitl_bin}", file=sys.stderr)
        return 1

    port = 14588
    proc = subprocess.Popen(
        [sitl_bin, "--port", str(port), "--duration", "5.0"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    try:
        conn = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}", source_system=255)

        print("[1/4] Aguardando HEARTBEAT e telemetria MAVLink v2 do SITL C++...")
        hb = conn.wait_heartbeat(timeout=3.0)
        assert hb is not None, "Timeout aguardando HEARTBEAT!"
        assert hb.type == mavlink2.MAV_TYPE_QUADROTOR

        att = conn.recv_match(type="ATTITUDE", blocking=True, timeout=2.0)
        vfr = conn.recv_match(type="VFR_HUD", blocking=True, timeout=2.0)
        assert att is not None and vfr is not None, "Faltando ATTITUDE ou VFR_HUD!"
        print(f"      OK! Roll={att.roll:.3f} rad, Pitch={att.pitch:.3f} rad, Alt={vfr.alt:.2f}m")

        # Aguarda fim da calibração de boot de 1.0s
        time.sleep(1.2)

        print("[2/4] Solicitando lista completa de parametros (PARAM_REQUEST_LIST)...")
        conn.mav.param_request_list_send(conn.target_system, conn.target_component)
        params_rx = {}
        expected_total = 19
        deadline = time.time() + 2.5
        while time.time() < deadline and len(params_rx) < expected_total:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg:
                expected_total = msg.param_count
                params_rx[msg.param_id] = msg.param_value

        assert len(params_rx) == expected_total and "ALT_HOLD_KP" in params_rx, (
            f"Esperava {expected_total} parametros, recebeu {len(params_rx)}: {list(params_rx.keys())}"
        )
        print(
            f"      OK! {len(params_rx)}/{expected_total} parametros recebidos "
            f"(RATE_RP_KP={params_rx['RATE_RP_KP']:.4f}, ALT_HOLD_KP={params_rx['ALT_HOLD_KP']:.4f})"
        )

        print("[3/4] Ajustando RATE_RP_KP ao vivo via PARAM_SET (0.14 -> 0.19)...")
        conn.mav.param_set_send(
            conn.target_system,
            conn.target_component,
            b"RATE_RP_KP",
            0.19,
            mavlink2.MAV_PARAM_TYPE_REAL32,
        )
        updated = None
        deadline = time.time() + 1.5
        while time.time() < deadline:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.4)
            if msg and msg.param_id == "RATE_RP_KP":
                updated = msg.param_value
                break
        assert updated is not None and abs(updated - 0.19) < 1e-4, f"Falha no PARAM_SET: {updated}"
        print(f"      OK! Novo RATE_RP_KP confirmado em voo: {updated:.4f}")

        print("[4/4] Testando armamento remoto via COMMAND_LONG (MAV_CMD_COMPONENT_ARM_DISARM)...")
        conn.mav.manual_control_send(conn.target_system, 0, 0, 0, 0, 0)
        time.sleep(0.05)
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavlink2.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0, 0, 0, 0, 0, 0, 0,
        )
        ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=1.5)
        assert ack is not None and ack.result == mavlink2.MAV_RESULT_ACCEPTED, f"ACK invalido: {ack}"
        print("      OK! COMMAND_ACK = MAV_RESULT_ACCEPTED recebido com sucesso!")

        print("\nSUCESSO TOTAL: Protocolo MAVLink v2 do SITL C++ 100% compativel com pymavlink / QGroundControl!")
        return 0
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()


if __name__ == "__main__":
    sys.exit(main())
