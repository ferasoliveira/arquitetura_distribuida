# -*- coding: utf-8 -*-
"""
benchmark.py — Adaptador Geral de Benchmark (CLI) para o EB15
============================================================
TCC Fernando Oliveira — Passo 3: Ambiente Virtual, Pontes e Simulação
"""

import os
import sys
import time
import argparse
import subprocess
import socket
import urllib.request
import json
import csv
import logging
import math
from typing import List, Dict, Any, Optional

# ─────────────────────────────────────────────────────────────────
# Exit codes distintos por categoria de falha (Fase 5 do plano)
# ─────────────────────────────────────────────────────────────────
EXIT_OK         = 0   # Execução concluída sem erros
EXIT_DEPENDENCY = 1   # Porta em uso, URSim inacessível ou dependência ausente
EXIT_TIMEOUT    = 2   # Timeout de conexão (ESP32 QEMU, URSim, Webots)
EXIT_TRAJECTORY = 3   # Falha durante execução de trajetória
EXIT_OUTPUT     = 4   # Falha ao gravar CSV ou manifesto JSON
EXIT_CONFIG     = 5   # Argumento inválido ou config.yaml corrompido

# Configura logs
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger("benchmark")

# Importa módulos internos
import trajectory
from rtde_eb15 import RtdeEb15Client
from rtde_ursim import RtdeUrsimClient

# --- Configurações Fixas ---
CONFIG_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config.yaml"))
WORKSPACE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

# Configurações de Fallback caso o config.yaml falhe em carregar
DEFAULT_CONFIG = {
    "network": {
        "host": "127.0.0.1",
        "rtde_eb15_port": 30003,
        "arduino_sim_port": 30100,
        "stepdir_esp_port": 30101,
        "stepdir_uno_port": 30102,
        "encoder_esp_port": 30103,
        "encoder_uno_port": 30104,
        "http_user_port": 8080,
        "ursim_host": "127.0.0.1",
        "ursim_rtde_port": 30004,
        "ursim_script_port": 31003,
        "ursim_dashboard_port": 29999
    },
    "webots": {
        "world_file": "worlds/eb15_hil.wbt",
        "headless": True,
        "mode": "fast"
    },
    "simulation": {
        "seed": 42,
        "noise_deg": 0.0
    },
    "paths": {
        "output_csv": "output",
        "logs": "output/logs"
    }
}

def load_config() -> dict:
    """Carrega as configurações do config.yaml de forma robusta."""
    try:
        import yaml
        if os.path.exists(CONFIG_PATH):
            with open(CONFIG_PATH, "r", encoding="utf-8") as f:
                return yaml.safe_load(f)
    except Exception as e:
        logger.warning("Erro ao carregar PyYAML ou config.yaml (%s). Usando configurações padrão.", e)
    return DEFAULT_CONFIG

config = load_config()

def eb15_fk_tcp(q_deg: List[float]) -> List[float]:
    """Cinemática direta posicional idêntica à implementada no firmware."""
    q = [math.radians(v) for v in q_deg]
    q23 = q[1] + q[2]
    radial = 200.0 * math.cos(q[1]) + 200.0 * math.cos(q23)
    wc = [math.cos(q[0]) * radial,
          math.sin(q[0]) * radial,
          150.0 + 200.0 * math.sin(q[1]) + 200.0 * math.sin(q23)]

    def matmul(a, b):
        return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
                for i in range(3)]
    def rz(v):
        return [[math.cos(v), -math.sin(v), 0.0],
                [math.sin(v), math.cos(v), 0.0], [0.0, 0.0, 1.0]]
    def ry(v):
        return [[math.cos(v), 0.0, math.sin(v)], [0.0, 1.0, 0.0],
                [-math.sin(v), 0.0, math.cos(v)]]
    rotation = matmul(matmul(rz(q[0]), ry(q23)),
                      matmul(matmul(rz(q[3]), ry(q[4])), rz(q[5])))
    return [wc[i] + 80.0 * rotation[i][2] for i in range(3)] + [0.0, 0.0, 0.0]

def is_port_in_use(port: int) -> bool:
    """Verifica se uma porta TCP está em uso no localhost."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.1)
        return s.connect_ex(("127.0.0.1", port)) == 0


def wait_for_port(port: int, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if is_port_in_use(port):
            return True
        time.sleep(0.1)
    return False

def check_dependencies(target: str):
    """Verifica se as portas necessárias estão livres ou se os serviços estão ativos."""
    logger.info("Verificando dependências de rede...")

    eb15_ports = [
        config["network"]["rtde_eb15_port"],
        config["network"]["stepdir_esp_port"],
        config["network"]["stepdir_uno_port"],
        config["network"]["encoder_esp_port"],
        config["network"]["encoder_uno_port"],
        config["network"]["http_user_port"]
    ]

    if target in ["eb15", "both"]:
        for p in eb15_ports:
            if is_port_in_use(p):
                logger.error(
                    "Porta %d já está em uso. Encerre qualquer emulador QEMU ou processo anterior.", p
                )
                sys.exit(EXIT_DEPENDENCY)

    if target in ["ursim", "both"]:
        p = config["network"]["ursim_rtde_port"]
        if not is_port_in_use(p):
            logger.error(
                "URSim não responde na porta %d. Inicialize o URSim antes de executar.", p
            )
            sys.exit(EXIT_DEPENDENCY)

    logger.info("Health check de dependências concluido. OK")

def set_esp32_rtde_mode(http_port: Optional[int] = None) -> bool:
    """Chama a API HTTP do ESP32 para ativar o modo RTDE."""
    port = http_port or config["network"]["http_user_port"]
    url = f"http://127.0.0.1:{port}/mode"
    data = b"rtde"

    logger.info("Ativando modo RTDE no ESP32-S3 via API HTTP (porta %d)...", port)
    for attempt in range(5):
        try:
            req = urllib.request.Request(
                url, data=data,
                headers={"Content-Type": "text/plain"},
                method="POST"
            )
            with urllib.request.urlopen(req, timeout=2.0) as r:
                res = r.read().decode().strip()
                if res == "RTDE":
                    logger.info("Modo RTDE ativado com sucesso no ESP32-S3. OK")
                    return True
        except Exception:
            time.sleep(1.0)

    logger.error("Falha ao ativar o modo RTDE no ESP32-S3 (timeout apos 5 tentativas).")
    return False

def save_csv(filename: str, history: List[Dict[str, Any]], source: str):
    """Grava o histórico de telemetria coletado em um arquivo CSV formatado."""
    output_dir = os.path.join(WORKSPACE_DIR, "webots", config["paths"]["output_csv"])
    os.makedirs(output_dir, exist_ok=True)
    filepath = os.path.join(output_dir, filename)

    columns = config.get("csv", {}).get("columns", [
        "timestamp", "source", "sequence",
        "q1", "q2", "q3", "q4", "q5", "q6",
        "q_ref1", "q_ref2", "q_ref3", "q_ref4", "q_ref5", "q_ref6",
        "qd1", "qd2", "qd3", "qd4", "qd5", "qd6",
        "qdd1", "qdd2", "qdd3", "qdd4", "qdd5", "qdd6",
        "tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz",
        "tcp_ref_x", "tcp_ref_y", "tcp_ref_z",
        "state", "error_code", "trigger"
    ])

    logger.info("Gravando %d amostras de telemetria em %s...", len(history), filepath)

    f = None
    for attempt in range(5):
        try:
            f = open(filepath, "w", newline="", encoding="utf-8")
            break
        except OSError as e:
            logger.warning("Tentativa %d de abrir CSV '%s' falhou (%s). Retentando em 1.0s...", attempt + 1, filepath, e)
            time.sleep(1.0)
    else:
        logger.error("Falha ao abrir CSV '%s' apos 5 tentativas.", filepath)
        sys.exit(EXIT_OUTPUT)

    try:
        with f:
            writer = csv.writer(f)
            writer.writerow(columns)

            for i, tel in enumerate(history):
                row = []
                qd = [0.0] * 6
                qdd = [0.0] * 6
                if i > 0:
                    dt = tel["timestamp"] - history[i - 1]["timestamp"]
                    if dt > 0.001:
                        qd = [(tel["joints"][j] - history[i - 1]["joints"][j]) / dt for j in range(6)]
                if i > 1:
                    dt = tel["timestamp"] - history[i - 1]["timestamp"]
                    if dt > 0.001:
                        qdd = [
                            (qd[j] - ((history[i - 1]["joints"][j] - history[i - 2]["joints"][j]) / dt)) / dt
                            for j in range(6)
                        ]

                for col in columns:
                    if col == "timestamp":
                        row.append(f"{tel['timestamp']:.6f}")
                    elif col == "source":
                        row.append(source)
                    elif col == "sequence":
                        row.append(tel.get("sequence", 0))
                    elif col.startswith("q_ref"):
                        idx = int(col[5:]) - 1
                        row.append(f"{tel.get('target_joints', [0.0] * 6)[idx]:.6f}")
                    elif col.startswith("q") and not col.startswith("qd") and not col.startswith("qdd"):
                        idx = int(col[1:]) - 1
                        row.append(f"{tel['joints'][idx]:.6f}")
                    elif col.startswith("qd") and not col.startswith("qdd"):
                        idx = int(col[2:]) - 1
                        row.append(f"{qd[idx]:.6f}")
                    elif col.startswith("qdd"):
                        idx = int(col[3:]) - 1
                        row.append(f"{qdd[idx]:.6f}")
                    elif col == "tcp_x":
                        row.append(f"{tel.get('tcp', [0]*6)[0]:.3f}")
                    elif col == "tcp_y":
                        row.append(f"{tel.get('tcp', [0]*6)[1]:.3f}")
                    elif col == "tcp_z":
                        row.append(f"{tel.get('tcp', [0]*6)[2]:.3f}")
                    elif col == "tcp_rx":
                        row.append(f"{tel.get('tcp', [0]*6)[3]:.3f}")
                    elif col == "tcp_ry":
                        row.append(f"{tel.get('tcp', [0]*6)[4]:.3f}")
                    elif col == "tcp_rz":
                        row.append(f"{tel.get('tcp', [0]*6)[5]:.3f}")
                    elif col == "tcp_ref_x":
                        row.append(f"{tel.get('tcp_ref', [0]*3)[0]:.3f}")
                    elif col == "tcp_ref_y":
                        row.append(f"{tel.get('tcp_ref', [0]*3)[1]:.3f}")
                    elif col == "tcp_ref_z":
                        row.append(f"{tel.get('tcp_ref', [0]*3)[2]:.3f}")
                    elif col == "state":
                        row.append(tel.get("state", "MOVING"))
                    elif col == "error_code":
                        errs = tel.get("errors", [0.0] * 6)
                        avg_err = sum(abs(e) for e in errs) / len(errs)
                        row.append(f"{avg_err:.6f}")
                    elif col == "trigger":
                        row.append(tel.get("trigger", 0))
                    else:
                        row.append("0")
                writer.writerow(row)
    except OSError as e:
        logger.error("Falha ao gravar CSV '%s': %s", filepath, e)
        sys.exit(EXIT_OUTPUT)

def save_manifest(args, eb15_history: list, ursim_history: list,
                  t_start: float, t_end: float, status: str):
    """Grava o manifesto JSON da execução (Fase 5 do plano_passo3)."""
    output_dir = os.path.join(WORKSPACE_DIR, "webots", config["paths"]["output_csv"])
    os.makedirs(output_dir, exist_ok=True)
    path = os.path.join(output_dir, f"manifest_{args.trajectory}_{args.target}.json")

    seed_used = args.seed if args.seed is not None else config.get("simulation", {}).get("seed", 42)

    manifest = {
        "schema_version": "1.0",
        "timestamp_start": round(t_start, 3),
        "timestamp_end": round(t_end, 3),
        "duration_s": round(t_end - t_start, 3),
        "target": args.target,
        "trajectory": args.trajectory,
        "seed": seed_used,
        "headless": args.headless,
        "status": status,
        "eb15": {
            "samples": len(eb15_history),
            "csv": f"benchmark_{args.trajectory}_eb15.csv" if eb15_history else None
        },
        "ursim": {
            "samples": len(ursim_history),
            "csv": f"benchmark_{args.trajectory}_ursim.csv" if ursim_history else None
        }
    }

    f = None
    for attempt in range(5):
        try:
            f = open(path, "w", encoding="utf-8")
            break
        except OSError as e:
            logger.warning("Tentativa %d de abrir manifesto '%s' falhou (%s). Retentando em 1.0s...", attempt + 1, path, e)
            time.sleep(1.0)
    else:
        logger.error("Falha ao abrir manifesto JSON '%s' apos 5 tentativas.", path)
        sys.exit(EXIT_OUTPUT)

    try:
        with f:
            json.dump(manifest, f, indent=2, ensure_ascii=False)
        logger.info("Manifesto JSON salvo em: %s", path)
    except OSError as e:
        logger.error("Falha ao gravar manifesto JSON '%s': %s", path, e)
        sys.exit(EXIT_OUTPUT)

def run_trajectory_eb15(traj: dict, prebind_sock=None) -> List[Dict[str, Any]]:
    """Executa a trajetória e mede somente depois de a pose estabilizar.

    Settle: variação máxima menor que 0,25 grau em seis leituras espaçadas
    por 0,5 s. O critério é de estabilidade, não de proximidade ao alvo:
    assim, uma divergência numérica do QEMU é preservada como resultado.
    """
    client = RtdeEb15Client(
        port=config["network"]["rtde_eb15_port"],
        server_mode=False  # QEMU: Python é cliente, ESP32 firmware é servidor
    )

    if not client.connect(timeout=90.0):
        logger.error("ESP32 QEMU não respondeu ao cliente RTDE-EB15 (timeout).")
        sys.exit(EXIT_TIMEOUT)

    set_esp32_rtde_mode()

    logger.info("Aguardando telemetria inicial do ESP32...")
    t_wait = time.time()
    while not client.get_latest_telemetry():
        if time.time() - t_wait > 30.0:
            logger.error("Timeout aguardando telemetria inicial do ESP32.")
            client.disconnect()
            sys.exit(EXIT_TIMEOUT)
        time.sleep(0.1)

    logger.info("Iniciando execucao da trajetoria no EB15/Webots...")
    all_telemetry = []
    speed_pct = traj.get("speed_pct", 30.0)

    for wp in traj["waypoints"]:
        seq = wp["seq"]
        target = wp["q"]
        duration_s = wp["duration_ms"] / 1000.0

        logger.info("Enviando WP %d/%d: target=%s, dur=%.2fs",
                    seq + 1, len(traj["waypoints"]), target, duration_s)

        if not client.send_movej(target, speed_pct):
            logger.error("Falha ao enviar waypoint %d — conexao RTDE perdida.", seq + 1)
            client.disconnect()
            sys.exit(EXIT_TRAJECTORY)

        start_time = time.time()
        timeout = max(30.0, duration_s + 30.0)
        settle_tol = 0.25
        # Convergência: o assentamento exige ESTABILIDADE *e* PROXIMIDADE ao alvo.
        # O critério antigo (só estabilidade) deixava J4-J6 (feedforward, despacho
        # UART com latência variável no QEMU) ser amostrado "parado" na pose antiga
        # antes de o seu movimento ocorrer — um falso assentamento. Exigir |q-alvo| <
        # conv_tol dá tempo ao despacho lento chegar e mede convergência REAL; se não
        # convergir até o timeout, marca-se SETTLE_TIMEOUT (falha honesta, não oculta).
        conv_tol = 3.0
        stable_needed = 6
        stable_count = 0
        last_stable_q = None
        # Começa a checar estabilidade após um curto pré-rolo (no máx. 2 s), não após a
        # duração estimada inteira: o firmware planeja a 100 % e conclui o movimento em
        # ~1,3 s, então esperar os ~5 s da estimativa só desperdiçava tempo por waypoint.
        # A própria exigência de convergência (|q-alvo| < conv_tol) impede falso settle
        # antes de o movimento ocorrer.
        next_stable_check = start_time + max(0.5, min(duration_s, 2.0))

        def _frame_zero(q, ref):
            # Artefato de CAPTAÇÃO do transporte do QEMU: J1-J3 lendo ~0 ao mesmo tempo
            # enquanto o alvo é não-trivial. Ignorar essas amostras na decisão de settle
            # evita resets espúrios do contador de estabilidade (não mascara falha real:
            # o critério de convergência continua valendo nas amostras válidas).
            return (abs(q[0]) < 2.0 and abs(q[1]) < 2.0 and abs(q[2]) < 2.0 and
                    (abs(ref[0]) > 3.0 or abs(ref[1]) > 3.0 or abs(ref[2]) > 3.0))
        settled = False
        diffs = []

        while True:
            time.sleep(0.02)
            tel = client.get_latest_telemetry()
            if tel:
                tel["sequence"] = seq
                tel["target_joints"] = list(target)
                tel["tcp_ref"] = eb15_fk_tcp(target)
                if not tel.get("tcp"):
                    tel["tcp"] = eb15_fk_tcp(tel["joints"])
                tel["state"] = "MOVING"
                tel["trigger"] = 1 if (time.time() - start_time <= duration_s) else 0
                all_telemetry.append(tel)

                diffs = [abs(tel["joints"][j] - target[j]) for j in range(6)]
                now = time.time()
                if now >= next_stable_check and not _frame_zero(tel["joints"], target):
                    q_now = list(tel["joints"])
                    if (last_stable_q is not None and
                            max(abs(q_now[j] - last_stable_q[j]) for j in range(6)) < settle_tol):
                        stable_count += 1
                    else:
                        stable_count = 0
                    last_stable_q = q_now
                    next_stable_check = now + 0.5
                    if stable_count >= stable_needed and max(diffs) < conv_tol:
                        tel["state"] = "SETTLED"
                        tel["trigger"] = 0
                        settled = True
                        logger.info("WP %d assentado (%d leituras; erro max=%.2f deg).",
                                    seq + 1, stable_needed, max(diffs))
                        break

            if time.time() - start_time > timeout:
                max_err = max(diffs) if diffs else float("inf")
                if all_telemetry:
                    all_telemetry[-1]["state"] = "SETTLE_TIMEOUT"
                    all_telemetry[-1]["trigger"] = 0
                logger.warning("WP %d sem settle em %.1fs (erro max=%.2f deg).",
                               seq + 1, timeout, max_err)
                break

    # Preserve SETTLED/SETTLE_TIMEOUT: essas marcas são a evidência usada
    # pela análise de convergência. DONE não deve sobrescrevê-las.
    for tel in all_telemetry[-5:]:
        tel["trigger"] = 0

    client.disconnect()
    return all_telemetry

def run_trajectory_ursim(traj: dict) -> List[Dict[str, Any]]:
    """Executa no URSim usando o mesmo critério de settle do EB15."""
    client = RtdeUrsimClient(
        host=config["network"]["ursim_host"],
        rtde_port=config["network"]["ursim_rtde_port"],
        # Script port do URSim mapeado para evitar colisão com o RTDE-EB15 (host 30003).
        script_port=config["network"].get("ursim_script_port", 30003)
    )
    if not client.connect():
        logger.error("Nao foi possivel conectar ao URSim.")
        sys.exit(EXIT_TIMEOUT)

    logger.info("Aguardando telemetria inicial do URSim...")
    t_wait = time.time()
    while not client.get_latest_telemetry():
        if time.time() - t_wait > 15.0:
            logger.error("Timeout aguardando telemetria inicial do URSim.")
            client.disconnect()
            sys.exit(EXIT_TIMEOUT)
        time.sleep(0.1)

    logger.info("Iniciando execucao da trajetoria no URSim...")
    all_telemetry = []
    speed_pct = traj.get("speed_pct", 30.0)

    for wp in traj["waypoints"]:
        seq = wp["seq"]
        target = wp["q"]
        duration_s = wp["duration_ms"] / 1000.0

        logger.info("Enviando WP %d/%d ao URSim: target=%s, dur=%.2fs",
                    seq + 1, len(traj["waypoints"]), target, duration_s)

        if not client.send_movej(target, speed_pct):
            logger.error("Falha ao enviar waypoint %d ao URSim.", seq + 1)
            client.disconnect()
            sys.exit(EXIT_TRAJECTORY)

        start_time = time.time()
        timeout = max(30.0, duration_s + 30.0)
        settle_tol = 0.25
        stable_needed = 6
        stable_count = 0
        last_stable_q = None
        next_stable_check = start_time + max(0.5, duration_s)
        diffs = []

        while True:
            time.sleep(0.02)
            tel = client.get_latest_telemetry()
            if tel:
                tel["sequence"] = seq
                tel["target_joints"] = list(target)
                tel["tcp_ref"] = eb15_fk_tcp(target)
                tel["state"] = "MOVING"
                tel["trigger"] = 1 if (time.time() - start_time <= duration_s) else 0
                all_telemetry.append(tel)

                diffs = [abs(tel["joints"][j] - target[j]) for j in range(6)]
                now = time.time()
                if now >= next_stable_check:
                    q_now = list(tel["joints"])
                    if (last_stable_q is not None and
                            max(abs(q_now[j] - last_stable_q[j]) for j in range(6)) < settle_tol):
                        stable_count += 1
                    else:
                        stable_count = 0
                    last_stable_q = q_now
                    next_stable_check = now + 0.5
                    if stable_count >= stable_needed:
                        tel["state"] = "SETTLED"
                        tel["trigger"] = 0
                        logger.info("URSim WP %d assentado (%d leituras).", seq + 1, stable_needed)
                        break

            if time.time() - start_time > timeout:
                if all_telemetry:
                    all_telemetry[-1]["state"] = "SETTLE_TIMEOUT"
                    all_telemetry[-1]["trigger"] = 0
                logger.warning("URSim WP %d sem settle em %.1fs.", seq + 1, timeout)
                break

    for tel in all_telemetry[-5:]:
        tel["trigger"] = 0

    client.disconnect()
    return all_telemetry

def main():
    rtde_server_prebind = None
    parser = argparse.ArgumentParser(
        description="Adaptador Geral de Benchmark EB15",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--target", choices=["eb15", "ursim", "both"], default="eb15",
                        help="Destino do benchmark")
    parser.add_argument("--trajectory", choices=list(trajectory.TRAJECTORIES.keys()), default="canonical",
                        help="Nome da trajetoria canonica")
    parser.add_argument("--headless", action="store_true", default=True,
                        help="Executa Webots no modo headless")
    parser.add_argument("--no-headless", action="store_false", dest="headless",
                        help="Executa Webots com interface grafica")
    parser.add_argument("--seed", type=int, default=None,
                        help="Seed para ruido de simulacao (sobrescreve config.yaml)")

    args = parser.parse_args()

    # Aplica seed ao config se fornecido via CLI
    if args.seed is not None:
        if "simulation" not in config:
            config["simulation"] = {}
        config["simulation"]["seed"] = args.seed
        logger.info("Seed de simulacao sobrescrito via CLI: %d", args.seed)

    logger.info("=== EB15 General Benchmark Adaptor | target=%s | traj=%s | seed=%s ===",
                args.target, args.trajectory, args.seed)

    t_start = time.time()
    check_dependencies(args.target)
    traj = trajectory.load_trajectory(args.trajectory)

    processes = []
    log_handles = []
    eb15_history: list = []
    ursim_history: list = []
    final_status = "error"
    simavr_container = "eb15_simavr_serve"
    simavr_started = False

    try:
        if args.target in ["eb15", "both"]:
            logger.info("Inicializando arquitetura canonica: QEMU + simavr + 3 pontes + Webots")
            logs_dir = os.path.join(WORKSPACE_DIR, "webots", config["paths"]["logs"])
            os.makedirs(logs_dir, exist_ok=True)

            # 1. Pontes 2 e 3 independentes (relés elétricos)
            for script, log_name in (("ponte2_webots.py", "ponte2.log"),
                                     ("ponte3_webots.py", "ponte3.log")):
                bridge_log = open(os.path.join(logs_dir, log_name), "w", encoding="utf-8")
                log_handles.append(bridge_log)
                processes.append(subprocess.Popen(
                    [sys.executable, os.path.join(WORKSPACE_DIR, "webots", script)],
                    stdout=bridge_log, stderr=subprocess.STDOUT))

            # 2. Webots real; o controlador fala apenas com as pontes em 30301/30302
            import shutil
            import tempfile
            webots_stage = os.path.join(tempfile.gettempdir(), "eb15_webots_runtime")
            if os.path.isdir(webots_stage):
                shutil.rmtree(webots_stage)
            os.makedirs(os.path.join(webots_stage, "worlds"), exist_ok=True)
            os.makedirs(os.path.join(webots_stage, "controllers"), exist_ok=True)
            shutil.copytree(os.path.join(WORKSPACE_DIR, "webots", "protos"),
                            os.path.join(webots_stage, "protos"))
            shutil.copytree(os.path.join(WORKSPACE_DIR, "webots", "controllers", "hil_bridge"),
                            os.path.join(webots_stage, "controllers", "hil_bridge"))
            runtime_ini = os.path.join(webots_stage, "controllers", "hil_bridge", "runtime.ini")
            python_path = sys.executable.replace("\\", "/")
            with open(runtime_ini, "w", encoding="utf-8") as f:
                f.write(f"[python]\nCOMMAND = {python_path}\n")
            shutil.copy2(os.path.join(WORKSPACE_DIR, "webots", config["webots"]["world_file"]),
                         os.path.join(webots_stage, "worlds", "eb15_hil.wbt"))
            world_file = os.path.join(webots_stage, "worlds", "eb15_hil.wbt")
            webots_exe = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
            if not os.path.isfile(webots_exe):
                webots_exe = shutil.which("webots") or "webots"
            webots_cmd = [webots_exe]
            # Headless no Windows: --mode=fast (realtime trava o robot.step sem
            # display) + --minimize (janela Win32 com message pump) + --no-rendering.
            # NAO usar QT_QPA_PLATFORM=offscreen no Windows (quebra o Webots).
            webots_cmd += ["--batch", "--mode=fast", "--minimize", "--no-rendering", "--stdout", "--stderr"] if args.headless else ["--mode=realtime"]
            webots_cmd.append(world_file)
            webots_log = open(os.path.join(logs_dir, "webots.log"), "w", encoding="utf-8")
            log_handles.append(webots_log)
            env_webots = os.environ.copy()
            python_dir = os.path.dirname(sys.executable)
            env_webots["PATH"] = f"{python_dir};{env_webots.get('PATH', '')}"
            p_webots = subprocess.Popen(webots_cmd, stdout=webots_log, stderr=subprocess.STDOUT,
                                        env=env_webots, stdin=subprocess.DEVNULL)
            processes.append(p_webots)
            time.sleep(2.0)

            # 3. Runner simavr compilado do fonte atual e container fresco por run
            sim_stage = os.path.join(tempfile.gettempdir(), "eb15_simavr_runtime")
            os.makedirs(sim_stage, exist_ok=True)
            shutil.copy2(os.path.join(WORKSPACE_DIR, "firmware", "simavr", "uno_runner.c"), sim_stage)
            shutil.copy2(os.path.join(WORKSPACE_DIR, "firmware", "arduino_escravo", "build",
                                      "arduino_escravo.ino.elf"), sim_stage)
            subprocess.run(["docker", "rm", "-f", simavr_container], capture_output=True)
            subprocess.run([
                "docker", "run", "--rm", "-v", f"{sim_stage}:/work", "eb15-simavr:latest",
                "bash", "-c", "cd /work && gcc -O2 -Wall -Werror -o uno_runner uno_runner.c "
                "-lsimavr -lsimavrparts -lpthread -lelf"
            ], check=True)
            # Pacing tempo-real do simavr (item 3): ritma o AVR ao relogio de parede para
            # que os passos do DDA de J4-J6 cheguem AO LONGO do movimento (a telemetria
            # captura a rampa S-curve em vez de uma rajada=degrau). 1.0=tempo real;
            # >1.0=mais lento. Configuravel por EB15_SIMAVR_PACE; default 1.0.
            sim_pace = os.environ.get("EB15_SIMAVR_PACE", "1.0")
            subprocess.run([
                "docker", "run", "-d", "--rm", "--name", simavr_container,
                "-p", "30200:30200", "-p", "30201:30201", "-p", "30202:30202", "-p", "30203:30203",
                "-v", f"{sim_stage}:/work", "eb15-simavr:latest", "bash", "-c",
                f"cd /work && ./uno_runner --elf arduino_escravo.ino.elf --serve --pace {sim_pace}"
            ], check=True)
            simavr_started = True
            time.sleep(1.0)

            # 4. QEMU ESP32-S3 real
            qemu_firmware = os.path.join(
                WORKSPACE_DIR, "firmware", "esp32_mestre", ".build", "eb15_mestre.merged.bin"
            )
            if not os.path.isfile(qemu_firmware):
                logger.error(
                    "Firmware QEMU não encontrado: %s\n"
                    "Execute Códigos/firmware/qemu/build_idf.ps1 primeiro.", qemu_firmware
                )
                sys.exit(EXIT_DEPENDENCY)

            qemu_temp_path = os.path.join(tempfile.gettempdir(), "esp32_mestre_merged_temp.bin")
            try:
                shutil.copy2(qemu_firmware, qemu_temp_path)
                qemu_firmware = qemu_temp_path
                logger.info("Firmware copiado para caminho ASCII temporario: %s", qemu_firmware)
            except Exception as e:
                logger.warning("Nao foi possivel copiar firmware para caminho temporario ASCII: %s. Utilizando caminho original.", e)

            serial_log = os.path.join(tempfile.gettempdir(), "eb15_qemu_serial.log")
            logger.info("Iniciando QEMU ESP32-S3: %s", qemu_firmware)
            rtde_port  = config["network"]["rtde_eb15_port"]
            http_port  = config["network"]["http_user_port"]
            enc_port   = config["network"]["encoder_esp_port"]
            qemu_exe = os.environ.get("QEMU_ESP_PATH")
            if not qemu_exe:
                candidates = [
                    os.path.join(os.environ.get("USERPROFILE", ""), ".espressif", "tools", "qemu-xtensa", "esp_develop_9.2.2_20250817", "qemu", "bin", "qemu-system-xtensa.exe"),
                    "C:\\tools\\qemu-esp\\bin\\qemu-system-xtensa.exe"
                ]
                for c in candidates:
                    if os.path.isfile(c):
                        qemu_exe = c
                        break
            if not qemu_exe:
                qemu_exe = "qemu-system-xtensa"

            # stdin=DEVNULL é CRÍTICO: o QEMU -nographic toma posse do stdin
            # herdado (monitor/console). Sem isolar, a ponte 1 (spawnada depois,
            # herda o mesmo handle de stdin) trava na inicialização e nunca relaya
            # o frame J4-J6 -> ACK timeout -> E-STOP do ESP.
            p_esp = subprocess.Popen(
                [
                    qemu_exe,
                    "-nographic",
                    "-machine", "esp32s3",
                    "-drive",  f"file={qemu_firmware},if=mtd,format=raw",
                    "-serial", f"file:{serial_log}",
                    "-serial", "tcp::30100,server,nowait",
                    "-nic", f"user,model=open_eth,hostfwd=tcp::{rtde_port}-:{rtde_port},hostfwd=tcp::{http_port}-:80,hostfwd=tcp::30101-:30101",
                ],
                stdout=(qemu_host_log := open(os.path.join(logs_dir, "qemu_host.log"), "w", encoding="utf-8")),
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
            )
            log_handles.append(qemu_host_log)
            processes.append(p_esp)

            # 5. Ponte 1 SOMENTE após o ESP terminar o boot (HTTP :8080 ativo): ao
            #    inicializar o driver UART1, o ESP reseta a conexao do chardev :30100;
            #    conectar durante o boot mata o rele. stdin=DEVNULL + log dedicado.
            wait_for_port(http_port, 90.0)
            time.sleep(2.0)
            ponte1_log = open(os.path.join(logs_dir, "ponte1.log"), "w", encoding="utf-8")
            log_handles.append(ponte1_log)
            ponte1_env = os.environ.copy()
            ponte1_env["EB15_PONTE1_LOG"] = os.path.join(logs_dir, "ponte1_self.log")
            processes.append(subprocess.Popen([
                sys.executable, "-u", os.path.join(WORKSPACE_DIR, "firmware", "qemu", "ponte1_uart.py")
            ], stdout=ponte1_log, stderr=subprocess.STDOUT, env=ponte1_env, stdin=subprocess.DEVNULL))
            logger.info("Log serial QEMU em: %s", serial_log)
            logger.info("Aguardando ESP32 QEMU inicializar e expor RTDE na porta %d (ate 90s)...", rtde_port)

        if args.target in ["eb15", "both"]:
            # Deadline compartilhado de 90s para todos os health checks do EB15
            time.sleep(3.0)  # boot mínimo
            HEALTH_T0    = time.monotonic()
            HEALTH_LIMIT = 90.0

            remaining = HEALTH_LIMIT - (time.monotonic() - HEALTH_T0)
            if p_webots.poll() is not None:
                webots_log.flush()
                logger.error(
                    "Webots encerrou durante a inicialização (exit=%s). Consulte %s.",
                    p_webots.returncode, os.path.join(logs_dir, "webots.log")
                )
                sys.exit(EXIT_DEPENDENCY)
            if not wait_for_port(config["network"]["rtde_eb15_port"], max(1.0, remaining)):
                logger.error("Health check falhou: RTDE :30003 indisponivel.")
                sys.exit(EXIT_TIMEOUT)

            # Verifica encoder ports com o tempo restante do deadline
            enc_esp_port = config["network"]["encoder_esp_port"]
            enc_uno_port = config["network"]["encoder_uno_port"]
            remaining = HEALTH_LIMIT - (time.monotonic() - HEALTH_T0)
            webots_ok = remaining > 0 and wait_for_port(enc_esp_port, min(20.0, remaining))

            if not webots_ok:
                webots_log.flush()
                logger.error(
                    "Health check falhou: hil_bridge do Webots não abriu a porta %d. "
                    "Fallbacks são proibidos no Passo 4; consulte %s.",
                    enc_esp_port, os.path.join(logs_dir, "webots.log")
                )
                sys.exit(EXIT_TIMEOUT)

            # Verifica porta do encoder Uno (30104)
            remaining = HEALTH_LIMIT - (time.monotonic() - HEALTH_T0)
            if remaining <= 0 or not wait_for_port(enc_uno_port, min(remaining, 15.0)):
                logger.error("Health check falhou: encoder Uno :%d indisponivel.", enc_uno_port)
                sys.exit(EXIT_TIMEOUT)

            logger.info("Todos os health checks EB15 passaram (%.1fs).", time.monotonic() - HEALTH_T0)
            eb15_history = run_trajectory_eb15(traj)
            save_csv(f"benchmark_{args.trajectory}_eb15.csv", eb15_history, "eb15")

        if args.target in ["ursim", "both"]:
            ursim_history = run_trajectory_ursim(traj)
            save_csv(f"benchmark_{args.trajectory}_ursim.csv", ursim_history, "ursim")

        final_status = "ok"
        logger.info("Benchmark concluido com sucesso!")

    except KeyboardInterrupt:
        logger.warning("Execucao interrompida pelo usuario via Ctrl+C.")
        final_status = "interrupted"
    finally:
        if simavr_started:
            subprocess.run(["docker", "rm", "-f", simavr_container], capture_output=True)
        if rtde_server_prebind:
            try:
                rtde_server_prebind.close()
            except OSError:
                pass

        t_end = time.time()
        save_manifest(args, eb15_history, ursim_history, t_start, t_end, final_status)

        if processes:
            logger.info("Encerrando processos de simulacao ativos...")
            for p in processes:
                try:
                    p.terminate()
                    p.wait(timeout=2.0)
                except Exception:
                    try:
                        p.kill()
                    except Exception:
                        pass
            logger.info("Todos os processos foram limpos. OK")
        for handle in log_handles:
            try:
                handle.close()
            except Exception:
                pass

if __name__ == "__main__":
    main()
