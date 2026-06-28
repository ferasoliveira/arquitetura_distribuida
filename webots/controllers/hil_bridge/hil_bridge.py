"""
hil_bridge.py — Ponte HIL Webots para o EB15
==============================================
TCC Fernando Oliveira — Passo 3: Ambiente Virtual, Pontes e Simulação

Controlador Webots (supervisor) que implementa a ponte Hardware-in-the-Loop:

  - A cada timestep (5 ms = basicTimeStep):
    1. Troca ground truth/comando com a ponte 2 independente (J1–J3)
    2. Troca ground truth/comando com a ponte 3 independente (J4–J6)
    3. Converte pulsos acumulados → referências angulares (rad)
    4. Aplica soft limits do EB15 às referências antes de passar ao UR10e
    5. Comanda os 6 motores do UR10e em modo velocidade/posição
    6. Lê os 6 sensores de posição do UR10e (ground truth)
    7. Quantiza em 12 bits (AS5600) e devolve às duas pontes

Separação de grandezas mantida:
  - g_cmd_deg[i]     : ângulo comandado pela ponte (entrada dos firmwares)
  - g_applied_rad[i] : ângulo efetivamente aplicado ao UR10e (após limites)
  - g_measured_rad[i]: ângulo real lido pelos sensores do UR10e (ground truth)

Mapeamento de juntas EB15 → UR10e (nomes dos devices):
  J1 → shoulder_pan_joint
  J2 → shoulder_lift_joint
  J3 → elbow_joint
  J4 → wrist_1_joint
  J5 → wrist_2_joint
  J6 → wrist_3_joint

Instalação no Webots:
  O arquivo deve estar em:
    Códigos/webots/controllers/hil_bridge/hil_bridge.py
  O controlador é referenciado no .wbt como:
    controller "hil_bridge"
"""

import os
import socket
import struct
import math
import logging

# Webots controller API
from controller import Robot, Supervisor

# ============================================================================
# Constantes (espelhadas de protocol.py para evitar dependência em runtime)
# ============================================================================

BRIDGE_ESP_PORT = 30301    # canal interno controlador <-> ponte 2
BRIDGE_UNO_PORT = 30302    # canal interno controlador <-> ponte 3

STEPS_PER_DEG    = (200 * 4 * 1.0) / 360.0   # ≈ 2.222 steps/grau
AS5600_RES       = 4096

# Limites de junta EB15 (graus) — aplicados como restrição adicional
EB15_MIN_DEG = [-170.0, -45.0, -120.0, -180.0, -90.0,  -360.0]
EB15_MAX_DEG = [ 170.0, 180.0,  120.0,  180.0,  90.0,   360.0]

# Nomes dos dispositivos UR10e no Webots (ordem J1–J6)
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Limites angulares do UR10e (radianos) — preservados para validação
UR10E_MIN_RAD = [math.radians(d) for d in [-360, -360, -360, -360, -360, -360]]
UR10E_MAX_RAD = [math.radians(d) for d in [ 360,  360,  360,  360,  360,  360]]

HOST = "127.0.0.1"

# Co-simulação SÍNCRONA (lockstep): cada passo do Webots dispara exatamente UM ciclo de
# controle no ESP (1:1). Nesse regime o teto de velocidade do motor-proxy de J1-J3 tem de
# ser ELEVADO — ver nota no laço principal (item: realização fiel do passo do stepper).
LOCKSTEP = os.environ.get("EB15_LOCKSTEP", "0") == "1"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d [hil_bridge] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
    handlers=[
        logging.FileHandler("hil_bridge.log", mode="w", encoding="utf-8"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("hil_bridge")


# ============================================================================
# Estado global da ponte
# ============================================================================

g_cmd_deg      = [0.0] * 6   # Ângulo comandado em graus (entrada dos firmwares)
g_applied_rad  = [0.0] * 6   # Ângulo aplicado ao UR10e (após limites)
g_measured_rad = [0.0] * 6   # Ângulo real lido pelos sensores do UR10e
g_encoder_raw  = [0] * 6     # Valores AS5600 simulados (0–4095)


# ============================================================================
# Utilitários
# ============================================================================

def clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


def steps_to_deg(steps: float) -> float:
    return steps / STEPS_PER_DEG


def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def rad_to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def quantize_as5600(deg: float) -> int:
    """Converte ângulo (graus) para valor bruto AS5600 de 12 bits (0–4095)."""
    norm = deg % 360.0
    if norm < 0:
        norm += 360.0
    return int(round(norm * AS5600_RES / 360.0)) % AS5600_RES


# ============================================================================
# Clientes TCP para leitura de Step/Dir dos firmwares simulados
# ============================================================================

_g_bridge_sockets = {}


def exchange_bridge(port: int, measured_raw: list) -> list:
    """
    Envia ground truth e recebe o comando de uma ponte elétrica independente.

    Protocolo:
      - Envia 6 bytes: 3 x uint16 AS5600 medidos
      - Recebe 12 bytes: 3 x float graus comandados

    Conexão PERSISTENTE: o controlador roda a cada basicTimeStep (5 ms) e o
    connect/close por chamada custava ~6 ms/porta (medido), dominando ~91% do tempo do
    passo e travando o Webots em ~0,38x tempo real. Reusando o socket (reconecta em
    falha) o transporte cai para frações de ms. Retorna [None]*3 em falha.
    """
    s = _g_bridge_sockets.get(port)
    if s is None:
        try:
            s = socket.create_connection((HOST, port), timeout=2.0)
            s.settimeout(2.0)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            _g_bridge_sockets[port] = s
        except OSError:
            _g_bridge_sockets[port] = None
            return [None, None, None]
    try:
        s.sendall(struct.pack('<3H', *measured_raw))
        data = b''
        while len(data) < 12:
            chunk = s.recv(12 - len(data))
            if not chunk:
                raise ConnectionError("conexao fechada pela ponte")
            data += chunk
        return list(struct.unpack('<3f', data))
    except (OSError, ConnectionError):
        try:
            s.close()
        except OSError:
            pass
        _g_bridge_sockets[port] = None
        return [None, None, None]


def exchange_bridge6(port: int, measured_raw6: list) -> list:
    """LOCKSTEP: troca de 6 juntas com a ponte2 (que sub-mestra o ESP+Uno).
    Envia 12 bytes (6x uint16 medido J1-J6); recebe 24 bytes (6 floats graus comandados).
    Conexão persistente. Retorna [None]*6 em falha."""
    s = _g_bridge_sockets.get(port)
    if s is None:
        try:
            s = socket.create_connection((HOST, port), timeout=2.0)
            s.settimeout(2.0)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            _g_bridge_sockets[port] = s
        except OSError:
            _g_bridge_sockets[port] = None
            return [None] * 6
    try:
        s.sendall(struct.pack('<6H', *measured_raw6))
        data = b''
        while len(data) < 24:
            chunk = s.recv(24 - len(data))
            if not chunk:
                raise ConnectionError("conexao fechada pela ponte")
            data += chunk
        return list(struct.unpack('<6f', data))
    except (OSError, ConnectionError):
        try:
            s.close()
        except OSError:
            pass
        _g_bridge_sockets[port] = None
        return [None] * 6


# ============================================================================
# Controlador principal (loop Webots)
# ============================================================================

def main():
    robot    = Supervisor()
    timestep = int(robot.getBasicTimeStep())   # 5 ms

    logger.info("=== HIL Bridge iniciada | basicTimeStep=%d ms ===", timestep)

    # Obtém dispositivos do UR10e
    motors  = []
    sensors = []
    for name in JOINT_NAMES:
        m = robot.getDevice(name)
        s = robot.getDevice(name + "_sensor")
        if m is None:
            logger.error("Motor não encontrado: %s", name)
        else:
            m.setPosition(float('inf'))   # Modo velocidade
            m.setVelocity(0.0)
            motors.append(m)

        if s is None:
            logger.error("Sensor não encontrado: %s_sensor", name)
        else:
            s.enable(timestep)
            sensors.append(s)

    if len(motors) != 6 or len(sensors) != 6:
        logger.error("Dispositivos incompletos. Verifique o nome do robô no .wbt")
        return

    logger.info("Dispositivos UR10e encontrados: %s", ", ".join(JOINT_NAMES))

    # Leitura inicial dos sensores (pose zero)
    robot.step(timestep)
    for i, s in enumerate(sensors):
        g_measured_rad[i] = s.getValue()
        g_applied_rad[i]  = g_measured_rad[i]
        g_cmd_deg[i]      = rad_to_deg(g_measured_rad[i])
        g_encoder_raw[i]  = quantize_as5600(g_cmd_deg[i])

    logger.info("Pose zero lida: %s",
                [f"J{i+1}={rad_to_deg(g_measured_rad[i]):.2f}°" for i in range(6)])

    tick = 0

    # ========================================================================
    # Loop principal de simulação (a cada 5 ms)
    # ========================================================================
    while robot.step(timestep) != -1:
        tick += 1

        # -- 1. Lê Step/Dir do ESP32 (J1–J3) — lockstep (síncrono) ou modo livre --
        esp_degs = exchange_bridge(BRIDGE_ESP_PORT, g_encoder_raw[0:3])
        for i in range(3):
            if esp_degs[i] is not None:
                g_cmd_deg[i] = esp_degs[i]

        # -- 2. Lê Step/Dir do Arduino (J4–J6) — malha fechada no Uno (sync tempo-real) --
        uno_degs = exchange_bridge(BRIDGE_UNO_PORT, g_encoder_raw[3:6])
        for i in range(3):
            if uno_degs[i] is not None:
                g_cmd_deg[i + 3] = uno_degs[i]

        # -- 3. Aplica limites EB15 + converte para radianos --
        for i in range(6):
            clamped        = clamp(g_cmd_deg[i], EB15_MIN_DEG[i], EB15_MAX_DEG[i])
            g_applied_rad[i] = deg_to_rad(clamped)

        # -- 4. Comanda motores UR10e --
        for i, motor in enumerate(motors):
            motor.setPosition(g_applied_rad[i])
            if LOCKSTEP and i < 3:
                # LOCKSTEP, J1-J3: o comando é a POSIÇÃO ACUMULADA dos passos do
                # firmware (dead-reckoning do stepper). Um stepper real executa os
                # passos comandados dentro do próprio ciclo de 5 ms — posição = passos,
                # SEM o atraso de 2ª ordem de um servo. Com o teto de 45°/s o motor-proxy
                # do Webots introduz um atraso que o stepper não tem; como em lockstep há
                # 1 ciclo de controle por passo do Webots, o acumulador (sem atraso) cresce
                # a ~45°/s e o motor, no mesmo teto, não consegue acompanhar → o erro nunca
                # fecha e o comando satura no limite (170°), congelando a junta. Elevando o
                # teto ao máximo do motor, a junta REALIZA a posição comandada a cada passo;
                # a velocidade EFETIVA continua limitada pela taxa de passos do PID no
                # firmware (~45°/s). Não toca o algoritmo de controle — só a fidelidade do
                # atuador-proxy. (No modo livre o Webots roda ~8x e há folga entre comandos,
                # por isso 45°/s basta lá.)
                motor.setVelocity(motor.getMaxVelocity())
            else:
                # Velocidade máxima proporcional (rad/s)
                max_vel = math.radians(90.0 if i >= 3 else 45.0)
                motor.setVelocity(max_vel)

        # -- 5. Lê sensores (ground truth) --
        for i, sensor in enumerate(sensors):
            g_measured_rad[i] = sensor.getValue()

        # -- 6. Quantiza em 12 bits (AS5600 simulados) --
        for i in range(6):
            g_encoder_raw[i] = quantize_as5600(rad_to_deg(g_measured_rad[i]))

        # -- 7. Log periódico (a cada 200 ciclos = 1 s) --
        if tick % 200 == 0:
            logger.info(
                "t=%.1fs | cmd=[%s] | meas=[%s]",
                tick * timestep / 1000.0,
                " ".join(f"{rad_to_deg(g_applied_rad[i]):+.1f}°" for i in range(6)),
                " ".join(f"{rad_to_deg(g_measured_rad[i]):+.1f}°" for i in range(6)),
            )

        # -- 8. Verificação de integridade --
        for i in range(6):
            if math.isnan(g_measured_rad[i]) or math.isinf(g_measured_rad[i]):
                logger.error("NaN/Inf detectado na junta %d — simulação instável!", i + 1)
                # Força posição segura
                motors[i].setPosition(0.0)

    logger.info("Simulação encerrada após %d ciclos", tick)


if __name__ == "__main__":
    main()
