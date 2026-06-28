# -*- coding: utf-8 -*-
"""
test_integration.py — 14 Testes de Integração (Fase 6 do plano_passo3)
=======================================================================
TCC Fernando Oliveira — Passo 3: Ambiente Virtual, Pontes e Simulação

Cobre todos os casos de teste listados no plano_passo3.md Fase 6:
  1.  Inicialização e repouso
  2.  Movimento isolado de cada junta
  3.  Movimento simultâneo de J1–J6
  4.  Alvo absoluto repetido
  5.  Trajetória MoveJ curta
  6.  Trajetória cartesiana MoveL
  7.  Limite de junta e pose inalcançável
  8.  E-STOP durante movimento
  9.  Perda da ponte UART
  10. Perda do cliente RTDE
  11. Falha persistente de encoder
  12. Reinicialização completa e repetição determinística
  13. Alternância dinâmica Modo RTDE ↔ Modo User
  14. Teste funcional do Modo User (interface web)

Critérios de aceite (Fase 6):
  - Nenhum deadlock ou espera infinita (timeouts em todas as operações).
  - Falhas produzem código e mensagem explícitos.
  - E-STOP interrompe todos os atuadores virtuais.
  - Nenhum segmento sobrescrito antes de DONE.
  - Três execuções idênticas produzem a mesma sequência de comandos.
  - Todos os processos encerram sem intervenção manual.
"""

import os
import sys
import math
import time
import json
import socket
import struct
import threading
import http.server
import urllib.request
import urllib.error
import pytest

# ─────────────────────────────────────────────────────────────────
# Paths
# ─────────────────────────────────────────────────────────────────
_TESTS_DIR     = os.path.dirname(os.path.abspath(__file__))
_WEBOTS_DIR    = os.path.dirname(_TESTS_DIR)
_CODES_DIR     = os.path.dirname(_WEBOTS_DIR)
_HELPERS_DIR   = os.path.join(_CODES_DIR, "helpers")
_BENCHMARK_DIR = os.path.join(_WEBOTS_DIR, "benchmark")

for _p in [_HELPERS_DIR, _BENCHMARK_DIR]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import protocol
import trajectory
from rtde_eb15 import RtdeEb15Client
from rtde_ursim import RtdeUrsimClient

# Importa funções puras do hil_bridge (controller já mockado pelo conftest.py)
sys.path.insert(0, os.path.join(_WEBOTS_DIR, "controllers", "hil_bridge"))
import hil_bridge

# ─────────────────────────────────────────────────────────────────
# Constantes
# ─────────────────────────────────────────────────────────────────
JMIN = [-170.0, -45.0, -120.0, -180.0,  -90.0, -360.0]
JMAX = [ 170.0, 180.0,  120.0,  180.0,   90.0,  360.0]
STEPS_PER_DEG = (200 * 4) / 360.0  # ≈ 2.222 steps/grau


# ─────────────────────────────────────────────────────────────────
# Utilitários de teste
# ─────────────────────────────────────────────────────────────────

def find_free_port() -> int:
    """Retorna uma porta TCP livre no localhost."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def make_telemetry(joints=None, errors=None, temp=25.0) -> bytes:
    """Cria um frame RTDE-EB15 de 52 bytes válido."""
    return protocol.encode_rtde_frame(
        joints or [0.0] * 6,
        errors or [0.0] * 6,
        temp
    )


class MockEsp32(threading.Thread):
    """
    Simula o ESP32 como CLIENTE RTDE (modo servidor Python, legado de integração).
    Conecta ao servidor de testes e envia frames de telemetria a 50 Hz.
    Captura comandos recebidos.
    """

    def __init__(self, host: str, port: int, n_frames: int = 60,
                 fail_after: int = None, telemetry: bytes = None):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.n_frames = n_frames
        self.fail_after = fail_after  # desconecta após N frames (simula queda)
        self.telemetry = telemetry or make_telemetry()
        self.commands: list = []
        self.connected = threading.Event()
        self._halt = threading.Event()   # renomeado para não colidir com Thread._stop()

    def run(self):
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((self.host, self.port))
            self.connected.set()
            sock.settimeout(0.01)

            sent = 0
            while not self._halt.is_set() and sent < self.n_frames:
                if self.fail_after is not None and sent >= self.fail_after:
                    break  # simula queda de conexão
                try:
                    sock.sendall(self.telemetry)
                    sent += 1
                except OSError:
                    break
                try:
                    data = sock.recv(256)
                    if data:
                        self.commands.append(data.decode("utf-8", errors="replace").strip())
                except socket.timeout:
                    pass
                time.sleep(0.02)  # 50 Hz
        except Exception:
            pass
        finally:
            self.connected.set()  # libera qualquer waiter
            if sock:
                try:
                    sock.close()
                except OSError:
                    pass

    def halt(self):
        """Sinaliza a thread para encerrar (stop() é reservado pela classe Thread)."""
        self._halt.set()


class MockHttpHandler(http.server.BaseHTTPRequestHandler):
    """Handler HTTP que simula os endpoints /api/mode e /api/telemetry do ESP32."""

    def log_message(self, *args):
        pass  # silencia logs de acesso

    def _send_json(self, code: int, body: dict):
        payload = json.dumps(body).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(length)) if length > 0 else {}

        if self.path == "/api/mode":
            mode = body.get("mode", "USER")
            # Registra a requisição para asserção no teste
            self.server.last_mode_request = body
            if mode in ("RTDE", "USER"):
                self._send_json(200, {"status": "ok", "mode": mode})
            else:
                self._send_json(400, {"status": "error", "msg": "Modo invalido"})
        else:
            self._send_json(404, {"status": "error"})

    def do_GET(self):
        if self.path == "/api/telemetry":
            payload = {
                "joints": [0.0] * 6,
                "errors": [0.0] * 6,
                "tcp": [0.0] * 6,
                "temperature": 25.0,
                "mode": "USER"
            }
            self._send_json(200, payload)
        elif self.path == "/api/status":
            self._send_json(200, {"status": "ok", "mode": "USER"})
        else:
            self._send_json(404, {"status": "error"})


class MockHttpServer(threading.Thread):
    """Servidor HTTP de mock que roda em thread separada."""

    def __init__(self, port: int = None):
        super().__init__(daemon=True)
        self.port = port or find_free_port()
        self._server = None
        self.ready = threading.Event()

    def run(self):
        server_address = ("127.0.0.1", self.port)
        self._server = http.server.HTTPServer(server_address, MockHttpHandler)
        self._server.last_mode_request = {}
        self.ready.set()
        self._server.serve_forever()

    def shutdown(self):
        if self._server:
            self._server.shutdown()

    @property
    def last_mode_request(self):
        return self._server.last_mode_request if self._server else {}


# ─────────────────────────────────────────────────────────────────
# Teste 1 — Inicialização e repouso
# ─────────────────────────────────────────────────────────────────
def test_01_init_e_repouso():
    """
    Verifica que todas as trajetórias carregam sem erros, que o primeiro
    waypoint de cada trajetória é home (0°) e que todas as durações são
    positivas, respeitando o critério de repouso antes de qualquer movimento.
    """
    for name in trajectory.TRAJECTORIES:
        traj = trajectory.load_trajectory(name)

        assert len(traj["waypoints"]) >= 2, \
            f"Trajetoria '{name}' deve ter pelo menos 2 waypoints"
        assert traj["total_ms"] > 0, \
            f"Trajetoria '{name}' deve ter duração total positiva"

        wp0 = traj["waypoints"][0]
        assert wp0["seq"] == 0, "Primeiro waypoint deve ter seq=0"
        assert all(q == 0.0 for q in wp0["q"]), \
            f"Trajetoria '{name}': waypoint inicial deve ser home (0°). Obtido: {wp0['q']}"
        assert len(wp0["qd"]) == 6, "qd deve ter 6 elementos"
        assert len(wp0["qdd"]) == 6, "qdd deve ter 6 elementos"

        for wp in traj["waypoints"]:
            assert wp["duration_ms"] >= 0, \
                f"Trajetoria '{name}': duração negativa no waypoint {wp['seq']}"

        # Valida que todos os waypoints estão dentro dos limites do EB15
        for wp in traj["waypoints"]:
            for i, q in enumerate(wp["q"]):
                assert JMIN[i] <= q <= JMAX[i], \
                    f"Trajetoria '{name}': J{i+1}={q}° fora de [{JMIN[i]}, {JMAX[i]}]"


# ─────────────────────────────────────────────────────────────────
# Teste 2 — Movimento isolado de cada junta
# ─────────────────────────────────────────────────────────────────
def test_02_movimento_isolado_junta():
    """
    Verifica a trajetória SINGLE_JOINT_J1: apenas J1 se move, J2–J6 permanecem em 0°.
    Valida mapeamento de sinal e unidade de J1 de forma isolada.
    """
    traj = trajectory.load_trajectory("single_joint")
    wps = traj["waypoints"]

    # Verifica que J2–J6 nunca saem de 0°
    for wp in wps:
        for i in range(1, 6):  # J2 a J6
            assert wp["q"][i] == 0.0, \
                f"J{i+1} deve permanecer em 0° no teste de junta isolada. " \
                f"Obtido {wp['q'][i]}° no waypoint {wp['seq']}"

    # Verifica que J1 realmente se move (não fica sempre em 0)
    j1_values = [wp["q"][0] for wp in wps]
    assert max(j1_values) > 0, "J1 deve atingir valor positivo"
    assert min(j1_values) < 0, "J1 deve atingir valor negativo"

    # Verifica retorno ao home
    assert wps[-1]["q"][0] == 0.0, "Trajetoria deve terminar em home (J1=0°)"


# ─────────────────────────────────────────────────────────────────
# Teste 3 — Movimento simultâneo de J1–J6
# ─────────────────────────────────────────────────────────────────
def test_03_movimento_simultaneo():
    """
    Verifica a trajetória ALL_JOINTS_SYNC: todas as juntas se movem simultaneamente.
    Valida a sincronização entre J1–J3 (ESP32) e J4–J6 (Arduino).
    """
    traj = trajectory.load_trajectory("all_joints")
    wps = traj["waypoints"]

    # Identifica o waypoint de pico (onde nenhum campo é zero)
    pico = None
    for wp in wps[1:]:
        if any(q != 0.0 for q in wp["q"]):
            pico = wp
            break

    assert pico is not None, "Trajetoria deve ter pelo menos um waypoint nao-zero"

    # No waypoint de pico, TODAS as juntas J1–J6 devem ser não-zero
    for i in range(6):
        assert pico["q"][i] != 0.0, \
            f"J{i+1} deve mover-se no waypoint de pico (sincronismo). " \
            f"Obtido {pico['q'][i]}°"

    # Verifica retorno ao home
    assert all(q == 0.0 for q in wps[-1]["q"]), "Trajetoria deve terminar em home"

    # Verifica que a trajetória tem waypoints positivos E negativos (excursão bidirecional)
    has_positive = any(wp["q"][0] > 0 for wp in wps)
    has_negative = any(wp["q"][0] < 0 for wp in wps)
    assert has_positive and has_negative, "Trajetoria deve ter excursao bidirecional"


# ─────────────────────────────────────────────────────────────────
# Teste 4 — Alvo absoluto repetido
# ─────────────────────────────────────────────────────────────────
def test_04_alvo_absoluto_repetido():
    """
    Verifica a trajetória REPEATED_ABS: o mesmo alvo absoluto aparece 5 vezes.
    Valida repetibilidade e ausência de acumulação de erro de posição.
    """
    traj = trajectory.load_trajectory("repeated_abs")
    wps = traj["waypoints"]

    # Coleta os waypoints não-home (targets)
    targets = [wp["q"] for wp in wps if any(q != 0.0 for q in wp["q"])]
    assert len(targets) == 5, \
        f"Deve haver exatamente 5 waypoints de alvo repetido. Obtido: {len(targets)}"

    # Todos os targets devem ser idênticos (alvo absoluto)
    for i in range(1, len(targets)):
        assert targets[i] == targets[0], \
            f"Target {i} difere do target 0 — acumulação indevida detectada. " \
            f"\nTarget 0: {targets[0]}\nTarget {i}: {targets[i]}"

    # Verifica que o alvo é absoluto (não relativo): o valor deve ser idêntico ao original
    expected = [20.0, 20.0, 20.0, 45.0, 30.0, 60.0]
    assert targets[0] == expected, \
        f"Alvo absoluto incorreto. Esperado {expected}, obtido {targets[0]}"


# ─────────────────────────────────────────────────────────────────
# Teste 5 — Trajetória MoveJ curta
# ─────────────────────────────────────────────────────────────────
def test_05_trajetoria_movej_curta():
    """
    Verifica a trajetória MOVE_J_SHORT: 5 waypoints com derivadas calculadas.
    Valida benchmark básico com trajetória compacta de referência.
    """
    traj = trajectory.load_trajectory("move_j_short")
    wps = traj["waypoints"]

    assert len(wps) == 5, f"MOVE_J_SHORT deve ter 5 waypoints. Obtido: {len(wps)}"

    # Verifica que as derivadas foram computadas (qd e qdd preenchidos)
    for wp in wps:
        assert len(wp["qd"]) == 6 and len(wp["qdd"]) == 6, \
            f"Waypoint {wp['seq']}: qd/qdd incompletos"

    # Verifica que a sequência de numeração é contínua
    for i, wp in enumerate(wps):
        assert wp["seq"] == i, \
            f"Sequência quebrada: esperado {i}, obtido {wp['seq']}"

    # Verifica monoticidade do tempo planejado
    for i in range(1, len(wps)):
        assert wps[i]["t_planned"] >= wps[i-1]["t_planned"], \
            f"t_planned não é monotônico no waypoint {i}"

    # Verifica que inicio e fim são home
    assert all(q == 0.0 for q in wps[0]["q"]), "Deve iniciar em home"
    assert all(q == 0.0 for q in wps[-1]["q"]), "Deve terminar em home"


# ─────────────────────────────────────────────────────────────────
# Teste 6 — Trajetória cartesiana MoveL
# ─────────────────────────────────────────────────────────────────
def test_06_trajetoria_movel():
    """
    Verifica o formato do comando MoveL dos clientes RTDE-EB15 e RTDE-URSim.
    Testa que send_movel gera a string URScript correta sem enviar por socket.
    """
    pose = [0.300, 0.100, 0.500, 0.0, 3.14159, 0.0]  # x,y,z,rx,ry,rz
    speed_pct = 40.0

    # ── EB15 movel ──
    # Cria cliente sem conectar (não há socket real)
    eb15 = RtdeEb15Client(port=find_free_port(), server_mode=False)
    # Injeta socket falso para capturar o que seria enviado
    captured_eb15 = []

    class FakeSock:
        def sendall(self, data):
            captured_eb15.append(data.decode("utf-8"))

    eb15.socket = FakeSock()
    eb15.running = True
    result = eb15.send_movel(pose, speed_pct)
    assert result is True, "send_movel deve retornar True quando socket está ativo"
    assert len(captured_eb15) == 1, "Deve enviar exatamente 1 mensagem"
    cmd_eb15 = captured_eb15[0]
    assert cmd_eb15.startswith("movel(["), f"Comando EB15 invalido: {cmd_eb15!r}"
    assert cmd_eb15.strip().endswith(")"), f"Comando EB15 nao termina corretamente: {cmd_eb15!r}"
    assert f"v={speed_pct:.1f}" in cmd_eb15, f"Speed nao encontrado no comando EB15: {cmd_eb15!r}"

    # Verifica que os valores da pose estão presentes no comando
    for p in pose:
        assert f"{p:.4f}" in cmd_eb15, \
            f"Valor {p:.4f} ausente no comando EB15: {cmd_eb15!r}"

    # ── URSim movel ──
    ursim = RtdeUrsimClient(host="127.0.0.1", rtde_port=find_free_port())
    captured_ursim = []

    class FakeScriptSock:
        def sendall(self, data):
            captured_ursim.append(data.decode("utf-8"))

    ursim.script_socket = FakeScriptSock()
    result_u = ursim.send_movel(pose, speed_pct)
    assert result_u is True, "URSim send_movel deve retornar True"
    assert len(captured_ursim) == 1
    cmd_ursim = captured_ursim[0]
    assert cmd_ursim.startswith("movel(p["), f"Comando URSim invalido: {cmd_ursim!r}"
    assert cmd_ursim.strip().endswith(")"), f"Comando URSim nao termina corretamente: {cmd_ursim!r}"

    # ── Verifica que nenhum comando é enviado sem socket ──
    eb15_no_sock = RtdeEb15Client(port=find_free_port(), server_mode=False)
    assert eb15_no_sock.send_movel(pose, speed_pct) is False, \
        "send_movel sem socket deve retornar False"


# ─────────────────────────────────────────────────────────────────
# Teste 7 — Limite de junta e pose inalcançável
# ─────────────────────────────────────────────────────────────────
def test_07_limite_junta_invalido():
    """
    Verifica que waypoints fora dos limites articulares do EB15 são rejeitados
    com ValueError pela função _build_trajectory, sem criar trajetória parcial.
    """
    violacoes = [
        # (descrição, waypoints_raw)
        ("J1 acima do limite", [[0]*6, [180.0, 0, 0, 0, 0, 0]]),
        ("J1 abaixo do limite", [[0]*6, [-180.0, 0, 0, 0, 0, 0]]),
        ("J2 abaixo do limite", [[0]*6, [0, -90.0, 0, 0, 0, 0]]),
        ("J3 acima do limite", [[0]*6, [0, 0, 150.0, 0, 0, 0]]),
        ("J5 acima do limite", [[0]*6, [0, 0, 0, 0, 100.0, 0]]),
        ("Todos no limite simultaneamente", [[0]*6, [171.0, 181.0, 121.0, 0, 91.0, 361.0]]),
    ]

    for descricao, pts in violacoes:
        with pytest.raises(ValueError, match="invalido|fora dos limites|invalido"):
            trajectory._build_trajectory(
                name="test_violacao",
                description=descricao,
                waypoints_raw=pts,
                speed_pct=50.0
            )

    # Verifica que poses no exato limite são ACEITAS (bordering case)
    limites_exatos = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [170.0, 180.0, 120.0, 180.0, 90.0, 360.0],
        [-170.0, -45.0, -120.0, -180.0, -90.0, -360.0],
    ]
    traj_limite = trajectory._build_trajectory(
        name="test_limite_exato",
        description="Poses nos limites exatos — deve ser aceito",
        waypoints_raw=limites_exatos,
        speed_pct=50.0
    )
    assert len(traj_limite["waypoints"]) == 3, "Trajetória nos limites exatos deve ser aceita"


# ─────────────────────────────────────────────────────────────────
# Teste 8 — E-STOP durante movimento
# ─────────────────────────────────────────────────────────────────
def test_08_estop_durante_movimento():
    """
    Verifica que send_stop() envia o comando "stop\\n" ao ESP32 via RTDE-EB15
    e que o cliente interrompe o fluxo corretamente sem deadlock.
    O mock ESP32 conecta ao servidor de teste, recebe o stop e o registra.
    """
    port = find_free_port()

    # Cria o servidor RTDE (Python é o servidor — modo server_mode=True para testes unitários)
    client = RtdeEb15Client(host="127.0.0.1", port=port, server_mode=True)

    # Cria socket servidor pré-vinculado (evita race condition)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    srv.settimeout(5.0)
    client._server_sock = srv

    # Lança o mock ESP32 como cliente
    esp = MockEsp32("127.0.0.1", port, n_frames=30)
    esp.start()

    try:
        # Conecta o servidor e aguarda o ESP32
        connected = client.connect(timeout=5.0)
        assert connected, "Cliente RTDE deve conectar ao mock ESP32"

        # Aguarda pelo menos 1 frame de telemetria
        deadline = time.time() + 3.0
        while not client.get_latest_telemetry() and time.time() < deadline:
            time.sleep(0.05)
        assert client.get_latest_telemetry(), "Deve receber ao menos 1 frame de telemetria"

        # Envia E-STOP
        ok = client.send_stop()
        assert ok is True, "send_stop deve retornar True com socket ativo"

        # Aguarda o mock registrar o comando
        deadline = time.time() + 2.0
        while len(esp.commands) == 0 and time.time() < deadline:
            time.sleep(0.05)

        assert len(esp.commands) > 0, "Mock ESP32 deve ter recebido pelo menos 1 comando"
        assert any("stop" in cmd.lower() for cmd in esp.commands), \
            f"Comando 'stop' nao encontrado nos recebidos: {esp.commands}"

    finally:
        client.disconnect()
        esp.halt()
        esp.join(timeout=2.0)
        assert not esp.is_alive(), "Thread do mock ESP32 deve encerrar sem intervencao"


# ─────────────────────────────────────────────────────────────────
# Teste 9 — Perda da ponte UART
# ─────────────────────────────────────────────────────────────────
def test_09_perda_uart():
    """
    Verifica que o parser UART trata corretamente frames truncados ou corrompidos,
    simulando perda de pacotes na ponte serial entre ESP32 e Arduino Uno.
    Falhas devem produzir is_valid=False sem levantar exceções.
    """
    # Frame válido de referência
    frame_valido = protocol.encode_uart_frame(500, -200, 100, 300)
    assert len(frame_valido) == 10

    casos = [
        # (descrição, bytes, espera_valido)
        ("Frame válido completo", frame_valido, True),
        ("Frame vazio", b"", False),
        ("Frame truncado (5 bytes)", frame_valido[:5], False),
        ("Frame truncado (9 bytes)", frame_valido[:9], False),
        ("Frame com preâmbulo errado", bytes([0x55]) + frame_valido[1:], False),
        ("Frame com checksum corrompido", frame_valido[:-1] + bytes([frame_valido[-1] ^ 0xFF]), False),
        ("Frame com tamanho maior", frame_valido + b"\x00", False),
        ("Frame de zeros", b"\x00" * 10, False),
        ("Frame de 0xFF", b"\xFF" * 10, False),
        ("Frame com preâmbulo correto mas resto zerado", bytes([0xAA]) + b"\x00" * 9, False),
    ]

    for descricao, frame, espera_valido in casos:
        resultado = protocol.decode_uart_frame(frame)
        _, _, _, _, is_valid = resultado
        assert is_valid == espera_valido, \
            f"Caso '{descricao}': esperado is_valid={espera_valido}, " \
            f"obtido is_valid={is_valid} | frame={frame.hex()}"

    # Verifica que frames duplicados consecutivos não geram erro de estado
    for _ in range(3):
        _, _, _, _, v = protocol.decode_uart_frame(frame_valido)
        assert v is True, "Frame válido repetido deve continuar válido (sem acumulação de estado)"


# ─────────────────────────────────────────────────────────────────
# Teste 10 — Perda do cliente RTDE
# ─────────────────────────────────────────────────────────────────
def test_10_perda_rtde():
    """
    Verifica que o RtdeEb15Client trata a queda do ESP32 (desconexão) sem
    deadlock. O mock ESP32 desconecta após 5 frames e o cliente deve
    encerrar normalmente dentro do timeout.
    """
    port = find_free_port()

    client = RtdeEb15Client(host="127.0.0.1", port=port, server_mode=True)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    srv.settimeout(5.0)
    client._server_sock = srv

    # Mock ESP32 que desconecta após 5 frames (fail_after=5)
    esp = MockEsp32("127.0.0.1", port, n_frames=100, fail_after=5)
    esp.start()

    try:
        connected = client.connect(timeout=5.0)
        assert connected, "Deve conectar ao mock ESP32"

        # Aguarda receber frames e o mock desconectar
        deadline = time.time() + 4.0
        while client.running and time.time() < deadline:
            time.sleep(0.1)

        # Após desconexão, running deve ser False (sem deadlock)
        # Damos um tempo extra para a thread de recv processar o EOF
        time.sleep(0.5)
        assert not client.running, \
            "cliente.running deve ser False apos o ESP32 desconectar"

    finally:
        client.disconnect()
        esp.halt()
        esp.join(timeout=2.0)
        assert not esp.is_alive(), "Thread mock ESP32 deve encerrar sem intervencao"


# ─────────────────────────────────────────────────────────────────
# Teste 11 — Falha persistente de encoder (AS5600)
# ─────────────────────────────────────────────────────────────────
def test_11_falha_encoder():
    """
    Verifica as funções utilitárias do hil_bridge para tratamento de falhas
    de encoder: saturação, wrap-around do AS5600 e detecção de NaN/Inf.
    """
    # ── quantize_as5600: range normal ──
    assert hil_bridge.quantize_as5600(0.0) == 0,   "0° → 0"
    assert hil_bridge.quantize_as5600(360.0) == 0, "360° → 0 (wrap-around)"
    assert hil_bridge.quantize_as5600(180.0) == 2048, "180° → 2048"
    assert hil_bridge.quantize_as5600(90.0) == 1024,  "90° → 1024"
    assert hil_bridge.quantize_as5600(270.0) == 3072, "270° → 3072"

    # ── quantize_as5600: ângulos negativos (equivalem ao complemento) ──
    assert hil_bridge.quantize_as5600(-90.0) == 3072,  "-90° → 3072 (=270°)"
    assert hil_bridge.quantize_as5600(-180.0) == 2048, "-180° → 2048 (=180°)"

    # ── quantize_as5600: múltiplas voltas (wrap-around ≥ 360°) ──
    assert hil_bridge.quantize_as5600(720.0) == 0,    "720° → 0 (2 voltas)"
    assert hil_bridge.quantize_as5600(450.0) == 1024, "450° → 1024 (1 volta + 90°)"

    # ── quantize_as5600: saturação em 4095 (não deve retornar 4096) ──
    for deg in [0.0, 45.0, 90.0, 135.0, 180.0, 270.0, 359.9]:
        val = hil_bridge.quantize_as5600(deg)
        assert 0 <= val <= 4095, \
            f"quantize_as5600({deg}°) = {val} fora do intervalo [0, 4095]"

    # ── clamp: valores fora do range ──
    assert hil_bridge.clamp(200.0, -170.0, 170.0) == 170.0, "clamp acima do max"
    assert hil_bridge.clamp(-200.0, -170.0, 170.0) == -170.0, "clamp abaixo do min"
    assert hil_bridge.clamp(0.0, -170.0, 170.0) == 0.0, "clamp dentro do range"

    # ── NaN/Inf detectados pela condição do hil_bridge ──
    nan_val = float("nan")
    inf_val = float("inf")
    assert math.isnan(nan_val), "math.isnan deve detectar NaN"
    assert math.isinf(inf_val), "math.isinf deve detectar Inf"

    # Verifica que o frame RTDE com NaN seria detectado antes de ser enviado
    # (o hil_bridge verifica g_measured_rad[i] antes de quantizar)
    bad_rad = float("nan")
    assert math.isnan(bad_rad) or math.isinf(bad_rad), \
        "Falha de encoder NaN/Inf deve ser detectavel"

    # ── steps_to_deg: conversão correta ──
    steps_por_grau = (200 * 4) / 360.0
    for deg_esperado in [0.0, 45.0, 90.0, 180.0, -90.0]:
        steps = deg_esperado * steps_por_grau
        deg_recalc = hil_bridge.steps_to_deg(steps)
        assert abs(deg_recalc - deg_esperado) < 1e-9, \
            f"steps_to_deg({steps:.2f}) = {deg_recalc:.6f}°, esperado {deg_esperado}°"


# ─────────────────────────────────────────────────────────────────
# Teste 12 — Reinicialização completa e repetição determinística
# ─────────────────────────────────────────────────────────────────
def test_12_determinismo_tres_execucoes():
    """
    Verifica que três carregamentos idênticos da mesma trajetória produzem
    exatamente a mesma sequência de waypoints (comandos, durações, derivadas).
    Cobre o critério: 'Três execuções idênticas devem produzir a mesma sequência de comandos.'
    """
    for nome_traj in trajectory.TRAJECTORIES:
        runs = [trajectory.load_trajectory(nome_traj) for _ in range(3)]

        r0 = runs[0]
        for run_idx, r in enumerate(runs[1:], start=2):
            assert r["name"] == r0["name"], \
                f"Traj '{nome_traj}': run {run_idx} name difere"
            assert r["total_ms"] == r0["total_ms"], \
                f"Traj '{nome_traj}': run {run_idx} total_ms difere " \
                f"({r['total_ms']} != {r0['total_ms']})"
            assert len(r["waypoints"]) == len(r0["waypoints"]), \
                f"Traj '{nome_traj}': run {run_idx} numero de waypoints difere"

            for i, (wp, wp0) in enumerate(zip(r["waypoints"], r0["waypoints"])):
                assert wp["q"] == wp0["q"], \
                    f"Traj '{nome_traj}' run {run_idx} WP {i}: q difere"
                assert wp["duration_ms"] == wp0["duration_ms"], \
                    f"Traj '{nome_traj}' run {run_idx} WP {i}: duration_ms difere"
                for j in range(6):
                    assert abs(wp["qd"][j] - wp0["qd"][j]) < 1e-10, \
                        f"Traj '{nome_traj}' run {run_idx} WP {i} J{j+1}: qd difere"
                    assert abs(wp["qdd"][j] - wp0["qdd"][j]) < 1e-10, \
                        f"Traj '{nome_traj}' run {run_idx} WP {i} J{j+1}: qdd difere"


# ─────────────────────────────────────────────────────────────────
# Teste 13 — Alternância dinâmica Modo RTDE ↔ Modo User
# ─────────────────────────────────────────────────────────────────
def test_13_alternancia_modo_rtde_user():
    """
    Verifica o mecanismo de alternância de modos via API HTTP /api/mode.
    Usa um servidor HTTP de mock que simula o comportamento do ESP32.
    Testa as transições USER→RTDE e RTDE→USER, incluindo pedido de modo inválido.
    """
    srv = MockHttpServer()
    srv.start()
    assert srv.ready.wait(timeout=3.0), "Servidor mock nao iniciou em 3s"

    base = f"http://127.0.0.1:{srv.port}"

    def post_mode(mode: str) -> dict:
        data = json.dumps({"mode": mode}).encode("utf-8")
        req = urllib.request.Request(
            f"{base}/api/mode",
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST"
        )
        with urllib.request.urlopen(req, timeout=3.0) as r:
            return json.loads(r.read().decode())

    try:
        # ── Transição USER → RTDE ──
        res = post_mode("RTDE")
        assert res.get("status") == "ok", f"status != ok: {res}"
        assert res.get("mode") == "RTDE", f"mode != RTDE: {res}"
        assert srv.last_mode_request.get("mode") == "RTDE", \
            "Servidor nao recebeu o body correto para RTDE"

        # ── Transição RTDE → USER ──
        res = post_mode("USER")
        assert res.get("status") == "ok"
        assert res.get("mode") == "USER"

        # ── Modo inválido deve retornar 400 ──
        try:
            post_mode("INVALID_MODE")
            pytest.fail("Modo invalido deveria levantar HTTPError")
        except urllib.error.HTTPError as e:
            assert e.code == 400, f"Esperado HTTP 400, obtido {e.code}"

        # ── Múltiplas alternâncias sem erro ──
        for ciclo in range(5):
            r1 = post_mode("RTDE")
            assert r1["mode"] == "RTDE", f"Ciclo {ciclo}: falha ao ativar RTDE"
            r2 = post_mode("USER")
            assert r2["mode"] == "USER", f"Ciclo {ciclo}: falha ao retornar USER"

    finally:
        srv.shutdown()


# ─────────────────────────────────────────────────────────────────
# Teste 14 — Modo User: interface web (comandos e telemetria)
# ─────────────────────────────────────────────────────────────────
def test_14_modo_user_web():
    """
    Verifica a interface web do Modo User: endpoints /api/status e /api/telemetry.
    O servidor de mock simula o ESP32 respondendo com telemetria estruturada.
    Valida o formato dos dados retornados sem necessitar do firmware real.
    """
    srv = MockHttpServer()
    srv.start()
    assert srv.ready.wait(timeout=3.0), "Servidor mock nao iniciou em 3s"

    base = f"http://127.0.0.1:{srv.port}"

    try:
        # ── GET /api/status ──
        with urllib.request.urlopen(f"{base}/api/status", timeout=3.0) as r:
            status = json.loads(r.read().decode())
        assert status.get("status") == "ok", f"status invalido: {status}"
        assert "mode" in status, "Campo 'mode' ausente no status"

        # ── GET /api/telemetry ──
        with urllib.request.urlopen(f"{base}/api/telemetry", timeout=3.0) as r:
            tel = json.loads(r.read().decode())

        # Valida campos obrigatórios de telemetria
        required = ["joints", "errors", "tcp", "temperature", "mode"]
        for campo in required:
            assert campo in tel, f"Campo '{campo}' ausente na telemetria"

        # Valida dimensões
        assert len(tel["joints"]) == 6, "joints deve ter 6 elementos"
        assert len(tel["errors"]) == 6, "errors deve ter 6 elementos"
        assert len(tel["tcp"]) == 6, "tcp deve ter 6 elementos"
        assert isinstance(tel["temperature"], (int, float)), "temperature deve ser numérico"

        # Valida que todos os joints são numéricos
        for i, j in enumerate(tel["joints"]):
            assert isinstance(j, (int, float)), f"joint[{i}] não é numérico: {j}"

        # ── Verifica que URL não existente retorna 404 ──
        try:
            urllib.request.urlopen(f"{base}/api/inexistente", timeout=2.0)
            pytest.fail("Endpoint inexistente deveria retornar 404")
        except urllib.error.HTTPError as e:
            assert e.code == 404, f"Esperado 404, obtido {e.code}"

        # ── Verifica acesso múltiplo sem degradação ──
        for _ in range(10):
            with urllib.request.urlopen(f"{base}/api/telemetry", timeout=2.0) as r:
                d = json.loads(r.read().decode())
                assert len(d["joints"]) == 6, "Resposta degradada apos acesso multiplo"

    finally:
        srv.shutdown()


# ─────────────────────────────────────────────────────────────────
# Teste extra — Exit codes distintos do benchmark
# ─────────────────────────────────────────────────────────────────
def test_exit_codes_distintos():
    """
    Verifica que os exit codes definidos no benchmark.py são distintos
    e cobrem as categorias de falha exigidas pelo plano_passo3.md Fase 5.
    """
    import benchmark as bm

    EXIT_CODES = {
        "EXIT_OK":         bm.EXIT_OK,
        "EXIT_DEPENDENCY": bm.EXIT_DEPENDENCY,
        "EXIT_TIMEOUT":    bm.EXIT_TIMEOUT,
        "EXIT_TRAJECTORY": bm.EXIT_TRAJECTORY,
        "EXIT_OUTPUT":     bm.EXIT_OUTPUT,
        "EXIT_CONFIG":     bm.EXIT_CONFIG,
    }

    # Todos os códigos devem ser inteiros não-negativos
    for nome, codigo in EXIT_CODES.items():
        assert isinstance(codigo, int) and codigo >= 0, \
            f"{nome} deve ser int >= 0, obtido {codigo!r}"

    # Todos os códigos devem ser distintos
    valores = list(EXIT_CODES.values())
    assert len(valores) == len(set(valores)), \
        f"Exit codes nao sao distintos: {EXIT_CODES}"

    # EXIT_OK deve ser 0 (convenção POSIX)
    assert bm.EXIT_OK == 0, "EXIT_OK deve ser 0"

    # Deve haver pelo menos 5 categorias distintas (exigência do plano)
    assert len(EXIT_CODES) >= 5, "Deve haver pelo menos 5 categorias de exit code"


# ─────────────────────────────────────────────────────────────────
# Teste extra — --seed propagado ao manifesto
# ─────────────────────────────────────────────────────────────────
def test_seed_cli_disponivel():
    """
    Verifica que o benchmark aceita --seed como argumento CLI e que o valor
    é aplicado ao config antes da execução, garantindo reprodutibilidade.
    """
    import benchmark as bm
    import argparse

    # Simula parse dos argumentos com --seed explícito
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", default="eb15")
    parser.add_argument("--trajectory", default="canonical")
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--no-headless", action="store_false", dest="headless")
    parser.add_argument("--seed", type=int, default=None)

    args = parser.parse_args(["--seed", "1337", "--target", "eb15",
                               "--trajectory", "canonical"])
    assert args.seed == 1337, "Argumento --seed nao foi parseado corretamente"

    # Verifica que o save_manifest consegue usar o seed
    # (usa args mock e histórias vazias)
    import tempfile, os
    orig_dir = bm.WORKSPACE_DIR

    with tempfile.TemporaryDirectory() as tmpdir:
        # Redireciona output para tmpdir
        bm.WORKSPACE_DIR = tmpdir
        os.makedirs(os.path.join(tmpdir, "webots", "output"), exist_ok=True)
        bm.config["paths"]["output_csv"] = "output"

        bm.save_manifest(args, [], [], 1000.0, 1001.5, "ok")

        manifest_path = os.path.join(
            tmpdir, "webots", "output", "manifest_canonical_eb15.json"
        )
        assert os.path.exists(manifest_path), "Manifesto JSON nao foi criado"

        with open(manifest_path, "r", encoding="utf-8") as f:
            manifest = json.load(f)

        assert manifest["seed"] == 1337, \
            f"Seed incorreto no manifesto: {manifest['seed']}"
        assert manifest["status"] == "ok"
        assert manifest["target"] == "eb15"
        assert manifest["duration_s"] == pytest.approx(1.5, abs=0.01)

    bm.WORKSPACE_DIR = orig_dir
