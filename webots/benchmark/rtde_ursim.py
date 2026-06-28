# -*- coding: utf-8 -*-
"""
rtde_ursim.py — Cliente RTDE-URSim para o Adaptador Geral de Benchmark
=====================================================================
TCC Fernando Oliveira — Passo 3: Ambiente Virtual, Pontes e Simulação
"""

import socket
import struct
import threading
import time
import math
import logging
from typing import Optional, Dict, Any, List

logger = logging.getLogger("rtde_ursim")

class RtdeUrsimClient:
    """Cliente para o simulador oficial URSim utilizando RTDE (30004) e Script (30003)."""

    def __init__(self, host: str = "127.0.0.1", rtde_port: int = 30004, script_port: int = 30003):
        self.host = host
        self.rtde_port = rtde_port
        self.script_port = script_port
        self.rtde_socket: Optional[socket.socket] = None
        self.script_socket: Optional[socket.socket] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        self.latest_telemetry: Dict[str, Any] = {}
        self.telemetry_lock = threading.Lock()
        self.telemetry_history: List[Dict[str, Any]] = []
        self.history_lock = threading.Lock()
        self.recipe_id: int = 0

    def connect(self, timeout: float = 5.0) -> bool:
        """Conecta ao RTDE (30004) e Script (30003) do URSim."""
        try:
            logger.info("Conectando ao URSim RTDE em %s:%d...", self.host, self.rtde_port)
            self.rtde_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.rtde_socket.settimeout(timeout)
            self.rtde_socket.connect((self.host, self.rtde_port))
            
            logger.info("Conectando ao URSim Script em %s:%d...", self.host, self.script_port)
            self.script_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.script_socket.settimeout(timeout)
            self.script_socket.connect((self.host, self.script_port))
            self.script_socket.settimeout(None) # bloqueante
            
            # Inicializa protocolo RTDE
            if not self._negotiate_rtde():
                raise OSError("Falha na negociação do protocolo RTDE")
                
            self.running = True
            self.thread = threading.Thread(target=self._recv_loop, daemon=True)
            self.thread.start()
            logger.info("Conectado ao URSim com sucesso.")
            return True
        except OSError as e:
            logger.error("Falha ao conectar ao URSim: %s", e)
            self.disconnect()
            return False

    def disconnect(self):
        """Desconecta do URSim."""
        self.running = False
        if self.rtde_socket:
            try:
                self.rtde_socket.close()
            except OSError:
                pass
            self.rtde_socket = None
        if self.script_socket:
            try:
                self.script_socket.close()
            except OSError:
                pass
            self.script_socket = None
        if self.thread:
            self.thread.join(timeout=1.0)
            self.thread = None
        logger.info("Desconectado do URSim.")

    def _send_rtde_packet(self, ptype: int, payload: bytes):
        """Envia um pacote RTDE estruturado."""
        # [length (2B)][type (1B)][payload]
        length = len(payload) + 3
        header = struct.pack("!HB", length, ptype)
        self.rtde_socket.sendall(header + payload)

    def _recv_rtde_packet(self) -> tuple:
        """Recebe um pacote RTDE estruturado."""
        header = self.rtde_socket.recv(3)
        if len(header) < 3:
            return 0, b""
        length, ptype = struct.unpack("!HB", header)
        payload_len = length - 3
        payload = b""
        while len(payload) < payload_len:
            chunk = self.rtde_socket.recv(payload_len - len(payload))
            if not chunk:
                break
            payload += chunk
        return ptype, payload

    # Códigos de mensagem do protocolo RTDE oficial da Universal Robots (ASCII).
    RTDE_REQUEST_PROTOCOL_VERSION = 86   # 'V'
    RTDE_TEXT_MESSAGE             = 77   # 'M'
    RTDE_DATA_PACKAGE            = 85   # 'U'
    RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79  # 'O'
    RTDE_CONTROL_PACKAGE_START   = 83   # 'S'

    def _recv_until(self, expected_type: int, timeout: float = 5.0):
        """Lê pacotes ignorando TEXT_MESSAGE (77) até obter o tipo esperado."""
        self.rtde_socket.settimeout(timeout)
        deadline = time.time() + timeout
        while time.time() < deadline:
            ptype, payload = self._recv_rtde_packet()
            if ptype == 0:
                return 0, b""
            if ptype == self.RTDE_TEXT_MESSAGE:
                logger.info("URSim RTDE text: %s", payload[1:].decode(errors="replace"))
                continue
            return ptype, payload
        return 0, b""

    def _negotiate_rtde(self) -> bool:
        """Handshake RTDE conforme o protocolo oficial da Universal Robots (v2)."""
        # 1. Request Protocol Version ('V' = 86), payload = uint16(2)
        self._send_rtde_packet(self.RTDE_REQUEST_PROTOCOL_VERSION, struct.pack(">H", 2))
        ptype, res = self._recv_until(self.RTDE_REQUEST_PROTOCOL_VERSION)
        if ptype != self.RTDE_REQUEST_PROTOCOL_VERSION or not res or res[0] != 1:
            logger.error("URSim RTDE rejeitou a versão do protocolo (ptype=%s)", ptype)
            return False

        # 2. Setup Outputs ('O' = 79). No protocolo v2 o payload começa com a
        #    frequência de saída (double) seguida da lista de variáveis.
        freq = 125.0
        recipe = "timestamp,actual_q,actual_qd,actual_TCP_pose,robot_mode,safety_mode"
        self._send_rtde_packet(self.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS,
                               struct.pack(">d", freq) + recipe.encode("utf-8"))
        ptype, res = self._recv_until(self.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS)
        if ptype != self.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS or len(res) < 2:
            logger.error("URSim RTDE rejeitou a receita de outputs (ptype=%s)", ptype)
            return False
        self.recipe_id = res[0]
        types = res[1:].decode(errors="replace")
        if "NOT_FOUND" in types:
            logger.error("URSim RTDE: variável inexistente na receita: %s", types)
            return False
        logger.info("URSim RTDE recipe_id=%d tipos=%s", self.recipe_id, types)

        # 3. Start ('S' = 83)
        self._send_rtde_packet(self.RTDE_CONTROL_PACKAGE_START, b"")
        ptype, res = self._recv_until(self.RTDE_CONTROL_PACKAGE_START)
        if ptype != self.RTDE_CONTROL_PACKAGE_START or not res or res[0] != 1:
            logger.error("URSim RTDE rejeitou o comando START (ptype=%s)", ptype)
            return False

        logger.info("URSim RTDE configurado e iniciado com sucesso.")
        return True

    def _recv_loop(self):
        """Loop de recepção da telemetria do URSim."""
        self.rtde_socket.settimeout(1.0)
        while self.running and self.rtde_socket:
            try:
                ptype, payload = self._recv_rtde_packet()
                if ptype == 0:
                    break
                
                # Pacote de dados RTDE ('U' = 85)
                if ptype == self.RTDE_DATA_PACKAGE:
                    recipe_id = payload[0]
                    # Unpack: 1 byte (recipe_id) + timestamp (d) + actual_q (6d) + actual_qd (6d) + TCP (6d) + robot_mode (i) + safety_mode (i)
                    # total: 1 + 8 + 48 + 48 + 48 + 4 + 4 = 161 bytes payload
                    if len(payload) >= 161:
                        data = struct.unpack("!d 6d 6d 6d i i", payload[1:161])
                        
                        timestamp = data[0]
                        # Converte radianos para graus para juntas
                        joints_deg = [math.degrees(q) for q in data[1:7]]
                        errors = [0.0] * 6 # URSim não expõe erro PID diretamente
                        
                        # Converte TCP de metros para mm
                        tcp = list(data[13:19])
                        tcp[0] *= 1000.0
                        tcp[1] *= 1000.0
                        tcp[2] *= 1000.0
                        # rx, ry, rz são em rad, convertemos para graus
                        tcp[3] = math.degrees(tcp[3])
                        tcp[4] = math.degrees(tcp[4])
                        tcp[5] = math.degrees(tcp[5])
                        
                        telemetry = {
                            "joints": joints_deg,
                            "errors": errors,
                            "tcp": tcp,
                            "temperature": 35.0, # Dummy
                            "timestamp": time.time(),
                            "ursim_time": timestamp,
                            "robot_mode": data[19],
                            "safety_mode": data[20]
                        }
                        
                        with self.telemetry_lock:
                            self.latest_telemetry = telemetry
                        
                        with self.history_lock:
                            self.telemetry_history.append(telemetry)
            except socket.timeout:
                continue
            except OSError as e:
                if self.running:
                    logger.error("Erro no laço RTDE URSim: %s", e)
                break
        self.running = False

    def get_latest_telemetry(self) -> Dict[str, Any]:
        """Retorna a última telemetria recebida do URSim."""
        with self.telemetry_lock:
            return dict(self.latest_telemetry)

    def get_history(self) -> List[Dict[str, Any]]:
        """Retorna e limpa o histórico de telemetria do URSim."""
        with self.history_lock:
            hist = list(self.telemetry_history)
            self.telemetry_history.clear()
            return hist

    def send_movej(self, joints: List[float], speed_pct: float) -> bool:
        """Envia comando movej em URScript (converte graus para radianos)."""
        if not self.script_socket:
            return False
            
        # Converte graus para radianos
        joints_rad = [math.radians(q) for q in joints]
        joints_str = ", ".join(f"{q:.6f}" for q in joints_rad)
        
        # Converte velocidade para rad/s (assumindo velocidade máxima nominal de 1.57 rad/s)
        eff_speed = 1.57 * (speed_pct / 100.0)
        eff_accel = 1.4
        
        # URScript command
        cmd = f"movej([{joints_str}], a={eff_accel:.2f}, v={eff_speed:.2f})\n"
        
        try:
            self.script_socket.sendall(cmd.encode("utf-8"))
            logger.info("Script URSim enviado: %s", cmd.strip())
            return True
        except OSError as e:
            logger.error("Erro ao enviar comando para o URSim: %s", e)
            return False

    def send_movel(self, pose: List[float], speed_pct: float) -> bool:
        """Envia comando movel em URScript (pose cartesiana [x,y,z,rx,ry,rz] em metros/radianos)."""
        if not self.script_socket:
            return False

        # pose em metros (x,y,z) e radianos (rx,ry,rz)
        pose_str = ", ".join(f"{p:.6f}" for p in pose)
        eff_speed = 0.5 * (speed_pct / 100.0)   # m/s (50 cm/s max)
        eff_accel = 1.2                            # m/s²

        cmd = f"movel(p[{pose_str}], a={eff_accel:.2f}, v={eff_speed:.3f})\n"
        try:
            self.script_socket.sendall(cmd.encode("utf-8"))
            logger.info("Comando movel URSim enviado: %s", cmd.strip())
            return True
        except OSError as e:
            logger.error("Erro ao enviar movel ao URSim: %s", e)
            return False

    def send_stop(self) -> bool:
        """Envia parada de emergência via URScript."""
        if not self.script_socket:
            return False

        cmd = "stopj(2.0)\n"
        try:
            self.script_socket.sendall(cmd.encode("utf-8"))
            logger.warning("Comando STOP enviado ao URSim.")
            return True
        except OSError as e:
            logger.error("Erro ao enviar stop ao URSim: %s", e)
            return False
