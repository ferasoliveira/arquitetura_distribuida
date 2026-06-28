# -*- coding: utf-8 -*-
"""
rtde_eb15.py — Cliente/Servidor RTDE-EB15 para o Adaptador Geral de Benchmark
===============================================================================
TCC Fernando Oliveira — Passo 3: Ambiente Virtual, Pontes e Simulação

Arquitetura:
  - Modo padrão (server_mode=False): Python conecta ao ESP32-S3 como CLIENTE TCP.
    Usado com QEMU (qemu-system-xtensa -machine esp32s3), onde o firmware é o
    SERVIDOR RTDE na porta 30003, e o hostfwd do QEMU encaminha localhost:30003.
  - Modo legado (server_mode=True): PYTHON é o SERVIDOR; o ESP32 conecta como
    cliente. Mantido para compatibilidade com os testes de integração unitários.
"""

import socket
import struct
import threading
import time
import logging
from typing import Optional, Dict, Any, List

# Adiciona o caminho dos helpers para importar protocol
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "helpers")))
import protocol

logger = logging.getLogger("rtde_eb15")

# Tamanho do frame binário (13 floats × 4 bytes = 52 bytes)
RTDE_FRAME_SIZE = 52


class RtdeEb15Client:
    """
    Interface RTDE-EB15 — suporta dois modos:

    server_mode=False (padrão, QEMU/hardware real):
        Python conecta ao ESP32 como cliente TCP.
        Com QEMU, o firmware é o servidor RTDE; o hostfwd encaminha a porta.

    server_mode=True (legado, testes unitários):
        Python escuta como servidor TCP; o mock de ESP32 conecta como cliente.
        Mantido apenas para os testes de integração (MockEsp32).
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 30003,
                 server_mode: bool = False):
        self.host = host
        self.port = port
        self.server_mode = server_mode
        self._server_sock: Optional[socket.socket] = None
        self.socket: Optional[socket.socket] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.latest_telemetry: Dict[str, Any] = {}
        self.telemetry_lock = threading.Lock()
        self.telemetry_history: List[Dict[str, Any]] = []
        self.history_lock = threading.Lock()
        self._connected_event = threading.Event()
        self._send_lock = threading.Lock()
        self._sequence = 0
        self._heartbeat_thread: Optional[threading.Thread] = None

    def connect(self, timeout: float = 60.0) -> bool:
        """
        Inicia a conexão RTDE.

        - server_mode=True:  escuta na porta e aguarda o ESP32 mock conectar (até `timeout`s).
        - server_mode=False: conecta ao ESP32 hardware real como cliente (timeout em segundos).
        """
        if self.server_mode:
            return self._start_server(timeout)
        else:
            return self._connect_client(timeout)

    def _start_server(self, accept_timeout: float) -> bool:
        """Abre servidor TCP e aguarda o ESP32 QEMU conectar.
        
        Se self._server_sock já estiver definido (pré-criado em main()),
        reutiliza o socket existente em vez de criar um novo.
        """
        try:
            if self._server_sock is None:
                # Cria e associa novo socket servidor
                self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._server_sock.bind((self.host, self.port))
                self._server_sock.listen(1)
                logger.info("Servidor RTDE-EB15 criado e associado à porta %d.", self.port)
            
            self._server_sock.settimeout(accept_timeout)
            logger.info("Aguardando ESP32 QEMU conectar em %s:%d (timeout=%ds)...",
                        self.host, self.port, int(accept_timeout))
            conn, addr = self._server_sock.accept()
            logger.info("ESP32 QEMU conectado ao servidor RTDE de %s!", addr)
            self.socket = conn
            self.socket.settimeout(None)
            self.running = True
            self.thread = threading.Thread(target=self._recv_loop, daemon=True)
            self.thread.start()
            return True
        except socket.timeout:
            logger.error("Timeout: ESP32 QEMU não conectou ao servidor RTDE em %ds.", int(accept_timeout))
            self._cleanup_server()
            return False
        except OSError as e:
            logger.error("Erro ao iniciar servidor RTDE-EB15: %s", e)
            self._cleanup_server()
            return False


    def _connect_client(self, timeout: float) -> bool:
        """Conecta ao ESP32 hardware real como cliente TCP, tentando em loop ate o timeout."""
        start_time = time.time()
        logger.info("Conectando ao RTDE-EB15 em %s:%d (timeout=%ds)...", self.host, self.port, int(timeout))
        while True:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(1.0)
                self.socket.connect((self.host, self.port))
                self.socket.settimeout(None)
                self.running = True
                self.thread = threading.Thread(target=self._recv_loop, daemon=True)
                self.thread.start()
                self._start_heartbeat()
                logger.info("Conectado ao RTDE-EB15.")
                return True
            except OSError as e:
                if self.socket:
                    try:
                        self.socket.close()
                    except OSError:
                        pass
                    self.socket = None
                if time.time() - start_time >= timeout:
                    logger.error("Falha ao conectar ao RTDE-EB15 apos %ds: %s", int(timeout), e)
                    return False
                time.sleep(0.5)

    def _cleanup_server(self):
        """Fecha o socket do servidor."""
        if self._server_sock:
            try:
                self._server_sock.close()
            except OSError:
                pass
            self._server_sock = None

    def disconnect(self):
        """Desconecta e encerra todas as threads e sockets."""
        self.running = False
        if self.socket:
            try:
                self.socket.close()
            except OSError:
                pass
            self.socket = None
        self._cleanup_server()
        if self.thread:
            self.thread.join(timeout=2.0)
            self.thread = None
        if self._heartbeat_thread:
            self._heartbeat_thread.join(timeout=1.0)
            self._heartbeat_thread = None
        logger.info("RTDE-EB15 desconectado.")

    def _send_command(self, command_type: int, joints=None, sequence: int = 0) -> bool:
        if not self.socket or not self.running:
            return False
        frame = protocol.encode_rtde_command(
            command_type,
            sequence=sequence,
            planned_time_ms=int(time.monotonic() * 1000),
            joints=joints,
        )
        try:
            with self._send_lock:
                self.socket.sendall(frame)
            return True
        except OSError as e:
            logger.error("Erro ao enviar frame RTDE-EB15: %s", e)
            return False

    def _start_heartbeat(self):
        def heartbeat_loop():
            while self.running:
                self._send_command(protocol.RTDE_CMD_HEARTBEAT)
                time.sleep(0.2)

        self._heartbeat_thread = threading.Thread(target=heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()

    def _recv_loop(self):
        """Loop de recepção contínua da telemetria (52 bytes por frame)."""
        buffer = b""
        while self.running and self.socket:
            try:
                chunk = self.socket.recv(RTDE_FRAME_SIZE - len(buffer))
                if not chunk:
                    logger.warning("Conexão RTDE-EB15 fechada pelo ESP32.")
                    break
                buffer += chunk

                if len(buffer) == RTDE_FRAME_SIZE:
                    telemetry = protocol.decode_rtde_frame(buffer)
                    telemetry["timestamp"] = time.time()

                    with self.telemetry_lock:
                        self.latest_telemetry = telemetry

                    with self.history_lock:
                        self.telemetry_history.append(telemetry)

                    buffer = b""
            except OSError as e:
                if self.running:
                    logger.error("Erro de leitura no RTDE-EB15: %s", e)
                break
        self.running = False

    def get_latest_telemetry(self) -> Dict[str, Any]:
        """Retorna a última telemetria recebida (thread-safe)."""
        with self.telemetry_lock:
            return dict(self.latest_telemetry)

    def get_history(self) -> List[Dict[str, Any]]:
        """Retorna e limpa o histórico de telemetria coletada."""
        with self.history_lock:
            hist = list(self.telemetry_history)
            self.telemetry_history.clear()
            return hist

    def send_movej(self, joints: List[float], speed_pct: float) -> bool:
        """Envia comando de juntas (movej) no formato URScript compatível."""
        self._sequence += 1
        sent = self._send_command(protocol.RTDE_CMD_MOVE_J, joints, self._sequence)
        if sent:
            logger.info("Comando MOVE_J binario enviado: seq=%d target=%s speed=%.1f%%",
                        self._sequence, joints, speed_pct)
        return sent

    def send_movel(self, pose: List[float], speed_pct: float) -> bool:
        """Envia comando linear cartesiano (movel)."""
        if not self.socket or not self.running:
            return False
        pose_str = ", ".join(f"{p:.4f}" for p in pose)
        cmd = f"movel([{pose_str}], v={speed_pct:.1f})\n"
        try:
            self.socket.sendall(cmd.encode("utf-8"))
            logger.info("Comando movel enviado: %s", cmd.strip())
            return True
        except OSError as e:
            logger.error("Erro ao enviar comando movel: %s", e)
            return False

    def send_stop(self) -> bool:
        """Envia comando de parada de emergência (stop)."""
        if not self.socket or not self.running:
            return False
        cmd = "stop\n"
        try:
            self.socket.sendall(cmd.encode("utf-8"))
            logger.warning("Comando STOP enviado via RTDE-EB15.")
            return True
        except OSError as e:
            logger.error("Erro ao enviar comando stop: %s", e)
            return False
