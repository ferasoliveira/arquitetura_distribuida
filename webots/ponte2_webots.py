"""Ponte elétrica 2: ESP32-S3 (QEMU) <-> Webots, juntas J1-J3.

Relé puro: consulta o Step/Dir real exportado pelo firmware em :30101,
entrega o comando ao controlador Webots em :30301 e serve o ground truth
recebido do Webots ao ESP como 3 x AS5600 em :30103.
"""
import argparse
import logging
import os
import socket
import socketserver
import struct
import threading
import time

LOG = logging.getLogger("ponte2")
state_lock = threading.Lock()
command_deg = [0.0, 0.0, 0.0]
encoder_raw = [0, 0, 0]
encoder_valid = False

# ===== Co-simulação SÍNCRONA (lockstep) =====
# Com EB15_LOCKSTEP=1, cada passo do Webots dispara UMA troca síncrona com o ESP:
# o handler do Webots recebe o encoder medido, REPASSA ao endpoint :30101 do ESP
# (que roda 1 ciclo de controle e devolve o comando) e retorna o comando ao Webots.
# Assim o controle de J1-J3 corre 1x por passo do Webots, com feedback sempre fresco
# (sem o poll assíncrono + leitura de encoder separada do modo livre).
LOCKSTEP = os.environ.get("EB15_LOCKSTEP", "0") == "1"
_esp_host = "127.0.0.1"
_esp_port = 30101
_esp_sock = None
_esp_lock = threading.Lock()


def recv_exact(sock: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("conexao encerrada")
        data.extend(chunk)
    return bytes(data)


def poll_esp(host: str, port: int, stop: threading.Event) -> None:
    # ONE-SHOT por poll. Persistir este canal (testado) DEGRADA o laço de J1-J3
    # (comando 1-ciclo-stale → timeouts); mantido connect/close por estabilidade.
    global command_deg
    while not stop.is_set():
        try:
            with socket.create_connection((host, port), timeout=0.1) as sock:
                sock.settimeout(0.1)
                sock.sendall(b"\x01")
                values = struct.unpack("<6f", recv_exact(sock, 24))
            with state_lock:
                command_deg = list(values[3:6])
        except (OSError, ConnectionError, struct.error):
            pass
        # 5 ms (200 Hz) — DEVE casar com o PID de 200 Hz do ESP. Testei 20 ms (50 Hz):
        # o comando para o Webots fica grosso, a atuação atrasa frente ao alvo do
        # segmento e o PID amplifica → drift (8/18 vs 15/18). Manter 200 Hz.
        stop.wait(0.005)


def esp_exchange(measured: bytes) -> list:
    """LOCKSTEP: envia o encoder medido das 6 juntas (12 bytes, 6x uint16) ao endpoint
    :30101 do ESP e recebe o comando das 6 juntas (6 floats em GRAUS: J1-J3 do ESP,
    J4-J6 do Uno sub-mestrado pelo ESP). Conexão persistente (reconecta em falha).
    Síncrono → sem staleness (a resposta é o comando DAQUELE medido)."""
    global _esp_sock
    with _esp_lock:
        if _esp_sock is None:
            _esp_sock = socket.create_connection((_esp_host, _esp_port), timeout=5.0)
            _esp_sock.settimeout(5.0)
            _esp_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            _esp_sock.sendall(measured)
            return list(struct.unpack("<6f", recv_exact(_esp_sock, 24)))
        except (OSError, ConnectionError, struct.error):
            try:
                _esp_sock.close()
            except OSError:
                pass
            _esp_sock = None
            raise


def make_webots_handler():
    class Handler(socketserver.BaseRequestHandler):
        def handle(self):
            # Conexão PERSISTENTE: o controlador Webots reusa o socket e troca a cada
            # timestep (5 ms). Em LOCKSTEP, cada troca dispara um ciclo de controle no
            # ESP (repasse síncrono a :30101) e devolve o comando fresco.
            global encoder_raw, encoder_valid, command_deg
            self.request.settimeout(5.0)
            try:
                self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass
            while True:
                try:
                    measured_6 = recv_exact(self.request, 6)
                except (ConnectionError, OSError):
                    break
                with state_lock:
                    encoder_raw = [v & 0x0FFF for v in struct.unpack("<3H", measured_6)]
                    encoder_valid = True
                    cmd = list(command_deg)
                if LOCKSTEP:
                    try:
                        cmd = esp_exchange(measured_6)[3:6]  # graus comandados (J1-J3)
                        with state_lock:
                            command_deg = list(cmd)
                    except (OSError, ConnectionError, struct.error):
                        pass  # mantém último comando em falha de transporte
                try:
                    self.request.sendall(struct.pack("<3f", *cmd))
                except OSError:
                    break
    return Handler


def make_encoder_handler():
    class Handler(socketserver.BaseRequestHandler):
        def handle(self):
            # J1-J3: handler ONE-SHOT. O ESP lê este canal por connect/close — torná-lo
            # persistente faz a leitura ficar stale e o PID J1-J3 disparar (runaway);
            # mantido one-shot por segurança do controle de malha fechada.
            self.request.settimeout(0.1)
            recv_exact(self.request, 1)
            with state_lock:
                if not encoder_valid:
                    return
                raw = list(encoder_raw)
            self.request.sendall(struct.pack("<3H", *raw))
    return Handler


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--esp-step-port", type=int, default=30101)
    parser.add_argument("--esp-encoder-port", type=int, default=30103)
    parser.add_argument("--webots-port", type=int, default=30301)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [ponte2] %(levelname)s %(message)s")
    global _esp_host, _esp_port
    _esp_host, _esp_port = args.host, args.esp_step_port
    stop = threading.Event()
    # O servidor de encoder (:30103) é sempre aberto (o health check do benchmark o usa
    # como prova de vida). Em LOCKSTEP o ESP NÃO se conecta a ele (recebe o medido na
    # troca síncrona de comando); fica ocioso. Só o poll assíncrono é desligado.
    servers = [
        Server((args.host, args.webots_port), make_webots_handler()),
        Server((args.host, args.esp_encoder_port), make_encoder_handler()),
    ]
    if LOCKSTEP:
        LOG.info("[READY] LOCKSTEP: Webots :%d <-> ESP :%d (síncrono); encoder :%d (ocioso)",
                 args.webots_port, args.esp_step_port, args.esp_encoder_port)
    else:
        threading.Thread(target=poll_esp, args=(args.host, args.esp_step_port, stop), daemon=True).start()
        LOG.info("[READY] ESP :%d <-> Webots :%d; encoder :%d",
                 args.esp_step_port, args.webots_port, args.esp_encoder_port)
    for server in servers:
        threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop.set()
        for server in servers:
            server.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
