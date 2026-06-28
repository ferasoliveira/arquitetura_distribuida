"""Ponte elétrica 3: Arduino UNO (simavr) <-> Webots, juntas J4-J6.

Consome cada borda Step/Dir real de :30202, integra somente a conversão
elétrica passo->ângulo, troca comando/ground truth com o Webots em :30302,
injeta o ground truth no escravo I2C do simavr em :30203 e o serve ao ESP
em :30104. Não contém trajetória, PID ou resposta de firmware.
"""
import argparse
import logging
import os
import socket
import socketserver
import struct
import threading
import time

LOG = logging.getLogger("ponte3")
STEPS_PER_DEG = (200 * 4 * 1.0) / 360.0
state_lock = threading.Lock()
steps = [0, 0, 0]
encoder_raw = [0, 0, 0]
encoder_valid = False


def recv_exact(sock: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("conexao encerrada")
        data.extend(chunk)
    return bytes(data)


def consume_steps(host: str, port: int, stop: threading.Event) -> None:
    while not stop.is_set():
        try:
            with socket.create_connection((host, port), timeout=0.5) as sock:
                sock.settimeout(0.5)
                LOG.info("STEP conectado ao simavr :%d", port)
                while not stop.is_set():
                    try:
                        rec = recv_exact(sock, 1)[0]
                    except socket.timeout:
                        continue
                    axis = rec & 0x03
                    if axis < 3:
                        with state_lock:
                            steps[axis] += 1 if (rec & 0x10) else -1
        except (OSError, ConnectionError):
            stop.wait(0.2)


def inject_encoder(host: str, port: int, stop: threading.Event) -> None:
    while not stop.is_set():
        try:
            with socket.create_connection((host, port), timeout=0.5) as sock:
                sock.settimeout(0.5)
                LOG.info("ENC conectado ao simavr :%d", port)
                while not stop.is_set():
                    with state_lock:
                        valid = encoder_valid
                        raw = list(encoder_raw)
                    if valid:
                        sock.sendall(struct.pack("<3H", *raw))
                    stop.wait(0.005)
        except OSError:
            stop.wait(0.2)


def make_webots_handler():
    class Handler(socketserver.BaseRequestHandler):
        def handle(self):
            # Conexão PERSISTENTE (ver ponte2): serve vários ping/resposta na mesma
            # conexão; o controlador Webots reusa o socket a cada timestep (5 ms),
            # eliminando o connect/close que travava o Webots em ~0,38x tempo real.
            global encoder_raw, encoder_valid
            self.request.settimeout(5.0)
            try:
                self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass
            while True:
                try:
                    raw = struct.unpack("<3H", recv_exact(self.request, 6))
                except (ConnectionError, OSError):
                    break
                with state_lock:
                    encoder_raw = [v & 0x0FFF for v in raw]
                    encoder_valid = True
                    cmd = [value / STEPS_PER_DEG for value in steps]
                try:
                    self.request.sendall(struct.pack("<3f", *cmd))
                except OSError:
                    break
    return Handler


def make_encoder_handler():
    class Handler(socketserver.BaseRequestHandler):
        def handle(self):
            # Conexão PERSISTENTE: serve vários ping/resposta na mesma conexão. O ESP
            # mantém o socket aberto e lê a 50 Hz; connect/close por leitura no NIC
            # emulado do QEMU starvava as leituras de J4-J6 durante o movimento.
            self.request.settimeout(5.0)
            while True:
                try:
                    recv_exact(self.request, 1)  # ping
                except (ConnectionError, OSError):
                    break
                with state_lock:
                    raw = list(encoder_raw) if encoder_valid else [0, 0, 0]
                try:
                    self.request.sendall(struct.pack("<3H", *raw))
                except OSError:
                    break
    return Handler


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def sampler(path: str, stop: threading.Event) -> None:
    """Trace de debug (50 Hz): registra steps[] (saída REAL do Uno) e encoder_raw
    (J4-J6 MEDIDO pelo Webots) com timestamp de parede. Permite ver se a rampa do
    punho está nos passos do Uno, na medida do Webots, ou se perde a jusante."""
    t0 = time.time()
    try:
        f = open(path, "w", encoding="utf-8")
    except OSError:
        return
    f.write("t,steps_j4,steps_j5,steps_j6,deg_j6_cmd,enc_j6_raw,enc_valid\n")
    while not stop.is_set():
        with state_lock:
            s = list(steps)
            e = encoder_raw[2]
            v = 1 if encoder_valid else 0
        f.write("%.4f,%d,%d,%d,%.3f,%d,%d\n" %
                (time.time() - t0, s[0], s[1], s[2], s[2] / STEPS_PER_DEG, e, v))
        f.flush()
        time.sleep(0.02)
    f.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--simavr-step-port", type=int, default=30202)
    parser.add_argument("--simavr-encoder-port", type=int, default=30203)
    parser.add_argument("--esp-encoder-port", type=int, default=30104)
    parser.add_argument("--webots-port", type=int, default=30302)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [ponte3] %(levelname)s %(message)s")
    stop = threading.Event()
    threading.Thread(target=consume_steps, args=(args.host, args.simavr_step_port, stop), daemon=True).start()
    threading.Thread(target=inject_encoder, args=(args.host, args.simavr_encoder_port, stop), daemon=True).start()
    trace_path = os.environ.get("EB15_PONTE3_TRACE")
    if trace_path:
        threading.Thread(target=sampler, args=(trace_path, stop), daemon=True).start()
        LOG.info("trace de debug J4-J6 ativo em %s", trace_path)
    servers = [
        Server((args.host, args.webots_port), make_webots_handler()),
        Server((args.host, args.esp_encoder_port), make_encoder_handler()),
    ]
    for server in servers:
        threading.Thread(target=server.serve_forever, daemon=True).start()
    LOG.info("[READY] simavr STEP :%d / ENC :%d <-> Webots :%d; ESP encoder :%d",
             args.simavr_step_port, args.simavr_encoder_port,
             args.webots_port, args.esp_encoder_port)
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
