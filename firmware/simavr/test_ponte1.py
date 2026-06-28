#!/usr/bin/env python3
"""
test_ponte1.py — Teste de integração da PONTE 1 contra o simavr REAL (Etapa 3).

Valida, SEM QEMU e SEM mock de firmware, que a ponte 1 (ponte1_uart.py) repassa o
handshake até o .ino real no simavr e que o surrogate de trigger inicia o DDA:

  ESP(fake) --frame--> ponte1 --> simavr(.ino real) --ACK--> ponte1 --> ESP(fake)
                                          ^ ponte1 dispara TRIGGER -> simavr gera passos -> DONE

O "ESP fake" é apenas um CLIENTE DE ESTÍMULO (envia o frame e confere ACK/DONE), no papel
que o ESP real terá no QEMU. NÃO simula o firmware do Uno — esse roda de verdade no simavr.

Pré: container eb15-simavr rodando `uno_runner --serve` com portas 30200/30201/30202 publicadas.
Uso: python test_ponte1.py
"""
import socket
import struct
import subprocess
import sys
import threading
import time
import os

QEMU_PORT = 30100   # ESP fake escuta aqui (QEMU é servidor desta porta no real)
STEP_PORT = 30202
HERE = os.path.dirname(os.path.abspath(__file__))
PONTE1 = os.path.join(os.path.dirname(HERE), "qemu", "ponte1_uart.py")

UART_ACK, UART_NAK, UART_DONE = 0x06, 0x15, 0x04


def build_frame(j4, j5, j6, dur, corrupt=False):
    body = struct.pack('<B hhh H', 0xAA, j4, j5, j6, dur)
    xor = 0
    for b in body:
        xor ^= b
    if corrupt:
        xor ^= 0xFF
    return body + bytes([xor])


def main():
    # 1) ESP fake: servidor TCP em :30100 (igual ao QEMU)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', QEMU_PORT))
    srv.listen(1)
    srv.settimeout(30)
    print(f"[test] ESP-fake ouvindo em :{QEMU_PORT}")

    # 2) contador de passos (ponte 3 fará isso de verdade na Etapa 4)
    steps = {'j4': 0, 'j5': 0, 'j6': 0}
    stop = threading.Event()

    def step_counter():
        try:
            s = socket.create_connection(('127.0.0.1', STEP_PORT), timeout=10)
        except OSError as e:
            print(f"[test] AVISO: sem step server: {e}")
            return
        s.settimeout(0.5)
        while not stop.is_set():
            try:
                d = s.recv(256)
            except socket.timeout:
                continue
            except OSError:
                break
            if not d:
                break
            for b in d:
                axis = b & 0x03
                steps[['j4', 'j5', 'j6'][axis]] += 1
        s.close()

    tc = threading.Thread(target=step_counter, daemon=True)
    tc.start()

    # 3) sobe a ponte 1
    proc = subprocess.Popen([sys.executable, PONTE1], cwd=os.path.dirname(PONTE1))

    # 4) aceita a conexão da ponte 1 (no papel do ESP)
    try:
        esp, _ = srv.accept()
    except socket.timeout:
        print("[test] FALHA: ponte 1 nao conectou ao ESP-fake")
        proc.terminate(); return 1
    esp.settimeout(8)
    print("[test] ponte 1 conectou ao ESP-fake")
    time.sleep(0.5)

    fails = 0
    rx = bytearray()

    def expect_byte(want, timeout, label):
        nonlocal rx
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                d = esp.recv(64)
            except socket.timeout:
                continue
            except OSError:
                break
            if d:
                rx.extend(d)
                if want in d:
                    print(f"[test]   {label}: recebido 0x{want:02X} OK")
                    return True
        print(f"[test]   {label}: 0x{want:02X} NAO recebido (rx={bytes(rx).hex()})")
        return False

    # --- teste NAK (checksum invalido) ---
    print("[test] === NAK ===")
    esp.sendall(build_frame(5, 0, 0, 80, corrupt=True))
    if not expect_byte(UART_NAK, 4, "NAK"):
        fails += 1

    # --- teste ACK + trigger + passos + DONE ---
    print("[test] === ACK + trigger + passos + DONE ===")
    s0 = dict(steps)
    esp.sendall(build_frame(5, 0, 0, 80, corrupt=False))
    if not expect_byte(UART_ACK, 4, "ACK"):
        fails += 1
    # a ponte 1 dispara o trigger ao ver o ACK; aguarda DONE
    if not expect_byte(UART_DONE, 6, "DONE"):
        fails += 1
    time.sleep(0.3)
    dj4 = steps['j4'] - s0['j4']
    print(f"[test]   passos J4={dj4} (esperado 5)")
    if dj4 != 5:
        print("[test]   passos divergentes"); fails += 1

    stop.set()
    esp.close(); srv.close()
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()

    print("=" * 40)
    print(f"[test] RESULT: {'PASS' if fails == 0 else 'FAIL'} ({fails} falha(s))")
    return 0 if fails == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
