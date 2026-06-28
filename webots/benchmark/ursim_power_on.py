# -*- coding: utf-8 -*-
"""
ursim_power_on.py — liga o robô do URSim via Dashboard Server (:29999).
Sequência oficial: power on -> aguardar IDLE -> brake release -> RUNNING.
Uso: python ursim_power_on.py [host] [port]
"""
import socket
import sys
import time


def dash(host, port, cmd, wait=0.5):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5.0)
    s.connect((host, port))
    banner = s.recv(4096)  # "Connected: Universal Robots Dashboard Server"
    s.sendall((cmd + "\n").encode())
    time.sleep(wait)
    try:
        resp = s.recv(4096).decode(errors="replace").strip()
    except socket.timeout:
        resp = "(timeout)"
    s.close()
    return resp


def robotmode(host, port):
    return dash(host, port, "robotmode")


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 29999

    # Espera o dashboard aceitar conexão (boot do container).
    deadline = time.time() + 120
    while time.time() < deadline:
        try:
            mode = robotmode(host, port)
            print("robotmode:", mode)
            break
        except OSError as e:
            print("aguardando dashboard...", e)
            time.sleep(3)
    else:
        print("ERRO: dashboard nunca respondeu")
        return 1

    # power on
    print("-> power on:", dash(host, port, "power on", wait=1.0))
    # aguarda IDLE
    for _ in range(40):
        mode = robotmode(host, port)
        print("   robotmode:", mode)
        if "IDLE" in mode or "RUNNING" in mode:
            break
        time.sleep(1.5)

    # brake release
    print("-> brake release:", dash(host, port, "brake release", wait=1.0))
    # aguarda RUNNING
    for _ in range(40):
        mode = robotmode(host, port)
        print("   robotmode:", mode)
        if "RUNNING" in mode:
            break
        time.sleep(1.5)

    # modo remoto (necessário para aceitar URScript externo)
    print("-> is in remote control:", dash(host, port, "is in remote control"))
    final = robotmode(host, port)
    safety = dash(host, port, "safetystatus")
    print("FINAL robotmode:", final)
    print("FINAL safetystatus:", safety)
    return 0 if "RUNNING" in final else 2


if __name__ == "__main__":
    sys.exit(main())
