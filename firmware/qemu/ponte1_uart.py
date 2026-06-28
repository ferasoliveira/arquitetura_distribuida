#!/usr/bin/env python3
"""
ponte1_uart.py — PONTE 1 da arquitetura canônica v2 (wiki/plano_acao_v2.md).

Relé ELÉTRICO da UART entre o ESP32-S3 (firmware .cpp REAL no QEMU, UART1 em :30100)
e o Arduino UNO (firmware .ino REAL no simavr, UART exposta pelo uno_runner em :30200),
mais a LINHA DE TRIGGER digital (D9/PCINT do Uno), via o canal de controle :30201.

NÃO contém lógica de firmware: apenas repassa bytes nos dois sentidos. A única "regra"
é o surrogate elétrico do trigger — ver abaixo.

TRIGGER (limitação de emulação, declarada):
  No hardware real o ESP pulsa GPIO15 (borda de descida) logo após receber o ACK do Uno,
  por um fio físico até D9. No QEMU o GPIO15 NÃO é exportável para o host, então esta ponte
  reproduz a MESMA causalidade elétrica: ao repassar um ACK (0x06) do Uno para o ESP, dispara
  a borda de descida no D9 do simavr (comando 'F' em :30201). A resposta do Uno (passos, DONE)
  continua 100% real, vinda do simavr. A fidelidade fina de T3/J_start NÃO é capturável por aqui
  (exige VCD por MCU — Etapa 7). Use --no-trigger para desabilitar o surrogate.

Todas as peças são servidores; esta ponte é CLIENTE de ambas.

Uso:
  python ponte1_uart.py [--qemu-host 127.0.0.1] [--qemu-port 30100]
                        [--sim-host 127.0.0.1] [--sim-uart 30200] [--sim-ctrl 30201]
                        [--no-trigger]
"""
import argparse
import os
import socket
import sys
import threading
import time
import logging

UART_ACK = 0x06
UART_NAK = 0x15
UART_BUSY = 0x12
UART_DONE = 0x04
UART_ESTOP = 0x05


class _Flush(logging.StreamHandler):
    def emit(self, r):
        super().emit(r); self.flush()


# Handlers: stderr (sempre) + FileHandler dedicado se EB15_PONTE1_LOG estiver
# definido. O FileHandler torna o log confiavel mesmo quando o stdout/stderr do
# processo e redirecionado por subprocess (a captura via redirect mostrou-se
# intermitente no Windows). O FileHandler faz flush por registro.
_handlers = [_Flush()]
_logpath = os.environ.get("EB15_PONTE1_LOG")
if _logpath:
    _fh = logging.FileHandler(_logpath, mode="w", encoding="utf-8")
    _handlers.append(_fh)
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s [ponte1] %(message)s', handlers=_handlers)
log = logging.getLogger('ponte1')


def connect_retry(host, port, what, attempts=40, delay=0.5):
    for i in range(attempts):
        try:
            s = socket.create_connection((host, port), timeout=2)
            s.settimeout(None)  # o timeout de conexão não pode matar o relé ocioso após 2 s
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            log.info("conectado a %s %s:%d", what, host, port)
            return s
        except OSError:
            log.info("  aguardando %s %s:%d... (%d/%d)", what, host, port, i + 1, attempts)
            time.sleep(delay)
    log.error("FALHA ao conectar em %s %s:%d", what, host, port)
    return None


def run(args):
    qemu = connect_retry(args.qemu_host, args.qemu_port, "QEMU-UART1")
    sim = connect_retry(args.sim_host, args.sim_uart, "simavr-UART")
    ctrl = None
    if not args.no_trigger:
        ctrl = connect_retry(args.sim_host, args.sim_ctrl, "simavr-CTRL")
    if not qemu or not sim or (not args.no_trigger and not ctrl):
        return 1

    stop = threading.Event()
    counters = {'esp2uno': 0, 'uno2esp': 0, 'acks': 0, 'naks': 0, 'dones': 0, 'estops': 0, 'triggers': 0}

    def esp_to_uno():
        """ESP (QEMU) -> Uno (simavr): repassa bytes crus.

        Buffer look-ahead: o mestre CARREGA N fatias (frames LOAD) e então envia UM
        frame EXECUTE (duration_ms == 0xFFFF). O Escravo INICIA a reprodução ao
        processar esse frame (auto-disparo no firmware) — esta ponte não precisa mais
        sintetizar o trigger D9. (Antes disparávamos a cada ACK, o que com multi-LOAD
        iniciava a reprodução já na 1ª fatia; e disparar no EXECUTE criava corrida com
        a leitura do próprio frame. O auto-disparo no firmware elimina ambos.)
        """
        while not stop.is_set():
            try:
                data = qemu.recv(256)
            except OSError:
                break
            if not data:
                break
            sim.sendall(data)
            counters['esp2uno'] += len(data)
            log.info("ESP->Uno  %s", data.hex())
        stop.set()

    def uno_to_esp():
        """Uno (simavr) -> ESP (QEMU): repassa bytes. O trigger NÃO é mais disparado
        aqui (era 1 por ACK); agora é disparado no frame EXECUTE (ver esp_to_uno)."""
        while not stop.is_set():
            try:
                data = sim.recv(256)
            except OSError:
                break
            if not data:
                break
            qemu.sendall(data)               # repassa ao ESP PRIMEIRO
            counters['uno2esp'] += len(data)
            for b in data:
                if b == UART_ACK:
                    counters['acks'] += 1
                    log.info("Uno->ESP  ACK(0x06)")
                elif b == UART_NAK:
                    counters['naks'] += 1; log.info("Uno->ESP  NAK(0x15)")
                elif b == UART_DONE:
                    counters['dones'] += 1; log.info("Uno->ESP  DONE(0x04)")
                elif b == UART_BUSY:
                    log.info("Uno->ESP  BUSY(0x12)")
                elif b == UART_ESTOP:
                    counters['estops'] += 1; log.info("Uno->ESP  ESTOP(0x05)")
                else:
                    log.info("Uno->ESP  byte 0x%02X", b)
        stop.set()

    t1 = threading.Thread(target=esp_to_uno, daemon=True)
    t2 = threading.Thread(target=uno_to_esp, daemon=True)
    t1.start(); t2.start()
    log.info("ponte 1 ATIVA (relé UART + trigger). Ctrl+C para encerrar.")
    try:
        while not stop.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    stop.set()
    log.info("encerrando. contadores=%s", counters)
    return 0


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu-host', default='127.0.0.1')
    ap.add_argument('--qemu-port', type=int, default=30100)
    ap.add_argument('--sim-host', default='127.0.0.1')
    ap.add_argument('--sim-uart', type=int, default=30200)
    ap.add_argument('--sim-ctrl', type=int, default=30201)
    ap.add_argument('--no-trigger', action='store_true')
    sys.exit(run(ap.parse_args()))
