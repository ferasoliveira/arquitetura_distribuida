# -*- coding: utf-8 -*-
"""
protocol.py — Parser e Encoder dos Protocols UART e RTDE-EB15
=============================================================
TCC Fernando Oliveira — Passo 3: Ambiente Virtual, Pontes e Simulação
"""

import struct

RTDE_COMMAND_MAGIC = 0x35314245
RTDE_PROTOCOL_VERSION = 1
RTDE_CMD_MOVE_J = 1
RTDE_CMD_HEARTBEAT = 2
RTDE_CMD_ESTOP = 3
RTDE_CMD_SET_MODE_RTDE = 4
RTDE_CMD_SET_MODE_USER = 5
RTDE_COMMAND_FORMAT = "<IBBHII6f"
RTDE_COMMAND_SIZE = struct.calcsize(RTDE_COMMAND_FORMAT)


def encode_rtde_command(command_type: int, sequence: int = 0,
                        planned_time_ms: int = 0, joints=None) -> bytes:
    """Empacota o CommandFrame RTDE-EB15 congelado de 40 bytes."""
    q = [0.0] * 6 if joints is None else list(joints)
    if len(q) != 6:
        raise ValueError("joints deve conter exatamente 6 elementos")
    return struct.pack(
        RTDE_COMMAND_FORMAT, RTDE_COMMAND_MAGIC, RTDE_PROTOCOL_VERSION,
        int(command_type), 24, int(sequence) & 0xFFFFFFFF,
        int(planned_time_ms) & 0xFFFFFFFF, *q,
    )

# Constantes UART
FRAME_PREAMBLE = 0xAA
UART_ACK = 0x06
UART_NAK = 0x15
UART_DONE = 0x04
UART_BUSY = 0x12

def compute_checksum(payload_bytes: bytes) -> int:
    """Calcula o XOR de todos os bytes fornecidos."""
    xor_sum = 0
    for b in payload_bytes:
        xor_sum ^= b
    return xor_sum

def encode_uart_frame(steps_j4: int, steps_j5: int, steps_j6: int, duration_ms: int) -> bytes:
    """
    Empacota o frame UART de 10 bytes no formato little-endian:
      [0xAA (1B)][steps_j4 (2B)][steps_j5 (2B)][steps_j6 (2B)][duration_ms (2B)][checksum (1B)]
    """
    # Converte para inteiros de 16 bits
    s4 = int(round(steps_j4))
    s5 = int(round(steps_j5))
    s6 = int(round(steps_j6))
    dur = int(round(duration_ms))
    
    # Empacota os primeiros 9 bytes
    payload = struct.pack("<BhhhH", FRAME_PREAMBLE, s4, s5, s6, dur)
    checksum = compute_checksum(payload)
    return payload + struct.pack("<B", checksum)

def decode_uart_frame(raw_bytes: bytes) -> tuple:
    """
    Desempacota o frame UART de 10 bytes.
    Retorna (steps_j4, steps_j5, steps_j6, duration_ms, is_valid)
    """
    if len(raw_bytes) != 10:
        return 0, 0, 0, 0, False
    
    if raw_bytes[0] != FRAME_PREAMBLE:
        return 0, 0, 0, 0, False
    
    checksum_calculated = compute_checksum(raw_bytes[:9])
    checksum_received = raw_bytes[9]
    is_valid = (checksum_calculated == checksum_received)
    
    unpacked = struct.unpack("<BhhhH", raw_bytes[:9])
    return unpacked[1], unpacked[2], unpacked[3], unpacked[4], is_valid

def encode_rtde_frame(joints: list, errors: list, temp: float) -> bytes:
    """
    Empacota o frame de telemetria RTDE-EB15 de 52 bytes:
      - joints: list de 6 floats (posições desejadas/reais em graus)
      - errors: list de 6 floats (erros PID em graus)
      - temp: float (temperatura do ESP32-S3 em °C)
    """
    if len(joints) != 6 or len(errors) != 6:
        raise ValueError("joints e errors devem ter exatamente 6 elementos")
    
    return struct.pack("<ffffff ffffff f", *joints, *errors, temp)

def decode_rtde_frame(raw_bytes: bytes) -> dict:
    """
    Desempacota o frame de telemetria RTDE-EB15 de 52 bytes.
    Retorna dicionário com os campos.
    """
    if len(raw_bytes) != 52:
        raise ValueError(f"Tamanho do payload inválido: esperado 52 bytes, obtido {len(raw_bytes)}")
    
    unpacked = struct.unpack("<ffffff ffffff f", raw_bytes)
    return {
        "joints": list(unpacked[0:6]),
        "errors": list(unpacked[6:12]),
        "temperature": unpacked[12]
    }
