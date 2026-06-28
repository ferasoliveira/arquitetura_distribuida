# -*- coding: utf-8 -*-
"""
test_protocol.py — Testes Unitários do Protocolo UART e RTDE-EB15
================================================================
TCC Fernando Oliveira — Passo 3: Ambiente Virtual, Pontes e Simulação
"""

import os
import sys
import pytest

# Adiciona o caminho dos helpers para importar o protocolo
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "helpers")))
import protocol

def test_uart_encode_decode_valid():
    """Valida que o empacotamento e desempacotamento de um frame UART válido mantêm integridade."""
    s4, s5, s6 = 1500, -800, 300
    dur = 250
    
    # Codifica
    frame = protocol.encode_uart_frame(s4, s5, s6, dur)
    assert len(frame) == 10
    assert frame[0] == 0xAA
    
    # Decodifica
    ds4, ds5, ds6, ddur, is_valid = protocol.decode_uart_frame(frame)
    assert is_valid is True
    assert ds4 == s4
    assert ds5 == s5
    assert ds6 == s6
    assert ddur == dur

def test_uart_checksum_corruption():
    """Valida que um frame corrompido é rejeitado pelo decodificador."""
    s4, s5, s6, dur = 100, 200, 300, 1000
    frame = bytearray(protocol.encode_uart_frame(s4, s5, s6, dur))
    
    # Corrompe o checksum (último byte)
    frame[9] ^= 0xFF
    
    ds4, ds5, ds6, ddur, is_valid = protocol.decode_uart_frame(bytes(frame))
    assert is_valid is False

def test_uart_invalid_preamble():
    """Valida que preâmbulos incorretos são rejeitados."""
    s4, s5, s6, dur = 100, 200, 300, 1000
    frame = bytearray(protocol.encode_uart_frame(s4, s5, s6, dur))
    
    # Altera preâmbulo
    frame[0] = 0x55
    
    ds4, ds5, ds6, ddur, is_valid = protocol.decode_uart_frame(bytes(frame))
    assert is_valid is False

def test_rtde_encode_decode_valid():
    """Valida o empacotamento e desempacotamento do frame RTDE-EB15 de 52 bytes."""
    joints = [10.5, -20.2, 30.1, 45.0, 90.0, -180.5]
    errors = [0.01, -0.02, 0.05, 0.0, 0.0, 0.0]
    temp = 32.5
    
    # Codifica
    frame = protocol.encode_rtde_frame(joints, errors, temp)
    assert len(frame) == 52
    
    # Decodifica
    decoded = protocol.decode_rtde_frame(frame)
    assert len(decoded["joints"]) == 6
    assert len(decoded["errors"]) == 6
    
    for i in range(6):
        assert abs(decoded["joints"][i] - joints[i]) < 1e-4
        assert abs(decoded["errors"][i] - errors[i]) < 1e-4
        
    assert abs(decoded["temperature"] - temp) < 1e-4

def test_rtde_invalid_length():
    """Valida que frames RTDE com tamanho incorreto levantam exceção."""
    with pytest.raises(ValueError):
        protocol.decode_rtde_frame(b"\x00" * 50) # Menor
        
    with pytest.raises(ValueError):
        protocol.decode_rtde_frame(b"\x00" * 54) # Maior
