# Protocolo de Interface Mestre-Escravo (EB15)

**Versao:** 1.0 - congelado em 2026-06-18
**Escopo:** Unico ponto de dependencia entre Parte A (ESP32-S3 Mestre) e Parte B (Arduino Uno Escravo).
Qualquer alteracao neste documento invalida ambas as implementacoes e exige revisao cruzada.

---

## 1. Frame UART binario - 10 bytes (little-endian)

| Offset | Campo     | Tipo         | Descricao                                 |
|--------|-----------|--------------|-------------------------------------------|
| 0      | Preambulo | uint8_t      | Fixo 0xAA (FRAME_PREAMBLE)                |
| 1-2    | Passos J4 | int16_t LE   | Passos **relativos** assinados da Junta 4 |
| 3-4    | Passos J5 | int16_t LE   | Passos **relativos** assinados da Junta 5 |
| 5-6    | Passos J6 | int16_t LE   | Passos **relativos** assinados da Junta 6 |
| 7-8    | Duracao   | uint16_t LE  | Duracao do segmento em milissegundos      |
| 9      | Checksum  | uint8_t      | XOR dos bytes 0-8 (inclusive)             |

**Garantia em compilacao:** `static_assert(sizeof(UnoFrame) == 10, "frame size")`.

### 1.1 Semantica dos passos (campo relativo)

Os campos de passos J4-J6 sao **deltas** (posicao alvo - posicao atual no Escravo), em unidades de micropasso.
O Escravo mantem seu proprio acumulador de posicao absoluta.

### 1.2 Byte de checksum

checksum = frame[0] XOR frame[1] XOR ... XOR frame[8]

### 1.3 Verificacao de integridade no Escravo

1. Aguarda byte 0xAA (preambulo) - descarta bytes anteriores (ressincronizacao).
2. Le os 9 bytes seguintes.
3. Calcula XOR dos 9 bytes lidos (bytes 0-8 do frame completo).
4. Se checksum correto -> responde UART_ACK; se incorreto -> responde UART_NAK.

---

## 2. Respostas do Escravo (1 byte)

| Byte   | Simbolo     | Significado                                    |
|--------|-------------|------------------------------------------------|
| 0x06   | UART_ACK    | Frame recebido, XOR valido, Escravo armado     |
| 0x15   | UART_NAK    | Frame rejeitado (XOR, falha ou timeout)        |
| 0x12   | UART_BUSY   | Escravo ainda executando segmento anterior     |
| 0x04   | UART_DONE   | Segmento concluido com sucesso                 |
| 0x05   | UART_ESTOP  | Falha de hardware (encoder, timeout critico)   |

Nota sobre UART_ESTOP: emitido pelo Escravo quando `ENCODER_MAX_FAILURES` falhas
consecutivas de I2C forem detectadas em um encoder AS5600. Ao receber 0x05,
o Mestre deve ativar E-STOP global e aguardar intervenção humana antes de
reiniciar. Distinto de UART_NAK (erro de protocolo recuperavel).

---

## 3. Sinais fisicos e temporização

| Parametro              | Valor                                                          |
|------------------------|----------------------------------------------------------------|
| Interface UART         | ESP32-S3 Serial2 (GPIO19 RX / GPIO20 TX) <-> Uno D0/D1        |
| Baudrate               | 115200 bps, 8N1                                                |
| Nivel logico           | Divisor resistivo 1 kOhm/2 kOhm na linha 5V -> 3,3V ESP32-S3  |
| Trigger de sincronismo | ESP32-S3 GPIO 15 -> Uno D9 (PCINT1, banco PCINT0)             |
| Vetor de interrupcao   | PCINT0_vect (nao PCINT1_vect - confirmado pela datasheet)      |
| Polaridade do trigger  | **Borda de descida** (OUTPUT HIGH -> OUTPUT LOW)               |
| Frequencia de controle | 200 Hz (periodo 5 000 us) nas duas placas                      |
| Timeout de handshake   | UART_ACK_TIMEOUT_MS = 500 ms                                   |
| Timeout do trigger     | 500 ms apos ACK; expiracao causa parada ativa e NAK            |
| E-STOP do Escravo      | Uno A0/PCINT8, ativo em LOW; requer reinicializacao segura      |

### 3.1 Sequencia temporal de envio de um segmento

```
Mestre                                   Escravo
  |                                         |
  |-- frame 10 bytes ---------------------->| (~875 us a 115200)
  |                                         | valida XOR -> arma timers
  |   <-- UART_ACK (0x06) -----------------| (~120 us processamento)
  |                                         |
  |-- GPIO15 HIGH->LOW ------------------->| trigger borda de descida
  |  (apenas apos receber ACK)              | ISR PCINT0_vect: start J4-J6
  |                                         | Mestre inicia J1-J3 no mesmo instante
  |   <-- UART_DONE (0x04) ---------------| segmento concluido
  |  (Mestre aguarda antes do proximo)      |
```

### 3.2 Unidades de conversao

STEPS_PER_DEG[i] = (MOTOR_STEPS[i] x MICROSTEP[i] x REDUCTION[i]) / 360.0

Com MOTOR_STEPS=200 (NEMA 17), MICROSTEP=4 (1/4 passo), REDUCTION=1.0 (padrao):
STEPS_PER_DEG = (200 x 4 x 1.0) / 360 aprox. 2.2222 passos/grau

---

## 4. Criterio de aceite do contrato

Este documento permite implementar Emissor (Mestre) e Receptor (Escravo) sem nenhuma decisao implicita sobre:
- Layout de bytes e endianess
- Semantica relativa vs. absoluta dos passos
- Polaridade e vetor do trigger
- Semantica exata de cada byte de resposta

Falha persistente de encoder, E-STOP ou timeout do trigger desabilitam o Timer1,
forcam o enable dos A4988 para HIGH e impedem novo movimento. A latencia fisica
do trigger sera medida no Passo 4; nesta etapa garante-se apenas que o Timer1 e
habilitado diretamente dentro de `PCINT0_vect`, sem despacho pelo `loop()`.

---

## 5. RTDE-EB15 TCP (porta 30003)

Este protocolo e proprietario do EB15 e nao e compativel com o RTDE da Universal
Robots. Todos os inteiros e floats IEEE-754 sao little-endian.

### 5.1 Comando cliente -> Mestre (40 bytes)

| Offset | Campo | Tipo | Descricao |
|---:|---|---|---|
| 0-3 | magic | uint32 | `0x35314245` (`EB15` em LE) |
| 4 | version | uint8 | `1` |
| 5 | type | uint8 | 1 MoveJ; 2 heartbeat; 3 E-STOP; 4 modo RTDE; 5 modo User |
| 6-7 | payload_size | uint16 | `24` |
| 8-11 | sequence | uint32 | estritamente crescente para MoveJ |
| 12-15 | planned_time_ms | uint32 | timestamp monotono planejado |
| 16-39 | q1-q6 | 6 x float32 | alvo articular absoluto em graus |

O Mestre responde um byte `DecodeResult` imediatamente apos cada comando:
`0 OK`, `1 tamanho`, `2 magic`, `3 versao`, `4 tipo`, `5 payload`, `6 sequencia`,
`7 modo incorreto`. Um MoveJ repetido ou regressivo nunca sobrescreve a fila.

### 5.2 Telemetria Mestre -> cliente (52 bytes, 50 Hz)

Treze `float32`: `q1-q6`, `erro_q1-erro_q6`, `temperatura_c`. Na ausencia de
sensor de temperatura configurado, o ultimo campo e `NaN`, em vez de um valor
inventado. Posicoes e erros sao expressos em graus.

### 5.3 Estados, modos e seguranca

Estados internos: `IDLE`, `ARMED`, `MOVING`, `DONE`, `ESTOP`, `FAULT`. Os modos
RTDE e User sao mutuamente exclusivos. A troca durante movimento primeiro para
os atuadores e esvazia as filas. Em modo RTDE, MoveJ tambem conta como heartbeat;
durante `MOVING`, ausencia de heartbeat por mais de 500 ms aciona E-STOP. Queda
da conexao durante movimento aplica a mesma parada.
