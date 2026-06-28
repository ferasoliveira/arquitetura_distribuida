#pragma once

#include <Arduino.h>

// CNC Shield v3.1 (projeto EB15): J4=D2/D3, J5=D4/D5, J6=D6/D7.
// STEP: D2(J4), D4(J5), D6(J6) — pinos pares de PORTD (0x54) — acesso atômico.
// DIR:  D3(J4), D5(J5), D7(J6) — pinos ímpares de PORTD (0xA8) — sem conflito com STEP.
// Enable: D8 (PORTB/PB0), ativo em LOW.
constexpr uint8_t STEP_PINS[3] = {2, 4, 6};
constexpr uint8_t DIR_PINS[3]  = {3, 5, 7};
constexpr uint8_t MOTORS_ENABLE = 8;  // ativo em LOW
constexpr uint8_t TRIGGER_PIN = 9;    // PB1/PCINT1 -> PCINT0_vect
constexpr uint8_t ESTOP_PIN = A0;     // PC0/PCINT8 -> PCINT1_vect, ativo em LOW

constexpr uint8_t I2C_SDA_PIN = A2;
constexpr uint8_t I2C_SCL_PIN = A3;
constexpr uint8_t TCA9548A_ADDR = 0x70;
constexpr uint8_t AS5600_ADDR = 0x36;
constexpr uint8_t AS5600_RAW_ANGLE = 0x0C;
constexpr uint8_t ENCODER_CHANNELS[3] = {3, 4, 5};
constexpr uint8_t ENCODER_MAX_FAILURES = 3;

/* Marcador EXECUTE do buffer look-ahead: um frame com duration_ms == 0xFFFF não é
   uma fatia (steps ignorados) — sinaliza "reproduza o buffer". Durações reais de
   fatia são « 0xFFFF, então não há ambiguidade. */
constexpr uint16_t FRAME_EXEC_MARKER = 0xFFFF;

/* Marcador TICK (co-sim SÍNCRONA, Etapa 3): um frame com duration_ms == 0xFFFE não é
   fatia — carrega o ENCODER medido fresco (steps[0..2] = raw AS5600 de J4-J6) e dispara
   UM ciclo de controle; o Uno responde com o COMANDO (3x int16 = posição comandada em
   passos). O mestre (via ESP) faz 1 TICK por passo do Webots → lockstep determinístico. */
constexpr uint16_t FRAME_TICK_MARKER = 0xFFFE;

/* Fusão de feedback (filtro complementar) no lockstep do Uno — MESMA ideia do ESP:
   estimador = dead-reckoning corrigido devagar pelo encoder (banda baixa → estável).
   E anti-windup: o comando não lidera o medido por mais que LEAD passos (stepper que
   perde passos sob sobrecarga, não acumula passos-fantasma). Ver project_passo4_lockstep. */
constexpr float    LOCKSTEP_ENC_ALPHA = 0.005F;
constexpr int16_t  LOCKSTEP_LEAD_STEPS = 4;    /* ~1,8° — pequeno: limita o overshoot (o
                                                  comando lidera o medido por no máx. isto) */

constexpr uint32_t SERIAL_BAUD = 115200UL;
constexpr uint16_t UART_RX_TIMEOUT_MS = 20;
/* Tempo máximo entre a última fatia carregada e o trigger de EXECUTE; após isso o
   buffer look-ahead é descartado (protege contra trigger perdido no transporte do
   co-sim, evitando que o próximo MoveJ acrescente fatias a um buffer obsoleto). */
constexpr uint16_t TRIGGER_TIMEOUT_MS = 1500;
constexpr uint16_t CONTROL_HZ = 200;

constexpr float REDUCTION[3] = {1.0F, 1.0F, 1.0F};
constexpr float STEPS_PER_DEG[3] = {
    (200.0F * 4.0F * REDUCTION[0]) / 360.0F,
    (200.0F * 4.0F * REDUCTION[1]) / 360.0F,
    (200.0F * 4.0F * REDUCTION[2]) / 360.0F};

static_assert(STEPS_PER_DEG[0] > 2.221F && STEPS_PER_DEG[0] < 2.223F,
              "Resolucao esperada: aproximadamente 2,222 passos/grau");

// ============================================================================
// Co-simulação SÍNCRONA em MALHA FECHADA (J4-J6) — UNO_CLOSED_LOOP
// 1 = o Uno fecha a malha com o encoder AS5600 (MESMO PID+tanh do ESP), rastreando
//     a referência da curva-S (fatias do buffer = sub-alvos temporizados, igual aos
//     segmentos do ESP). 0 = modo feedforward histórico (DDA toca os passos).
// ============================================================================
#ifndef UNO_CLOSED_LOOP
#define UNO_CLOSED_LOOP 0   /* 0 = feedforward (DDA) conhecido-bom para a integração:
                              a malha fechada do Uno valida isolada mas o sync por
                              tempo-real do co-sim a quebra; ver project_passo4_lockstep.
                              Mantido 0 enquanto o J1-J3 (Kd) é o foco. */
#endif

// Constantes do controlador PID+tanh para J4-J6 (mesmas de J1-J3 no ESP — simetria
// total entre mestre e escravo). Ajustáveis se houver overshoot.
constexpr float PID_KP[3]    = { 0.8F, 0.8F, 0.8F };
constexpr float PID_KI[3]    = { 0.05F, 0.05F, 0.05F };
// Kd REDUZIDO p/ J4-J6 (era 2,0, herdado de J1-J3): no laço SÍNCRONO discreto, d_error ≈
// -velocidade_junta (°/s), então Kd alto fazia o termo derivativo dominar e travar o avanço
// em ciclo-limite bang-bang (validado no test_uno_lockstep_tick). Kd menor mantém o
// amortecimento perto do alvo sem impedir o movimento. Mantém Kp/Ki/γ e a forma PID+tanh.
constexpr float PID_KD[3]    = { 0.10F, 0.10F, 0.10F };
constexpr float PID_GAMMA[3] = { 8.0F, 8.0F, 8.0F };
#define PID_DEADBAND_DEG   0.5F     /* zona morta ~1 passo (1/2,222 ≈ 0,45°) */
#define PID_I_LIMIT        20.0F    /* anti-windup do integrador            */

// Velocidade máxima de atuação por junta (graus/s). Modesta de propósito: as curvas-S
// têm pico « isto; pode ser reduzida ainda mais se a junta oscilar (prioridade: fazer
// direito, não rápido). FMAX (Hz) = vel_max * STEPS_PER_DEG.
constexpr float MAX_SPEED_DEG_S[3] = { 90.0F, 90.0F, 90.0F };
constexpr float PID_FMAX_HZ[3] = {
    MAX_SPEED_DEG_S[0] * STEPS_PER_DEG[0],
    MAX_SPEED_DEG_S[1] * STEPS_PER_DEG[1],
    MAX_SPEED_DEG_S[2] * STEPS_PER_DEG[2]};
