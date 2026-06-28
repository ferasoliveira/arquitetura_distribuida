/**
 * EB-15 Robotic Arm — Mestre (ESP32-S3)
 * config.h — Configuração centralizada de hardware e dinâmica
 *
 * Compilar com ESP-IDF (idf.py build) e flag EB15_QEMU=1 para emulação QEMU.
 */
#pragma once

#include <stdint.h>
#include <math.h>

#ifndef constrainf
#define constrainf(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#endif

// ============================================================================
// 1. PINAGEM (ESP32-S3 DevKitC-1)
// ============================================================================

/* Step/Dir J1–J3 → Drivers TB6600 */
#define PUL_J1   4
#define DIR_J1   5
#define PUL_J2   6
#define DIR_J2   7
#define PUL_J3   8
#define DIR_J3   3

/* I2C principal — encoders AS5600 J1-J3 via TCA9548A */
#define I2C_SDA_PIN    1
#define I2C_SCL_PIN    2
#define TCA9548A_ADDR  0x70

/* UART2 para comunicação com Arduino Uno Escravo */
#define UART2_RX_PIN   19   /* recebe TX do Uno */
#define UART2_TX_PIN   20   /* envia ao RX do Uno via divisor 1kΩ/2kΩ */
#define UART2_BAUD     115200

/* Trigger digital de sincronismo — GPIO 15 → Uno D9/PCINT1 (banco PCINT0) */
#define TRIGGER_PIN    15

/* LED NeoPixel embutido */
#define LED_PIN        48

// ============================================================================
// 2. GEOMETRIA DO ROBÔ (comprimentos de elos em mm — modelo proxy UR10e)
// ============================================================================

static constexpr float LINK_L1 = 150.0f;   /* altura base → ombro          */
static constexpr float LINK_L2 = 200.0f;   /* braço (J2 → J3)              */
static constexpr float LINK_L3 = 200.0f;   /* antebraço (J3 → wrist center) */
static constexpr float LINK_L6 =  80.0f;   /* garra TCP (wrist center → TCP)*/

// ============================================================================
// 3. TRANSMISSÃO MECÂNICA
// ============================================================================

/* Reduções por junta (altere aqui para calibrar sem tocar no código principal) */
static constexpr float REDUCTION[6] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };

static constexpr int   MOTOR_STEPS[6] = { 200, 200, 200, 200, 200, 200 }; /* NEMA17 */
static constexpr int   MICROSTEP[6]   = {   4,   4,   4,   4,   4,   4 }; /* 1/4 passo */

/* Passos por grau — calculado a partir dos parâmetros acima */
static constexpr float STEPS_PER_DEG[6] = {
    (MOTOR_STEPS[0] * MICROSTEP[0] * REDUCTION[0]) / 360.0f,
    (MOTOR_STEPS[1] * MICROSTEP[1] * REDUCTION[1]) / 360.0f,
    (MOTOR_STEPS[2] * MICROSTEP[2] * REDUCTION[2]) / 360.0f,
    (MOTOR_STEPS[3] * MICROSTEP[3] * REDUCTION[3]) / 360.0f,
    (MOTOR_STEPS[4] * MICROSTEP[4] * REDUCTION[4]) / 360.0f,
    (MOTOR_STEPS[5] * MICROSTEP[5] * REDUCTION[5]) / 360.0f,
};

// ============================================================================
// 4. LIMITES DE SEGURANÇA (soft limits, em GRAUS)
// ============================================================================

static constexpr float LIMIT_MIN_DEG[6] = { -170.0f, -45.0f, -120.0f, -180.0f, -90.0f, -360.0f };
static constexpr float LIMIT_MAX_DEG[6] = {  170.0f, 180.0f,  120.0f,  180.0f,  90.0f,  360.0f };

// ============================================================================
// 5. DINÂMICA DE MOVIMENTO (por junta)
// ============================================================================

static constexpr float MAX_SPEED_DEG_S[6]  = {  45.0f,  45.0f,  45.0f,  90.0f,  90.0f,  90.0f };
static constexpr float MAX_ACCEL_DEG_S2[6] = {  90.0f,  90.0f,  90.0f, 180.0f, 180.0f, 180.0f };
static constexpr float MAX_JERK_DEG_S3[6]  = { 300.0f, 300.0f, 300.0f, 600.0f, 600.0f, 600.0f };

// ============================================================================
// 6. CONTROLADOR PID + TANH (J1–J3, malha fechada local)
// ============================================================================

static constexpr float PID_KP[3]    = { 0.8f, 0.8f, 0.8f };
static constexpr float PID_KI[3]    = { 0.05f, 0.05f, 0.05f };
// Kd REDUZIDO (era 2,0): no laço SÍNCRONO discreto d_error ≈ -velocidade_junta (°/s), então
// Kd alto fazia o derivativo dominar e só permitir convergência ESTÁVEL com v < 0,4·e (lento)
// — origem da lentidão histórica do J1-J3 no co-sim. Kd menor amplia a faixa estável
// (v < 8·e), permitindo convergir RÁPIDO. Mantém Kp/Ki/γ e a forma PID+tanh. Validado p/
// J4-J6 no test_uno_lockstep_tick; aplicado a J1-J3 p/ o circular. Ver project_passo4_lockstep.
static constexpr float PID_KD[3]    = { 0.1f, 0.1f, 0.1f };
static constexpr float PID_GAMMA[3] = { 8.0f, 8.0f, 8.0f };

#define PID_DEADBAND_DEG   0.5f     /* zona morta >= 1 passo (1/STEPS_PER_DEG ≈ 0.45°) */
#define PID_I_LIMIT        20.0f    /* anti-windup do integrador         */

/* Frequência máxima de step (Hz) para PID+Tanh */
static constexpr float PID_FMAX_HZ[3] = {
    MAX_SPEED_DEG_S[0] * STEPS_PER_DEG[0],
    MAX_SPEED_DEG_S[1] * STEPS_PER_DEG[1],
    MAX_SPEED_DEG_S[2] * STEPS_PER_DEG[2],
};

// ============================================================================
// 7. TEMPORIZAÇÃO DO SISTEMA
// ============================================================================

#define CONTROL_HZ            200
#define TIMER_PERIOD_US       (1000000 / CONTROL_HZ)   /* 5 000 µs */
#define SEG_SLICE_MS          50     /* fatia de trajetória: 50 ms → 20 Hz */
#define MAX_SEGMENTS          60     /* fila circular de segmentos          */

// ============================================================================
// 8. PROTOCOLO UART (frame binário 10 bytes)
// ============================================================================

#define FRAME_PREAMBLE        0xAA
#define FRAME_EXEC_MARKER     0xFFFF   /* duration_ms: reproduz o buffer (EXECUTE)        */
#define FRAME_TICK_MARKER     0xFFFE   /* duration_ms: TICK lockstep — steps=encoder J4-J6,
                                          escravo roda 1 ciclo e responde 6 bytes (comando) */
#define UART_ACK              0x06
#define UART_NAK              0x15
#define UART_DONE             0x04
#define UART_BUSY             0x12
#define UART_ESTOP_BYTE       0x05
#ifdef EB15_QEMU
/* QEMU: latência TCP→UART1 variável. 1500 ms cobre o ACK típico (~11 ms) com folga,
 * mas limita o bloqueio quando o transporte emulado trava: 3 retries → ≤4,5 s em vez
 * de ≤15 s, melhorando o tempo de resposta de J4-J6 (protocolo absoluto → reenvio
 * idempotente). */
#define UART_ACK_TIMEOUT_MS   1500
#else
#define UART_ACK_TIMEOUT_MS   500
#endif
#define UART_MAX_RETRIES      3

/* Frame enviado do Mestre para o Escravo */
struct __attribute__((__packed__)) UnoFrame {
    uint8_t  preamble;     /* 0xAA                                    */
    int16_t  steps_j4;    /* posição ALVO ABSOLUTA J4 em passos (LE) */
    int16_t  steps_j5;    /* posição ALVO ABSOLUTA J5 em passos (LE) */
    int16_t  steps_j6;    /* posição ALVO ABSOLUTA J6 em passos (LE) */
    uint16_t duration_ms; /* duração do segmento (LE)                */
    uint8_t  checksum;    /* XOR de bytes 0-8                        */
};
static_assert(sizeof(UnoFrame) == 10, "UnoFrame deve ter exatamente 10 bytes");
