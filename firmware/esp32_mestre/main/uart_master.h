/**
 * EB-15 Robotic Arm — Mestre (ESP32-S3)
 * uart_master.h — Protocolo UART para comunicação com Arduino Uno Escravo
 *
 * RESPONSABILIDADES:
 *   - Empacotamento do frame de 10 bytes com checksum XOR
 *   - Envio via UART2 e aguardo de ACK/NAK com timeout e retransmissão
 *   - Disparo do trigger GPIO 15 após confirmação de ACK
 *   - Aguardo de UART_DONE antes de enviar próximo segmento
 *   - E-STOP: força GPIO 15 LOW e não envia novos frames
 *
 * POSIÇÃO DOS PASSOS: deltas relativos (J4-J6) calculados pelo chamador.
 * O Escravo mantém posição absoluta internamente.
 *
 * DEPENDÊNCIA: driver/uart.h (ESP-IDF) ou stub em testes nativos.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* --- Dependência de plataforma --- */
#ifndef EB15_NATIVE_TEST
  #include "driver/uart.h"
  #include "driver/gpio.h"
  #include "esp_log.h"
  /* QEMU: UART1 é o segundo -serial arg (mais confiável que UART2 no emulador).
   * Hardware real: usar UART_NUM_2 (GPIO19/20). Mudar aqui ao migrar. */
  #ifdef EB15_QEMU
    #define UART_PORT UART_NUM_1
  #else
    #define UART_PORT UART_NUM_2
  #endif
  static const char* UART_TAG = "UART_MST";

  #include "rom/ets_sys.h"   /* esp_rom_delay_us() */
  static inline int  uart_write_bytes_plat(const void *buf, size_t len) {
      return uart_write_bytes(UART_PORT, buf, len);
  }
  static inline int  uart_read_bytes_plat(void *buf, size_t len, uint32_t timeout_ms) {
      return uart_read_bytes(UART_PORT, buf, (uint32_t)len,
                             pdMS_TO_TICKS(timeout_ms));
  }
  static inline void trigger_set(bool high) {
      gpio_set_level((gpio_num_t)TRIGGER_PIN, high ? 1 : 0);
  }
  static inline void uart_log_error(const char *msg) { ESP_LOGE(UART_TAG, "%s", msg); }
  static inline void uart_log_info(const char *msg)  { ESP_LOGI(UART_TAG, "%s", msg); }
#else
  /* Stubs para testes nativos (validate_math.cpp) */
  #include <string.h>
  #define UART_PORT 0
  static inline int uart_write_bytes_plat(const void*, size_t)           { return 0; }
  static inline int uart_read_bytes_plat(void *buf, size_t, uint32_t)    { return 0; }
  static inline void trigger_set(bool)  {}
  static inline void uart_log_error(const char*) {}
  static inline void uart_log_info(const char*)  {}
#endif

/* Estado da máquina de estados UART do Mestre */
typedef enum {
    UART_STATE_IDLE = 0,
    UART_STATE_WAIT_ACK,
    UART_STATE_WAIT_DONE,
    UART_STATE_ESTOP
} UartMasterState;

static UartMasterState s_uart_state = UART_STATE_IDLE;

/* ---- Funções internas ---- */

static uint8_t compute_checksum(const uint8_t *buf, int n) {
    uint8_t csum = 0;
    for (int i = 0; i < n; ++i) csum ^= buf[i];
    return csum;
}

/**
 * Dispara o trigger físico (GPIO 15 HIGH → LOW → HIGH após 10 µs).
 * Deve ser chamado SOMENTE após carregar as fatias (uart_load_frame).
 * O Escravo detecta a borda de descida e inicia os timers na ISR.
 */
static void uart_fire_trigger(void) {
    trigger_set(true);   /* garante HIGH antes da borda */
#ifndef EB15_NATIVE_TEST
    esp_rom_delay_us(2);
    trigger_set(false);  /* borda de descida → ISR no Escravo */
    esp_rom_delay_us(10);
    trigger_set(true);   /* restaura HIGH */
#endif
}

/**
 * Aguarda UART_DONE do Escravo antes de prosseguir para o próximo segmento.
 * Timeout: duration_ms + 200ms de margem.
 */
static bool uart_wait_done(uint32_t segment_duration_ms) {
    if (s_uart_state != UART_STATE_WAIT_DONE) return true;

    uint8_t resp = 0;
#ifdef EB15_QEMU
    /* QEMU: ISR de UART via TCP pode ter latência variável.
     * Usa polling em janelas de 50 ms para garantir que o byte seja lido
     * mesmo se o sinal do ISR chegar tarde. Timeout total = 30 s virtual. */
    {
        /* Espera o DONE, mas com teto curto: se o byte DONE se perder no transporte
         * emulado do QEMU, NÃO bloquear task_uart_master por dezenas de segundos
         * (era o que atrasava a resposta de J4-J6 vários segundos). O movimento do
         * escravo já terminou (duração « 1 s) e o próximo frame só vem após o settle
         * (segundos depois) — assume-se concluído e segue. Protocolo absoluto torna
         * o próximo frame auto-corretivo, então um DONE perdido é inócuo. */
        const uint32_t POLL_MS   = 50;
        const uint32_t MAX_POLLS = (segment_duration_ms + 1500) / POLL_MS;
        for (uint32_t p = 0; p < MAX_POLLS; ++p) {
            int n = uart_read_bytes_plat(&resp, 1, POLL_MS);
            if (n > 0 && resp == UART_DONE) {
                s_uart_state = UART_STATE_IDLE;
                return true;
            }
            if (n > 0) {
                uart_log_error("byte inesperado aguardando DONE — descartado");
            }
        }
        uart_log_info("DONE nao recebido no prazo — assumindo concluido (mov. ja terminou)");
        s_uart_state = UART_STATE_IDLE;
        return true;  /* não-fatal: evita E-STOP e bloqueio longo */
    }
#else
    const uint32_t timeout = segment_duration_ms + 200;
    int n = uart_read_bytes_plat(&resp, 1, timeout);
    if (n <= 0 || resp != UART_DONE) {
        uart_log_error("DONE nao recebido apos segmento");
        return false;
    }
    s_uart_state = UART_STATE_IDLE;
    return true;
#endif
}

/**
 * LOAD (buffer look-ahead, PIPELINED): escreve UMA fatia da curva S (posição ALVO
 * ABSOLUTA em passos) para o Escravo bufferizar — SEM esperar ACK aqui. Os ACKs são
 * lidos em bloco por uart_load_barrier() antes do EXECUTE. Pipelinar evita pagar a
 * latência de ida-e-volta do relé UART do QEMU por fatia (~350 ms × N fatias → N
 * dezenas de ms no total). Perda de fatia é rara no co-sim e, se ocorrer, a última
 * (alvo exato) cobre o destino e o protocolo absoluto corrige no MoveJ seguinte.
 */
/**
 * TICK lockstep (co-sim síncrona): envia o encoder MEDIDO de J4-J6 (raw AS5600) ao
 * Escravo, que roda UM ciclo de controle fechado e responde 6 bytes (3x int16 = comando
 * em passos). É o "sub-master" do Uno feito pelo ESP, 1 vez por passo do Webots. Retorna
 * true e preenche out_cmd_steps[3]; em timeout/E-STOP retorna false (mestre mantém último).
 */
static bool uart_tick(const uint16_t measured_raw[3], int32_t out_cmd_steps[3])
{
    if (s_uart_state == UART_STATE_ESTOP) return false;
    UnoFrame frame;
    frame.preamble    = FRAME_PREAMBLE;
    frame.steps_j4    = (int16_t)(measured_raw[0] & 0x0FFFu);
    frame.steps_j5    = (int16_t)(measured_raw[1] & 0x0FFFu);
    frame.steps_j6    = (int16_t)(measured_raw[2] & 0x0FFFu);
    frame.duration_ms = FRAME_TICK_MARKER;
    frame.checksum    = compute_checksum((const uint8_t*)&frame, 9);
    uart_write_bytes_plat(&frame, sizeof(frame));

    uint8_t resp[6];
    int got = 0;
    while (got < 6) {
        int n = uart_read_bytes_plat(resp + got, 6 - got, UART_ACK_TIMEOUT_MS);
        if (n <= 0) { uart_log_error("TICK sem resposta"); return false; }
        got += n;
    }
    for (int i = 0; i < 3; ++i)
        out_cmd_steps[i] = (int16_t)((uint16_t)resp[2*i] | ((uint16_t)resp[2*i+1] << 8));
    return true;
}

static bool uart_load_frame(const int32_t target_steps_j[3], uint16_t duration_ms)
{
    if (s_uart_state == UART_STATE_ESTOP) return false;
    for (int i = 0; i < 3; ++i)
        if (target_steps_j[i] < INT16_MIN || target_steps_j[i] > INT16_MAX) {
            uart_log_error("Alvo absoluto excede int16");
            return false;
        }

    UnoFrame frame;
    frame.preamble    = FRAME_PREAMBLE;
    frame.steps_j4    = (int16_t)target_steps_j[0];
    frame.steps_j5    = (int16_t)target_steps_j[1];
    frame.steps_j6    = (int16_t)target_steps_j[2];
    frame.duration_ms = duration_ms;
    frame.checksum    = compute_checksum((const uint8_t*)&frame, 9);

    uart_write_bytes_plat(&frame, sizeof(frame));   /* pipelined: ACK lido depois */
    return true;
}

/**
 * Barreira do pipeline: lê os ACKs das `n` fatias carregadas, garantindo que o
 * Escravo bufferizou todas ANTES do EXECUTE. Best-effort (não fatal): se um ACK não
 * chegar a tempo, segue (perda rara; alvo final cobre). Retorna o nº de ACKs lidos.
 */
static int uart_load_barrier(int n)
{
    int acks = 0;
    for (int i = 0; i < n; ++i) {
        uint8_t resp = 0;
        int r = uart_read_bytes_plat(&resp, 1, UART_ACK_TIMEOUT_MS);
        if (r <= 0) break;                                   /* timeout: para */
        if (resp == UART_ACK)              ++acks;
        else if (resp == UART_ESTOP_BYTE) { s_uart_state = UART_STATE_ESTOP; break; }
        /* NAK/outro: ignora e segue */
    }
    return acks;
}

/**
 * EXECUTE (buffer look-ahead): envia o frame MARCADOR (duration_ms = 0xFFFF, steps
 * = 0) e então dispara o trigger; o Escravo reproduz o buffer de fatias encadeado e
 * envia UM DONE ao fim. Por que um frame e não só o trigger: em QEMU o GPIO do ESP
 * não é exportável e a ponte1 sintetiza o trigger — antes era 1 por ACK (quebrava o
 * multi-LOAD); agora a ponte1 dispara ao VER este frame EXECUTE. Em HW o pulso GPIO
 * abaixo inicia a reprodução. total_dur_ms = duração do movimento (teto do DONE).
 */
static bool uart_execute(uint32_t total_dur_ms)
{
    if (s_uart_state == UART_STATE_ESTOP) return false;

    UnoFrame frame;
    frame.preamble    = FRAME_PREAMBLE;
    frame.steps_j4    = 0;
    frame.steps_j5    = 0;
    frame.steps_j6    = 0;
    frame.duration_ms = 0xFFFF;            /* marcador EXECUTE */
    frame.checksum    = compute_checksum((const uint8_t*)&frame, 9);

    bool acked = false;
    for (int attempt = 0; attempt < UART_MAX_RETRIES; ++attempt) {
        uart_write_bytes_plat(&frame, sizeof(frame));
        uint8_t resp = 0;
        int n = uart_read_bytes_plat(&resp, 1, UART_ACK_TIMEOUT_MS);
        if (n <= 0) { uart_log_error("ACK timeout (execute)"); continue; }
        if (resp == UART_ACK)        { acked = true; break; }
        if (resp == UART_ESTOP_BYTE) { s_uart_state = UART_STATE_ESTOP; return false; }
        uart_log_info("resp inesperada (execute) — retransmitindo");
    }
    if (!acked) { uart_log_error("EXECUTE sem ACK"); return false; }

    uart_fire_trigger();                   /* HW: pulso GPIO; QEMU: no-op (ponte1 dispara) */
    s_uart_state = UART_STATE_WAIT_DONE;
    return uart_wait_done(total_dur_ms);
}

/** Coloca o canal UART em E-STOP imediato. */
static void uart_estop(void) {
    s_uart_state = UART_STATE_ESTOP;
    trigger_set(true);  /* mantém HIGH (inativo) */
}

/** Reseta estado após E-STOP externo ser resolvido. */
static void uart_reset(void) {
    s_uart_state = UART_STATE_IDLE;
    trigger_set(true);
}
