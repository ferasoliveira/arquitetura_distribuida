#include <Arduino.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "config.h"
#include "protocol_core.hpp"
#if UNO_CLOSED_LOOP
#include "pid_tanh.h"
#endif

using namespace eb15_uno;

FrameParser g_parser;
Dda3Axis g_dda;
volatile bool g_control_tick = false;
volatile bool g_done_pending = false;
volatile bool g_fault = false;
volatile uint8_t g_timer1_clock_bits = _BV(CS11);
uint32_t g_last_rx_ms = 0;
uint32_t g_armed_at_ms = 0;
uint8_t g_encoder_failures[3] = {0, 0, 0};
uint16_t g_encoder_raw[3] = {0, 0, 0};
volatile int32_t g_position_steps[3] = {0, 0, 0};

// Buffer look-ahead: o mestre CARREGA todas as fatias da curva S de J4-J6 (frames
// só com ACK, sem mover); no trigger, reproduzimos o buffer encadeado, seguindo a
// mesma curva S de J1-J3. Cada fatia traz a POSIÇÃO ALVO ABSOLUTA (passos) e a
// duração; o delta é calculado contra a posição real a cada fatia (auto-correção).
#define SEG_BUF_N 64
struct BufSeg { int16_t steps[3]; uint16_t dur; };
BufSeg g_buf[SEG_BUF_N];
volatile uint8_t g_buf_count = 0;   // fatias carregadas
volatile uint8_t g_buf_idx = 0;     // próxima a tocar
volatile bool g_playing = false;

#if UNO_CLOSED_LOOP
// ----------------------------------------------------------------------------
// Estado do controle em MALHA FECHADA (J4-J6). A referência da curva-S são as
// fatias do buffer (sub-alvos ABSOLUTOS em passos + duração), percorridas no tempo
// exatamente como os SEGMENTOS do ESP em run_control_cycle. O PID+tanh fecha sobre
// o encoder AS5600. Espelha o mestre — mesma estrutura, mesmo algoritmo.
// ----------------------------------------------------------------------------
static PIDState g_pid[3];
static float    g_step_accum[3]   = {0.0f, 0.0f, 0.0f};  // DDA fracionário do PID
static float    g_seg_target_deg[3] = {0.0f, 0.0f, 0.0f};// sub-alvo atual (graus)
static int16_t  g_seg_ticks_left  = 0;                   // ticks restantes da fatia
static bool     g_seg_active      = false;
static bool     g_have_target     = false;
static float    g_meas_deg[3]     = {0.0f, 0.0f, 0.0f};  // encoder → graus (debug/telemetria)

// Estado do LOCKSTEP (Etapa 3): estimador fundido + comando acumulado (espelha o ESP).
static float    g_est_deg[3]      = {0.0f, 0.0f, 0.0f};  // estimador (dead-reckon + fusão)
static int32_t  g_cmd_steps[3]    = {0, 0, 0};           // comando devolvido ao mestre (passos)
static bool     g_ls_seeded       = false;               // 1ª amostra adota o medido direto
static volatile bool g_lockstep_mode = false;            // true após o 1º TICK do mestre
                                                         // (desliga o tick interno + I2C)

// Converte o encoder bruto AS5600 (12 bits sobre 360°) em graus [-180,180].
static inline float encoderDeg(uint16_t raw) {
  float deg = (float)(raw & 0x0FFFu) * 360.0f / 4096.0f;
  if (deg > 180.0f) deg -= 360.0f;
  return deg;
}

// Emite `pulses` pulsos de passo no eixo (PD2/PD4/PD6), com direção. Igual ao
// emit_step do ESP: o trem de passos é a saída elétrica lida pela ponte3 → Webots.
static const uint8_t STEP_PORT_BIT[3] = { _BV(PD2), _BV(PD4), _BV(PD6) };
static inline void emitSteps(uint8_t axis, bool dir, int pulses) {
  if (pulses <= 0) return;
  digitalWrite(DIR_PINS[axis], dir ? HIGH : LOW);
  const uint8_t bit = STEP_PORT_BIT[axis];
  for (int p = 0; p < pulses; ++p) {
    PORTD |= bit;  delayMicroseconds(2);
    PORTD &= (uint8_t)~bit; delayMicroseconds(2);
    g_position_steps[axis] += dir ? 1 : -1;
  }
}
#endif

static inline void disableMotionFromIsr() {
  TIMSK1 &= static_cast<uint8_t>(~_BV(OCIE1A));
  TCCR1B &= static_cast<uint8_t>(~(_BV(CS12) | _BV(CS11) | _BV(CS10)));
  PORTD &= static_cast<uint8_t>(~(_BV(PD2) | _BV(PD4) | _BV(PD6)));  // STEP_J4/J5/J6 LOW
  PORTB |= _BV(PB0);  // D8 HIGH: A4988 desabilitados
}

void activeStop() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    disableMotionFromIsr();
    g_dda.stop();
    g_playing = false;
    g_buf_count = 0;
    g_buf_idx = 0;
  }
}

static void configureTimer1(uint16_t ticks, uint16_t duration_ms,
                            uint16_t &compare, uint8_t &clock_bits) {
  if (ticks == 0) {
    compare = 0xFFFF;
    clock_bits = _BV(CS12) | _BV(CS10);
    return;
  }
  if (duration_ms == 0) duration_ms = 1;
  uint32_t rate_hz = (static_cast<uint32_t>(ticks) * 1000UL + duration_ms - 1) / duration_ms;
  if (rate_hz < 1) rate_hz = 1;
  const uint16_t divisors[4] = {8, 64, 256, 1024};
  const uint8_t bits[4] = {_BV(CS11), _BV(CS11) | _BV(CS10),
                           _BV(CS12), _BV(CS12) | _BV(CS10)};
  for (uint8_t i = 0; i < 4; ++i) {
    uint32_t counts = (F_CPU / divisors[i]) / rate_hz;
    if (counts >= 20 && counts <= 65536UL) {
      compare = static_cast<uint16_t>(counts - 1UL);
      clock_bits = bits[i];
      return;
    }
  }
  // Comando acima da capacidade fisica: limita a 100 kHz (/8, 20 ticks).
  compare = 19;
  clock_bits = _BV(CS11);
}

// Arma e INICIA a próxima fatia do buffer que tenha movimento real (≥1 passo),
// pulando fatias de delta 0. Calcula o delta contra a posição REAL atual (protocolo
// absoluto, auto-correção). Retorna false se não há mais movimento no buffer.
// Chamada do PCINT0 (1ª fatia) e da ISR do TIMER1 (encadeamento) — sempre com
// interrupções desabilitadas (contexto de ISR).
static bool startNextBufSeg() {
  while (g_buf_idx < g_buf_count) {
    const uint8_t idx = g_buf_idx++;
    const int32_t d0 = (int32_t)g_buf[idx].steps[0] - g_position_steps[0];
    const int32_t d1 = (int32_t)g_buf[idx].steps[1] - g_position_steps[1];
    const int32_t d2 = (int32_t)g_buf[idx].steps[2] - g_position_steps[2];
    if (d0 == 0 && d1 == 0 && d2 == 0) continue;  // fatia sem movimento → pula

    UnoFrame f{};
    f.preamble    = FRAME_PREAMBLE;
    f.steps[0]    = g_buf[idx].steps[0];
    f.steps[1]    = g_buf[idx].steps[1];
    f.steps[2]    = g_buf[idx].steps[2];
    f.duration_ms = g_buf[idx].dur;
    const int32_t pos_now[3] = { g_position_steps[0], g_position_steps[1],
                                 g_position_steps[2] };
    g_dda.arm(f, pos_now);
    for (uint8_t i = 0; i < 3; ++i)
      digitalWrite(DIR_PINS[i], g_dda.directionPositive(i) ? HIGH : LOW);
    uint16_t compare; uint8_t clock_bits;
    configureTimer1(g_dda.totalTicks(), g_dda.durationMs(), compare, clock_bits);
    OCR1A = compare;
    g_timer1_clock_bits = clock_bits;
    TCNT1 = 0;
    g_dda.start();
    if (g_dda.running()) {
      TCCR1B = _BV(WGM12) | clock_bits;
      TIMSK1 |= _BV(OCIE1A);
      return true;
    }
  }
  return false;  // buffer esgotado
}

void processSerial() {
  const uint32_t now = millis();
  if (g_parser.receiving() && now - g_last_rx_ms > UART_RX_TIMEOUT_MS) g_parser.reset();

  while (Serial.available()) {
    const uint8_t byte_in = static_cast<uint8_t>(Serial.read());
    g_last_rx_ms = millis();
    UnoFrame frame{};
    const ParseResult result = g_parser.push(byte_in, g_dda.busy(), frame);
    if (result == ParseResult::ACK) {
#if UNO_CLOSED_LOOP
      if (frame.duration_ms == FRAME_TICK_MARKER) {
        g_lockstep_mode = true;   // a partir daqui: tick externo manda (desliga interno+I2C)
        // TICK do mestre (co-sim síncrona): steps[0..2] = encoder medido fresco de J4-J6.
        // Roda UM ciclo de controle fechado e responde 3x int16 = comando (passos).
        const uint16_t meas[3] = {
          (uint16_t)((uint16_t)frame.steps[0] & 0x0FFFu),
          (uint16_t)((uint16_t)frame.steps[1] & 0x0FFFu),
          (uint16_t)((uint16_t)frame.steps[2] & 0x0FFFu) };
        if (g_playing && !g_fault) {
          controlStep(true, meas);
        } else {
          // ocioso: segue a verdade (estimador) e MANTÉM o último comando (hold).
          for (uint8_t i = 0; i < 3; ++i) g_est_deg[i] = encoderDeg(meas[i]);
          g_ls_seeded = true;
        }
        uint8_t resp[6];
        for (uint8_t i = 0; i < 3; ++i) {
          int32_t c = g_cmd_steps[i];
          if (c >  32767) c =  32767; if (c < -32768) c = -32768;
          resp[2*i]   = (uint8_t)(c & 0xFF);
          resp[2*i+1] = (uint8_t)((c >> 8) & 0xFF);
        }
        Serial.write(resp, 6);
      } else if (frame.duration_ms == FRAME_EXEC_MARKER) {
#else
      if (frame.duration_ms == FRAME_EXEC_MARKER) {
#endif
        // EXECUTE: marcador (dur=0xFFFF) — NÃO bufferiza (steps=0 seria volta a 0).
        // Confirma PRIMEIRO (DDA ainda ocioso → ACK, não BUSY) e então inicia a
        // reprodução do buffer encadeado. Iniciar aqui (e não via trigger D9 externo)
        // evita a corrida em que o trigger chegava antes desta leitura e o parser
        // via o DDA já ocupado → respondia BUSY ao próprio EXECUTE.
        Serial.write(UART_ACK);
        if (!g_fault && g_buf_count > 0 && !g_playing) {
#if UNO_CLOSED_LOOP
          // MALHA FECHADA: arma o playback da REFERÊNCIA (controlCycle percorre as
          // fatias como sub-alvos e o PID+tanh fecha sobre o encoder). Sem DDA.
          ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            PORTB &= static_cast<uint8_t>(~_BV(PB0));  // habilita motores
            g_buf_idx = 0;
            g_seg_active = false;
            g_have_target = false;
            g_seg_ticks_left = 0;
            for (uint8_t i = 0; i < 3; ++i) {
              g_pid[i].integral = 0.0f;
              g_pid[i].prev_error = 0.0f;
              g_step_accum[i] = 0.0f;
            }
            g_playing = true;
          }
#else
          bool started;
          ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            PORTB &= static_cast<uint8_t>(~_BV(PB0));  // habilita motores
            g_playing = true;
            g_buf_idx = 0;
            started = startNextBufSeg();
          }
          if (!started) {  // buffer só com fatias de delta 0
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
              disableMotionFromIsr();
              g_playing = false;
              g_buf_count = 0;
            }
            g_done_pending = true;
          }
#endif
        }
      } else if (g_fault || g_playing) {
        // LOAD durante fault/reprodução: rejeita.
        Serial.write(UART_NAK);
      } else if (g_buf_count < SEG_BUF_N) {
        g_buf[g_buf_count].steps[0] = frame.steps[0];
        g_buf[g_buf_count].steps[1] = frame.steps[1];
        g_buf[g_buf_count].steps[2] = frame.steps[2];
        g_buf[g_buf_count].dur      = frame.duration_ms;
        g_buf_count++;
        g_armed_at_ms = millis();
        Serial.write(UART_ACK);
      } else {
        Serial.write(UART_NAK);  // buffer cheio
      }
    } else if (result == ParseResult::NAK) {
      Serial.write(UART_NAK);
    } else if (result == ParseResult::BUSY) {
      Serial.write(UART_BUSY);
    }
  }
}

#define SDA_RELEASE() (DDRC &= static_cast<uint8_t>(~_BV(PC2)))
#define SDA_LOW() do { PORTC &= static_cast<uint8_t>(~_BV(PC2)); DDRC |= _BV(PC2); } while (0)
#define SCL_RELEASE() (DDRC &= static_cast<uint8_t>(~_BV(PC3)))
#define SCL_LOW() do { PORTC &= static_cast<uint8_t>(~_BV(PC3)); DDRC |= _BV(PC3); } while (0)

static inline void i2cDelay() { delayMicroseconds(5); }
void i2cStart() { SDA_RELEASE(); SCL_RELEASE(); i2cDelay(); SDA_LOW(); i2cDelay(); SCL_LOW(); }
void i2cStop() { SDA_LOW(); i2cDelay(); SCL_RELEASE(); i2cDelay(); SDA_RELEASE(); i2cDelay(); }

bool i2cWrite(uint8_t value) {
  for (uint8_t mask = 0x80; mask; mask >>= 1) {
    if (value & mask) SDA_RELEASE(); else SDA_LOW();
    i2cDelay(); SCL_RELEASE(); i2cDelay(); SCL_LOW();
  }
  SDA_RELEASE(); i2cDelay(); SCL_RELEASE(); i2cDelay();
  const bool ack = !(PINC & _BV(PC2));
  SCL_LOW();
  return ack;
}

uint8_t i2cRead(bool ack) {
  uint8_t value = 0;
  SDA_RELEASE();
  for (uint8_t i = 0; i < 8; ++i) {
    value <<= 1; i2cDelay(); SCL_RELEASE(); i2cDelay();
    if (PINC & _BV(PC2)) value |= 1;
    SCL_LOW();
  }
  if (ack) SDA_LOW(); else SDA_RELEASE();
  i2cDelay(); SCL_RELEASE(); i2cDelay(); SCL_LOW(); SDA_RELEASE();
  return value;
}

bool readEncoder(uint8_t channel, uint16_t &raw) {
  i2cStart();
  if (!i2cWrite(TCA9548A_ADDR << 1) || !i2cWrite(static_cast<uint8_t>(1U << channel))) {
    i2cStop(); return false;
  }
  i2cStop();
  i2cStart();
  if (!i2cWrite(AS5600_ADDR << 1) || !i2cWrite(AS5600_RAW_ANGLE)) {
    i2cStop(); return false;
  }
  i2cStart();
  if (!i2cWrite((AS5600_ADDR << 1) | 1U)) { i2cStop(); return false; }
  const uint8_t high = i2cRead(true);
  const uint8_t low = i2cRead(false);
  i2cStop();
  raw = (static_cast<uint16_t>(high) << 8 | low) & 0x0FFFU;
  return true;
}

void pollEncoders() {
  for (uint8_t i = 0; i < 3; ++i) {
    uint16_t raw;
    if (readEncoder(ENCODER_CHANNELS[i], raw)) {
      g_encoder_raw[i] = raw;
      g_encoder_failures[i] = 0;
    } else if (++g_encoder_failures[i] >= ENCODER_MAX_FAILURES) {
      activeStop();
      g_fault = true;
      Serial.write(UART_ESTOP);  /* falha de hardware: mestre deve E-STOP */
      return;
    }
  }
}

#if UNO_CLOSED_LOOP
// Avança a referência (fatias do buffer = sub-alvos temporizados). Retorna true se está
// em HOLD (buffer esgotado, mantendo o último alvo até convergir), false se ativo/ocioso.
// Atualiza g_seg_target_deg/g_seg_ticks_left. Igual à lógica de segmentos do ESP.
static bool advanceReference() {
  const float dt = 1.0f / (float)CONTROL_HZ;
  bool holding = false;
  if (!g_seg_active || g_seg_ticks_left <= 0) {
    if (g_buf_idx < g_buf_count) {
      const uint8_t idx = g_buf_idx++;
      for (uint8_t i = 0; i < 3; ++i)
        g_seg_target_deg[i] = (float)g_buf[idx].steps[i] / STEPS_PER_DEG[i];
      const int ticks = (int)((float)g_buf[idx].dur / (1000.0f * dt) + 0.5f);
      g_seg_ticks_left = (int16_t)(ticks > 0 ? ticks : 1);
      g_seg_active = true;
      g_have_target = true;
    } else {
      g_seg_active = false;
      if (!g_have_target) return false;   // ocioso, sem alvo
      g_seg_ticks_left = 0;
      holding = true;
    }
  }
  if (!holding && g_seg_ticks_left > 0) g_seg_ticks_left--;
  return holding;
}

// Um ciclo de controle em malha fechada (espelha run_control_cycle do ESP).
//  lockstep=false (Etapa 1): tick interno; medido = encoder I2C direto; emite passos.
//  lockstep=true  (Etapa 3): medido vem do TICK do mestre (meas_raw); FUSÃO (banda baixa)
//                 + dead-reckoning + anti-windup; acumula g_cmd_steps (devolvido ao mestre).
static void controlStep(bool lockstep, const uint16_t *meas_raw) {
  const float dt = 1.0f / (float)CONTROL_HZ;  // 0,005 s
  if (!g_have_target && (g_buf_idx >= g_buf_count)) {
    // nada carregado: em lockstep ainda precisamos responder; só não há movimento.
    if (!lockstep) return;
  }
  const bool holding = advanceReference();

  bool converged = true;
  for (uint8_t i = 0; i < 3; ++i) {
    const float m_deg = encoderDeg(lockstep ? meas_raw[i] : g_encoder_raw[i]);
    g_meas_deg[i] = m_deg;

    // PID fecha sobre o MEDIDO (suave). NÃO usamos estimador dead-reckoned aqui: ele
    // salta 1 passo/ciclo e o termo derivativo (Kd/dt) amplifica o salto → oscilação
    // bang-bang. Com cur=medido, d_error reflete a velocidade real (suave) da junta. O
    // windup é contido pelo ANTI-WINDUP explícito no acumulador (não pelo dead-reckon).
    const float cur = m_deg;
    (void)g_est_deg;

    const float f_hz = pid_tanh_update(i, g_seg_target_deg[i], cur, dt, &g_pid[i]);
    int pulses; bool dir;
    pid_step_pulses(f_hz, dt, &g_step_accum[i], &pulses, &dir);

    const int signed_pulses = dir ? pulses : -pulses;
    if (lockstep) {
      // acumula o comando (o mestre lê pela resposta do TICK, não pelo step-port).
      g_cmd_steps[i] += signed_pulses;
      // ANTI-WINDUP: o comando não lidera o medido por mais que LEAD passos (stepper
      // que perde passos sob sobrecarga; impede o acumulador de disparar até o limite).
      const int32_t m_steps = (int32_t)lroundf(m_deg * STEPS_PER_DEG[i]);
      if (g_cmd_steps[i] > m_steps + LOCKSTEP_LEAD_STEPS) g_cmd_steps[i] = m_steps + LOCKSTEP_LEAD_STEPS;
      if (g_cmd_steps[i] < m_steps - LOCKSTEP_LEAD_STEPS) g_cmd_steps[i] = m_steps - LOCKSTEP_LEAD_STEPS;
    } else {
      emitSteps(i, dir, pulses);
    }
    if (fabsf(g_seg_target_deg[i] - m_deg) > PID_DEADBAND_DEG) converged = false;
  }
  if (lockstep) g_ls_seeded = true;

  if (holding && converged && g_playing) {
    g_playing = false;
    g_done_pending = true;
  }
}

// Wrapper p/ o tick interno da Etapa 1 (compatibilidade).
static inline void controlCycle() { controlStep(false, (const uint16_t *)0); }
#endif

ISR(TIMER1_COMPA_vect) {
  const uint8_t axes = g_dda.tick();
  // CNC Shield v3.1 EB15: STEP_J4=PD2(D2), STEP_J5=PD4(D4), STEP_J6=PD6(D6)
  // Máscara 0x54 = PD2|PD4|PD6 — pinos pares, sem overlap com DIR (PD3|PD5|PD7=0xA8)
  uint8_t port_mask = 0;
  if (axes & 0x01) port_mask |= _BV(PD2);
  if (axes & 0x02) port_mask |= _BV(PD4);
  if (axes & 0x04) port_mask |= _BV(PD6);
  PORTD |= port_mask;             // os tres STEP sobem na mesma instrucao
  delayMicroseconds(2);
  PORTD &= static_cast<uint8_t>(~port_mask);
  for (uint8_t i = 0; i < 3; ++i)
    if (axes & (1U << i)) g_position_steps[i] += g_dda.directionPositive(i) ? 1 : -1;
  if (!g_dda.running()) {
    // Fatia concluída: encadeia a próxima do buffer (curva S contínua). Quando o
    // buffer esgota, encerra a reprodução e sinaliza DONE (uma vez, fim do MoveJ).
    if (!startNextBufSeg()) {
      disableMotionFromIsr();
      g_playing = false;
      g_buf_count = 0;
      g_buf_idx = 0;
      g_done_pending = true;
    }
  }
}

ISR(TIMER2_COMPA_vect) {
  // 78,125 ticks de 64 us: sete periodos de 78 e um de 79 = 200 Hz exatos na media.
  static uint8_t phase = 0;
  OCR2A = (++phase & 0x07) ? 77 : 78;
  g_control_tick = true;
}

ISR(PCINT0_vect) {
#if UNO_CLOSED_LOOP
  // Malha fechada: o EXECUTE (UART) arma o playback; o trigger externo D9 não é usado
  // (evita armar o DDA, que não participa do controle fechado).
  (void)0;
#else
  // Trigger (D9 ↓): inicia a reprodução do buffer look-ahead. Toca a 1ª fatia com
  // movimento; a ISR do TIMER1 encadeia as demais (curva S de J4-J6).
  if (!(PINB & _BV(PB1)) && g_buf_count > 0 && !g_playing && !g_fault) {
    PORTB &= static_cast<uint8_t>(~_BV(PB0));  // enable LOW
    g_playing = true;
    g_buf_idx = 0;
    if (!startNextBufSeg()) {   // buffer só com fatias de delta 0 → nada a mover
      disableMotionFromIsr();
      g_playing = false;
      g_buf_count = 0;
      g_done_pending = true;
    }
  }
#endif
}

ISR(PCINT1_vect) {
  if (!(PINC & _BV(PC0))) {
    disableMotionFromIsr();
    g_dda.stop();
    g_fault = true;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);  // HardwareSerial D0/D1, recepcao por interrupcao do core AVR
  for (uint8_t i = 0; i < 3; ++i) {
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
  }
  pinMode(MOTORS_ENABLE, OUTPUT);
  digitalWrite(MOTORS_ENABLE, HIGH);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  SDA_RELEASE(); SCL_RELEASE();

  cli();
  TCCR1A = 0; TCCR1B = _BV(WGM12); TIMSK1 = 0;
  TCCR2A = _BV(WGM21); TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
  OCR2A = 77; TIMSK2 = _BV(OCIE2A);
  PCICR = _BV(PCIE0) | _BV(PCIE1);
  PCMSK0 = _BV(PCINT1);  // D9
  PCMSK1 = _BV(PCINT8);  // A0
  sei();
}

void loop() {
  processSerial();
  // Buffer carregado mas trigger não chegou a tempo → descarta (evita que o próximo
  // MoveJ acrescente fatias a um buffer obsoleto). g_armed_at_ms reinicia a cada
  // LOAD, então não dispara no meio de uma carga.
  if (g_buf_count > 0 && !g_playing && millis() - g_armed_at_ms > TRIGGER_TIMEOUT_MS) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { g_buf_count = 0; g_buf_idx = 0; }
  }
  if (g_control_tick) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { g_control_tick = false; }
#if UNO_CLOSED_LOOP
    // Em LOCKSTEP (tick externo do mestre) o medido vem pelo TICK (UART) e o controle
    // roda lá — o tick interno e o encoder I2C ficam DESLIGADOS (senão dupla-controle e
    // E-STOP por falha de leitura I2C). Etapa 1 (sem TICKs): tick interno + I2C normais.
    if (!g_lockstep_mode) {
      pollEncoders();
      if (g_playing && !g_fault) controlCycle();
    }
#else
    pollEncoders();
#endif
  }
  if (g_done_pending) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { g_done_pending = false; }
    g_dda.takeDone();              // limpa o flag interno do DDA
#if UNO_CLOSED_LOOP
    // Em lockstep o DONE quebraria o enquadramento das respostas de 6 bytes do TICK; o
    // mestre detecta convergência pelo medido. Só emite DONE fora do lockstep.
    if (!g_lockstep_mode) Serial.write(UART_DONE);
#else
    Serial.write(UART_DONE);       // fim do MoveJ (buffer reproduzido por inteiro)
#endif
  }
}
