#pragma once

#include <stdint.h>
#include <string.h>

namespace eb15_uno {

constexpr uint8_t FRAME_PREAMBLE = 0xAA;
constexpr uint8_t UART_ACK   = 0x06;
constexpr uint8_t UART_NAK   = 0x15;
constexpr uint8_t UART_BUSY  = 0x12;
constexpr uint8_t UART_DONE  = 0x04;
constexpr uint8_t UART_ESTOP = 0x05;  /* falha de hardware: encoder, timeout crítico */
constexpr uint8_t FRAME_SIZE = 10;

#pragma pack(push, 1)
struct UnoFrame {
  uint8_t preamble;
  int16_t steps[3];
  uint16_t duration_ms;
  uint8_t checksum;
};
#pragma pack(pop)
static_assert(sizeof(UnoFrame) == FRAME_SIZE, "UnoFrame deve ocupar 10 bytes");

inline uint8_t frameXor(const uint8_t *bytes) {
  uint8_t value = 0;
  for (uint8_t i = 0; i < FRAME_SIZE - 1; ++i) value ^= bytes[i];
  return value;
}

inline int16_t readI16LE(const uint8_t *p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}

inline uint16_t readU16LE(const uint8_t *p) {
  return static_cast<uint16_t>(p[0]) |
         (static_cast<uint16_t>(p[1]) << 8);
}

enum class ParseResult : uint8_t { NONE, ACK, NAK, BUSY };

class FrameParser {
 public:
  FrameParser() : index_(0) {}

  void reset() { index_ = 0; }
  bool receiving() const { return index_ != 0; }

  ParseResult push(uint8_t byte, bool busy, UnoFrame &out) {
    if (index_ == 0) {
      if (byte == FRAME_PREAMBLE) buffer_[index_++] = byte;
      return ParseResult::NONE;
    }

    buffer_[index_++] = byte;
    if (index_ != FRAME_SIZE) return ParseResult::NONE;
    index_ = 0;

    if (frameXor(buffer_) != buffer_[FRAME_SIZE - 1]) return ParseResult::NAK;
    if (busy) return ParseResult::BUSY;

    out.preamble = buffer_[0];
    out.steps[0] = readI16LE(&buffer_[1]);
    out.steps[1] = readI16LE(&buffer_[3]);
    out.steps[2] = readI16LE(&buffer_[5]);
    out.duration_ms = readU16LE(&buffer_[7]);
    out.checksum = buffer_[9];
    return ParseResult::ACK;
  }

 private:
  uint8_t buffer_[FRAME_SIZE];
  uint8_t index_;
};

class Dda3Axis {
 public:
  Dda3Axis() { clear(); }

  // PROTOCOLO ABSOLUTO: frame.steps[i] é a POSIÇÃO ALVO ABSOLUTA (em passos).
  // O delta é calculado contra a posição REAL atual do escravo (current_pos,
  // = g_position_steps, incrementada por passo realmente emitido). Assim, frames
  // perdidos/divergência do mestre se auto-corrigem no próximo segmento: o escravo
  // sempre caminha do ponto onde de fato está até o alvo absoluto comandado.
  void arm(const UnoFrame &frame, const int32_t current_pos[3]) {
    clear();
    duration_ms_ = frame.duration_ms;
    for (uint8_t i = 0; i < 3; ++i) {
      const int32_t delta = static_cast<int32_t>(frame.steps[i]) - current_pos[i];
      direction_positive_[i] = delta >= 0;
      const uint32_t mag = static_cast<uint32_t>(delta < 0 ? -delta : delta);
      steps_[i] = static_cast<uint16_t>(mag > 0xFFFFu ? 0xFFFFu : mag);
      if (steps_[i] > total_ticks_) total_ticks_ = steps_[i];
    }
    // Fase inicial: todo eixo nao nulo emite seu primeiro pulso no ciclo 1.
    for (uint8_t i = 0; i < 3; ++i)
      accumulator_[i] = steps_[i] ? total_ticks_ - steps_[i] : 0;
    armed_ = true;
  }

  void start() {
    if (!armed_) return;
    if (total_ticks_ == 0) {
      /* Nenhum passo necessário: reporta DONE imediatamente sem mover. */
      armed_ = false;
      done_  = true;
      return;
    }
    running_ = true;
  }

  uint8_t tick() {
    if (!running_) return 0;
    uint8_t pulse_mask = 0;
    for (uint8_t i = 0; i < 3; ++i) {
      accumulator_[i] += steps_[i];
      if (steps_[i] && accumulator_[i] >= total_ticks_) {
        accumulator_[i] -= total_ticks_;
        ++emitted_[i];
        pulse_mask |= static_cast<uint8_t>(1U << i);
      }
    }
    if (++tick_index_ >= total_ticks_) {
      running_ = false;
      armed_ = false;
      done_ = true;
    }
    return pulse_mask;
  }

  void stop() { clear(); }
  bool busy() const { return armed_ || running_; }
  bool waitingTrigger() const { return armed_ && !running_; }
  bool running() const { return running_; }
  bool takeDone() {
    const bool result = done_;
    done_ = false;
    return result;
  }
  bool directionPositive(uint8_t axis) const { return direction_positive_[axis]; }
  uint16_t totalTicks() const { return total_ticks_; }
  uint16_t durationMs() const { return duration_ms_; }
  uint16_t emitted(uint8_t axis) const { return emitted_[axis]; }

 private:
  void clear() {
    memset(steps_, 0, sizeof(steps_));
    memset(accumulator_, 0, sizeof(accumulator_));
    memset(emitted_, 0, sizeof(emitted_));
    memset(direction_positive_, 0, sizeof(direction_positive_));
    total_ticks_ = tick_index_ = duration_ms_ = 0;
    armed_ = running_ = done_ = false;
  }

  uint16_t steps_[3], accumulator_[3], emitted_[3];
  bool direction_positive_[3];
  uint16_t total_ticks_, tick_index_, duration_ms_;
  bool armed_, running_, done_;
};

}  // namespace eb15_uno
