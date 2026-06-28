#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>

#include "../arduino_escravo/protocol_core.hpp"

using namespace eb15_uno;

std::vector<uint8_t> frame(int16_t a, int16_t b, int16_t c, uint16_t ms) {
  std::vector<uint8_t> v = {FRAME_PREAMBLE,
      uint8_t(a), uint8_t(uint16_t(a) >> 8), uint8_t(b), uint8_t(uint16_t(b) >> 8),
      uint8_t(c), uint8_t(uint16_t(c) >> 8), uint8_t(ms), uint8_t(ms >> 8), 0};
  v[9] = frameXor(v.data());
  return v;
}

ParseResult feed(FrameParser &p, const std::vector<uint8_t> &bytes, bool busy, UnoFrame &out) {
  ParseResult result = ParseResult::NONE;
  for (uint8_t b : bytes) {
    ParseResult current = p.push(b, busy, out);
    if (current != ParseResult::NONE) result = current;
  }
  return result;
}

int main() {
  static_assert(sizeof(UnoFrame) == 10);
  FrameParser parser;
  UnoFrame decoded{};
  auto valid = frame(-300, 9, 1024, 50);
  assert(feed(parser, valid, false, decoded) == ParseResult::ACK);
  assert(decoded.steps[0] == -300 && decoded.steps[1] == 9 && decoded.steps[2] == 1024);
  assert(decoded.duration_ms == 50);

  auto corrupt = valid;
  corrupt[4] ^= 0x40;
  assert(feed(parser, corrupt, false, decoded) == ParseResult::NAK);

  std::vector<uint8_t> truncated(valid.begin(), valid.begin() + 5);
  assert(feed(parser, truncated, false, decoded) == ParseResult::NONE);
  parser.reset();  // equivale ao timeout inter-byte do firmware
  assert(feed(parser, valid, false, decoded) == ParseResult::ACK);
  assert(feed(parser, valid, true, decoded) == ParseResult::BUSY);

  Dda3Axis dda;
  UnoFrame move{};
  move.steps[0] = 9; move.steps[1] = 9; move.steps[2] = -9; move.duration_ms = 45;
  dda.arm(move); dda.start();
  assert(dda.tick() == 0x07);  // partida fisicamente simultanea
  while (dda.running()) dda.tick();
  assert(dda.emitted(0) == 9 && dda.emitted(1) == 9 && dda.emitted(2) == 9);
  assert(dda.takeDone() && !dda.takeDone());

  move.steps[0] = 2; move.steps[1] = 5; move.steps[2] = 9;
  dda.arm(move);
  assert(dda.waitingTrigger());
  dda.start();
  assert(dda.tick() == 0x07);
  while (dda.running()) dda.tick();
  assert(dda.emitted(0) == 2 && dda.emitted(1) == 5 && dda.emitted(2) == 9);

  dda.arm(move); dda.start(); dda.tick(); dda.stop();
  assert(!dda.busy() && dda.tick() == 0);  // E-STOP logico

  std::cout << "PASS: frame, XOR, ressincronizacao, BUSY, DDA e E-STOP\n";
}
