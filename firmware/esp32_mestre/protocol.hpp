#pragma once

#include <cstdint>
#include <cstring>
#include "config.hpp"

namespace eb15 {
constexpr uint8_t FRAME_PREAMBLE = 0xAA;
constexpr uint8_t UART_ACK = 0x06;
constexpr uint8_t UART_NAK = 0x15;
constexpr uint8_t UART_BUSY = 0x12;
constexpr uint8_t UART_DONE = 0x04;

#pragma pack(push, 1)
struct UnoFrame {
    uint8_t preamble;
    int16_t steps_j4;
    int16_t steps_j5;
    int16_t steps_j6;
    uint16_t duration_ms;
    uint8_t checksum;
};
#pragma pack(pop)
static_assert(sizeof(UnoFrame) == 10, "UnoFrame deve ocupar 10 bytes");

inline uint8_t frame_xor(const UnoFrame &frame) {
    const auto *bytes = reinterpret_cast<const uint8_t *>(&frame);
    uint8_t value = 0;
    for (std::size_t i = 0; i < sizeof(UnoFrame) - 1; ++i) value ^= bytes[i];
    return value;
}

inline UnoFrame make_frame(const int16_t relative_steps[3], uint16_t duration_ms) {
    UnoFrame frame{FRAME_PREAMBLE, relative_steps[0], relative_steps[1],
                   relative_steps[2], duration_ms, 0};
    frame.checksum = frame_xor(frame);
    return frame;
}
}

