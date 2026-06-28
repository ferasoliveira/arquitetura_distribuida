#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace eb15 {
constexpr unsigned CONTROL_HZ = 200;
constexpr unsigned CONTROL_PERIOD_US = 1000000U / CONTROL_HZ;
constexpr unsigned TELEMETRY_HZ = 50;
constexpr unsigned SEG_SLICE_MS = 50;
constexpr std::size_t MAX_SEGMENTS = 60;
constexpr unsigned UART_ACK_TIMEOUT_MS = 500;
constexpr unsigned UART_MAX_RETRIES = 2;

constexpr int PULSE_PIN[3] = {4, 6, 8};
constexpr int DIR_PIN[3] = {5, 7, 3};
constexpr int TRIGGER_PIN = 15;
constexpr int UART_RX_PIN = 19;
constexpr int UART_TX_PIN = 20;

constexpr float LINK_L1 = 150.0F;
constexpr float LINK_L2 = 200.0F;
constexpr float LINK_L3 = 200.0F;
constexpr float LINK_L6 = 80.0F;

constexpr std::array<float, 6> STEPS_PER_DEG = {
    200.0F * 4.0F / 360.0F, 200.0F * 4.0F / 360.0F,
    200.0F * 4.0F / 360.0F, 200.0F * 4.0F / 360.0F,
    200.0F * 4.0F / 360.0F, 200.0F * 4.0F / 360.0F};
constexpr std::array<float, 6> LIMIT_MIN_DEG = {-170, -45, -120, -180, -90, -360};
constexpr std::array<float, 6> LIMIT_MAX_DEG = {170, 180, 120, 180, 90, 360};
constexpr std::array<float, 6> MAX_SPEED_DEG_S = {45, 45, 45, 90, 90, 90};
constexpr std::array<float, 6> MAX_ACCEL_DEG_S2 = {90, 90, 90, 180, 180, 180};
constexpr std::array<float, 6> MAX_JERK_DEG_S3 = {300, 300, 300, 600, 600, 600};

constexpr std::array<float, 3> PID_KP = {0.8F, 0.8F, 0.8F};
constexpr std::array<float, 3> PID_KI = {0.05F, 0.05F, 0.05F};
constexpr std::array<float, 3> PID_KD = {0.08F, 0.08F, 0.08F};
constexpr float PID_GAMMA = 8.0F;
constexpr float PID_I_LIMIT = 20.0F;
constexpr float PID_DEADBAND_DEG = 0.05F;
}

