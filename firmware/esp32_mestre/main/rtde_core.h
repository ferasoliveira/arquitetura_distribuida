#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace eb15_rtde {

constexpr uint32_t COMMAND_MAGIC = 0x35314245u;  // "EB15" em little-endian
constexpr uint8_t PROTOCOL_VERSION = 1;
// Watchdog de heartbeat. Em hardware real, 500 ms detecta perda de comunicação.
// Sob QEMU (não-tempo-real), o agendamento da task_rtde e os relés TCP introduzem
// jitter que pode atrasar o PROCESSAMENTO dos heartbeats de 200 ms para além de
// 500 ms de tempo EMULADO, disparando E-STOP espúrio que trava o robô pelo resto
// do ensaio. Relaxa-se o limite no build QEMU — mesma política já usada em
// UART_ACK_TIMEOUT_MS (config.h). O cliente continua enviando heartbeats a 200 ms.
#ifdef EB15_QEMU
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 5000;
#else
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 500;
#endif

enum class CommandType : uint8_t {
    MOVE_J = 1,
    HEARTBEAT = 2,
    ESTOP = 3,
    SET_MODE_RTDE = 4,
    SET_MODE_USER = 5,
};

enum class OperationMode : uint8_t { RTDE = 0, USER = 1 };
enum class RobotState : uint8_t { IDLE = 0, ARMED, MOVING, DONE, ESTOP, FAULT };
enum class DecodeResult : uint8_t {
    OK = 0,
    BAD_SIZE,
    BAD_MAGIC,
    BAD_VERSION,
    BAD_TYPE,
    BAD_PAYLOAD,
    STALE_SEQUENCE,
    WRONG_MODE,
};

struct __attribute__((packed)) CommandFrame {
    uint32_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t payload_size;
    uint32_t sequence;
    uint32_t planned_time_ms;
    float q_deg[6];
};

// Contrato congelado do Passo 3: 13 floats little-endian, total de 52 bytes.
struct __attribute__((packed)) TelemetryFrame {
    float q_deg[6];
    float error_deg[6];
    float temperature_c;
};

static_assert(sizeof(CommandFrame) == 40, "CommandFrame deve ter 40 bytes");
static_assert(sizeof(TelemetryFrame) == 52, "TelemetryFrame deve ter 52 bytes");

inline bool validType(uint8_t raw) {
    return raw >= static_cast<uint8_t>(CommandType::MOVE_J) &&
           raw <= static_cast<uint8_t>(CommandType::SET_MODE_USER);
}

inline bool validJoints(const float q[6]) {
    // Limite conservador adicional do EB15; os limites finais por junta ficam
    // na ponte HIL. Rejeitar NaN/Inf aqui evita contaminar a malha de controle.
    for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(q[i]) || q[i] < -360.0f || q[i] > 360.0f) return false;
    }
    return true;
}

class Session {
public:
    DecodeResult decode(const uint8_t *data, size_t size, uint32_t now_ms,
                        CommandFrame &out) {
        if (size != sizeof(CommandFrame)) return DecodeResult::BAD_SIZE;
        std::memcpy(&out, data, sizeof(out));
        if (out.magic != COMMAND_MAGIC) return DecodeResult::BAD_MAGIC;
        if (out.version != PROTOCOL_VERSION) return DecodeResult::BAD_VERSION;
        if (!validType(out.type)) return DecodeResult::BAD_TYPE;
        if (out.payload_size != sizeof(out.q_deg)) return DecodeResult::BAD_PAYLOAD;

        const CommandType type = static_cast<CommandType>(out.type);
        if (type == CommandType::MOVE_J) {
            if (mode_ != OperationMode::RTDE) return DecodeResult::WRONG_MODE;
            float aligned_q[6];
            std::memcpy(aligned_q, out.q_deg, sizeof(aligned_q));
            if (!validJoints(aligned_q)) return DecodeResult::BAD_PAYLOAD;
            if (has_sequence_ && out.sequence <= last_sequence_)
                return DecodeResult::STALE_SEQUENCE;
            last_sequence_ = out.sequence;
            has_sequence_ = true;
            state_ = RobotState::ARMED;
        }

        if (type == CommandType::HEARTBEAT || type == CommandType::MOVE_J) {
            last_heartbeat_ms_ = now_ms;
            heartbeat_seen_ = true;
        } else if (type == CommandType::ESTOP) {
            state_ = RobotState::ESTOP;
        }
        return DecodeResult::OK;
    }

    // A troca de modo só é autorizada em repouso. O chamador deve primeiro
    // frear e esvaziar a fila; nunca se troca a propriedade durante movimento.
    bool setMode(OperationMode requested) {
        if (state_ == RobotState::ARMED || state_ == RobotState::MOVING) return false;
        mode_ = requested;
        has_sequence_ = false;
        return true;
    }

    bool watchdogExpired(uint32_t now_ms) {
        if (mode_ != OperationMode::RTDE || state_ != RobotState::MOVING ||
            !heartbeat_seen_) return false;
        if (static_cast<uint32_t>(now_ms - last_heartbeat_ms_) <= HEARTBEAT_TIMEOUT_MS)
            return false;
        state_ = RobotState::ESTOP;
        return true;
    }

    void setState(RobotState state) { state_ = state; }
    RobotState state() const { return state_; }
    OperationMode mode() const { return mode_; }
    uint32_t lastSequence() const { return last_sequence_; }

private:
    OperationMode mode_ = OperationMode::RTDE;
    RobotState state_ = RobotState::IDLE;
    uint32_t last_sequence_ = 0;
    uint32_t last_heartbeat_ms_ = 0;
    bool has_sequence_ = false;
    bool heartbeat_seen_ = false;
};

}  // namespace eb15_rtde
