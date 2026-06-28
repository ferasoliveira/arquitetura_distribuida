#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>

#include "../esp32_mestre/main/rtde_core.h"

using namespace eb15_rtde;

static CommandFrame command(CommandType type, uint32_t sequence = 0) {
    CommandFrame frame{};
    frame.magic = COMMAND_MAGIC;
    frame.version = PROTOCOL_VERSION;
    frame.type = static_cast<uint8_t>(type);
    frame.payload_size = sizeof(frame.q_deg);
    frame.sequence = sequence;
    frame.planned_time_ms = 1000 + sequence * 50;
    for (int i = 0; i < 6; ++i) frame.q_deg[i] = static_cast<float>(i + sequence);
    return frame;
}

int main() {
    static_assert(sizeof(CommandFrame) == 40, "contrato de comando alterado");
    static_assert(sizeof(TelemetryFrame) == 52, "contrato de telemetria alterado");

    Session session;
    CommandFrame decoded{};
    CommandFrame move = command(CommandType::MOVE_J, 10);
    assert(session.decode(reinterpret_cast<const uint8_t *>(&move), sizeof(move), 100,
                          decoded) == DecodeResult::OK);
    assert(decoded.sequence == 10 && session.state() == RobotState::ARMED);

    // Sequência repetida ou regressiva nunca pode sobrescrever um segmento.
    assert(session.decode(reinterpret_cast<const uint8_t *>(&move), sizeof(move), 110,
                          decoded) == DecodeResult::STALE_SEQUENCE);
    move.sequence = 9;
    assert(session.decode(reinterpret_cast<const uint8_t *>(&move), sizeof(move), 120,
                          decoded) == DecodeResult::STALE_SEQUENCE);

    // Corrupção estrutural e valores não finitos são rejeitados.
    move = command(CommandType::MOVE_J, 11);
    move.magic ^= 1;
    assert(session.decode(reinterpret_cast<const uint8_t *>(&move), sizeof(move), 130,
                          decoded) == DecodeResult::BAD_MAGIC);
    move = command(CommandType::MOVE_J, 11);
    move.q_deg[2] = NAN;
    assert(session.decode(reinterpret_cast<const uint8_t *>(&move), sizeof(move), 140,
                          decoded) == DecodeResult::BAD_PAYLOAD);

    // Troca de propriedade durante movimento é proibida; em repouso reinicia seq.
    session.setState(RobotState::MOVING);
    assert(!session.setMode(OperationMode::USER));
    session.setState(RobotState::IDLE);
    assert(session.setMode(OperationMode::USER));
    move = command(CommandType::MOVE_J, 1);
    assert(session.decode(reinterpret_cast<const uint8_t *>(&move), sizeof(move), 150,
                          decoded) == DecodeResult::WRONG_MODE);
    assert(session.setMode(OperationMode::RTDE));
    assert(session.decode(reinterpret_cast<const uint8_t *>(&move), sizeof(move), 200,
                          decoded) == DecodeResult::OK);

    // O watchdog atua apenas durante movimento e exatamente após a janela de 500 ms.
    session.setState(RobotState::MOVING);
    assert(!session.watchdogExpired(700));
    assert(session.watchdogExpired(701));
    assert(session.state() == RobotState::ESTOP);

    std::cout << "PASS: RTDE frames, sequencia, modos, validacao e watchdog\n";
}
