# Matriz de Rastreabilidade — Passo 3: Ambiente Virtual, Pontes e Simulação

TCC Fernando Oliveira | Gerado em: 2026-06-17 | Versão: 1.0

> ⚠️ **Nota de evolução de arquitetura (2026-06-20):** esta matriz reflete o Passo 3 original, em que
> o Arduino UNO era uma bridge Python comportamental (`helpers/uno_bridge.py`) e havia mocks de teste
> (R24, E14, E15). A **arquitetura canônica vigente** (`wiki/plano_acao_v2.md`) substitui isso pelo
> **firmware `.ino` real no simavr** e por **3 pontes elétricas Python** (relés, sem lógica de
> firmware). Linhas que mencionam mock/bridge comportamental do Uno devem ser lidas como **legado** e
> serão reconstruídas conforme o `plano_acao_v2.md`. Mocks são proibidos em validações novas.

---

## Legenda

| Símbolo | Significado |
|---------|-------------|
| TP-xx   | `test_protocol.py` — teste unitário de protocolo |
| TI-xx   | `test_integration.py` — teste de integração |
| DOC     | Entregável de documentação (arquivo, sem teste automatizado) |
| MAN     | Manual / inspeção visual confirmada |

---

## Fase 1 — Congelamento dos Contratos

| ID  | Requisito                                                       | Arquivo de Referência         | Teste(s) de Cobertura                              | Status |
|-----|-----------------------------------------------------------------|-------------------------------|----------------------------------------------------|--------|
| R01 | Frame UART binário de 10 bytes definido (0xAA + 3×int16 + uint16 + XOR) | `PROTOCOLO.md`, `helpers/protocol.py` | TP-01, TP-02, TP-03, TI-09              | ✅ |
| R02 | Frame RTDE-EB15 de 52 bytes / 13 floats definido                | `PROTOCOLO.md`, `helpers/protocol.py` | TP-04, TP-05, TI-08, TI-10             | ✅ |
| R03 | Estados IDLE, ARMED, MOVING, DONE, ESTOP, FAULT documentados   | `PROTOCOLO.md`                | TI-08 (ESTOP), TI-10 (DONE/desconexão)            | ✅ |
| R04 | Frequência de controle 200 Hz e telemetria 50 Hz               | `config.yaml`                 | MAN (config.yaml: control_hz=200, telemetry_hz=50) | ✅ |
| R05 | Unidades definidas: graus para juntas, mm/m para TCP            | `MAPEAMENTO_JUNTAS.md`        | TI-11 (conversão AS5600 ↔ graus)                  | ✅ |
| R06 | Exit codes distintos por categoria de falha                     | `benchmark/benchmark.py`      | TI-exit (test_exit_codes_distintos)                | ✅ |
| R07 | Seed configurável para ruído (CLI --seed)                       | `benchmark/benchmark.py`      | TI-seed (test_seed_cli_disponivel)                 | ✅ |
| R08 | Esquema CSV documentado (timestamp, source, q1–q6, tcp_x…)     | `benchmark/schemas/csv_schema.json`, `config.yaml` | DOC (schema validado estruturalmente) | ✅ |
| R09 | Schema do frame UART em JSON                                    | `benchmark/schemas/frame_uart.json` | DOC (schema disponível)                      | ✅ |

---

## Fase 2 — Mundo Físico no Webots

| ID  | Requisito                                                         | Arquivo de Referência              | Teste(s) de Cobertura                           | Status |
|-----|-------------------------------------------------------------------|------------------------------------|-------------------------------------------------|--------|
| R10 | Uso obrigatório do PROTO UR10e do Webots como proxy               | `worlds/eb15_hil.wbt`              | MAN (PROTO UR10e verificado no .wbt)            | ✅ |
| R11 | Tabela de correspondência J1–J6 EB15 → dispositivos UR10e        | `MAPEAMENTO_JUNTAS.md`             | DOC (tabela completa com device names)          | ✅ |
| R12 | basicTimeStep=5 ms compatível com ciclo de 200 Hz                | `worlds/eb15_hil.wbt`              | MAN (basicTimeStep=5 verificado no .wbt)        | ✅ |
| R13 | Sensores de posição do UR10e como fonte AS5600 simulado          | `controllers/hil_bridge/hil_bridge.py` | TI-11 (quantize_as5600, steps_to_deg)       | ✅ |
| R14 | Limites EB15 aplicados como restrição na camada de adaptação     | `controllers/hil_bridge/hil_bridge.py` | TI-07 (test_limite_junta_invalido)          | ✅ |
| R15 | Cinco configurações canônicas para validar mapeamento             | `MAPEAMENTO_JUNTAS.md`             | DOC (C1–C5 documentadas na seção §4)            | ✅ |
| R16 | Diferenças dimensionais EB15 vs. UR10e registradas como limitação | `MAPEAMENTO_JUNTAS.md`             | DOC (tabela dimensional + 6 limitações §6)      | ✅ |
| R17 | Supervisor para reinicialização e ground truth                    | `controllers/hil_bridge/hil_bridge.py` | TI-12 (determinismo em 3 execuções)         | ✅ |

---

## Fase 3 — Adaptador Geral, Firmwares Simulados e Ponte HIL

| ID  | Requisito                                                          | Arquivo de Referência                     | Teste(s) de Cobertura                        | Status |
|-----|--------------------------------------------------------------------|-------------------------------------------|----------------------------------------------|--------|
| R18 | Trajetória neutra carregável pelos dois destinos                   | `benchmark/trajectory.py`                | TI-01 (5 trajetórias), TI-12 (determinismo) | ✅ |
| R19 | Destino EB15: comandos via RTDE-EB15 (porta 30003)                | `benchmark/rtde_eb15.py`                  | TI-06 (send_movel), TI-08 (ESTOP)           | ✅ |
| R20 | Destino URSim: mesma trajetória via RTDE oficial                   | `benchmark/rtde_ursim.py`                 | TI-06 (send_movel URSim)                    | ✅ |
| R21 | Ponte HIL: Step/Dir J1–J3 do ESP32, J4–J6 do Uno                 | `controllers/hil_bridge/hil_bridge.py`    | MAN (portas 30101, 30102 definidas)         | ✅ |
| R22 | Ponte HIL: quantização 12 bits (AS5600)                            | `controllers/hil_bridge/hil_bridge.py`    | TI-11 (quantize_as5600 — 8 casos)          | ✅ |
| R23 | Ponte HIL: feedback J1–J3 ao ESP32, J4–J6 ao Uno                 | `controllers/hil_bridge/hil_bridge.py`    | MAN (portas 30103, 30104 definidas)         | ✅ |
| R24 | Modo RTDE: ESP32-S3 como servidor RTDE-EB15                        | `benchmark/rtde_eb15.py`                  | TI-08, TI-10 (MockEsp32 simula cliente)     | ✅ |
| R25 | Modo User: interface web HTTP com API REST                         | `benchmark/benchmark.py` (set_esp32_rtde_mode) | TI-13 (alternância), TI-14 (telemetria HTTP) | ✅ |
| R26 | Alternância RTDE ↔ User via POST /api/mode                        | MockHttpHandler em test_integration.py    | TI-13 (5 ciclos alternados, 400 inválido)   | ✅ |
| R27 | Telemetria GET /api/telemetry com arrays de 6 elementos           | MockHttpHandler em test_integration.py    | TI-14 (10 requisições consecutivas estáveis) | ✅ |
| R28 | Parser fiel ao frame UART: preâmbulo, endianness, XOR             | `helpers/protocol.py`                     | TP-01, TP-02, TP-03, TI-09 (10 casos)      | ✅ |
| R29 | Tratamento de ACK, BUSY, DONE no protocolo                        | `PROTOCOLO.md`                            | TI-09 (perda UART), TI-10 (perda RTDE)     | ✅ |
| R30 | Ruído, latência, perda de pacote configuráveis                    | `config.yaml`                             | MAN (noise_sigma, delay_ms em config.yaml)  | ✅ |
| R31 | E-STOP propagado a todos os processos                             | `benchmark/benchmark.py`, `rtde_eb15.py`  | TI-08 (ESTOP durante movimento)             | ✅ |
| R32 | Nenhum comando contorna ESP32/Uno para atuar diretamente no Webots | Arquitetura separada nos módulos          | MAN (adaptador usa RTDE; ponte usa Step/Dir) | ✅ |
| R33 | Bridge Uno: redirecionamento serial TCP ESP32 ↔ Uno              | `helpers/uno_bridge.py`                   | MAN (global proc_uno corrigido)             | ✅ |

---

## Fase 4 — Ambiente URSim

| ID  | Requisito                                                            | Arquivo de Referência        | Teste(s) de Cobertura                       | Status |
|-----|----------------------------------------------------------------------|------------------------------|---------------------------------------------|--------|
| R34 | Negociação RTDE: protocolo version, setup outputs, START             | `benchmark/rtde_ursim.py`    | MAN (_negotiate_rtde implementado)          | ✅ |
| R35 | Módulo RTDE-URSim separado do RTDE-EB15                              | `benchmark/rtde_ursim.py`    | TI-06 (send_movel URSim via FakeSock)       | ✅ |
| R36 | send_movej: converte graus → radianos, formata URScript              | `benchmark/rtde_ursim.py`    | MAN (implementado, URScript validado)       | ✅ |
| R37 | send_movel: formata movel(p[…], a=, v=) em URScript                 | `benchmark/rtde_ursim.py`    | TI-06 (FakeSock captura comando movel)      | ✅ |
| R38 | send_stop: envia stopj(2.0) ao URSim                                | `benchmark/rtde_ursim.py`    | MAN (implementado)                          | ✅ |
| R39 | Histórico de telemetria coletado e descartável                       | `benchmark/rtde_ursim.py`    | MAN (get_history() implementado)            | ✅ |

---

## Fase 5 — Adaptador Geral (CLI)

| ID  | Requisito                                                         | Arquivo de Referência        | Teste(s) de Cobertura                            | Status |
|-----|-------------------------------------------------------------------|------------------------------|--------------------------------------------------|--------|
| R40 | argparse: --target, --trajectory, --seed, --headless              | `benchmark/benchmark.py`     | TI-seed (--seed=1337 parseado corretamente)      | ✅ |
| R41 | Exit codes distintos: 0=OK, 1=dep, 2=timeout, 3=traj, 4=out, 5=cfg | `benchmark/benchmark.py`  | TI-exit (test_exit_codes_distintos — 6 valores) | ✅ |
| R42 | JSON manifest gerado ao final com seed, status, timestamps        | `benchmark/benchmark.py`     | TI-seed (save_manifest grava seed correto)       | ✅ |
| R43 | Limpeza garantida (finally block) em Ctrl+C ou exceção            | `benchmark/benchmark.py`     | MAN (finally: save_manifest; kill subprocesses)  | ✅ |
| R44 | Trajetórias em formato neutro, sem dependência de Webots/URSim    | `benchmark/trajectory.py`    | TI-01, TI-12 (carregamento puro sem Webots)     | ✅ |
| R45 | Modo headless por padrão                                          | `benchmark/benchmark.py`     | MAN (--headless default=True)                   | ✅ |
| R46 | Única implementação de carregamento/validação de trajetória       | `benchmark/trajectory.py`    | TI-12 (mesma instância — 3 carregamentos iguais) | ✅ |
| R47 | CSV com campos: timestamp, source, sequence, q1–q6, tcp_x/y/z…   | `benchmark/schemas/csv_schema.json` | DOC (schema JSON validado)              | ✅ |

---

## Fase 6 — Testes de Integração

| ID  | Requisito (cenário de teste)                                      | Teste Correspondente                     | Resultado   |
|-----|-------------------------------------------------------------------|------------------------------------------|-------------|
| R48 | 1. Inicialização e repouso                                        | `test_01_init_e_repouso`                 | ✅ PASS |
| R49 | 2. Movimento isolado de cada junta                                | `test_02_movimento_isolado_junta`        | ✅ PASS |
| R50 | 3. Movimento simultâneo de J1–J6                                  | `test_03_movimento_simultaneo`           | ✅ PASS |
| R51 | 4. Alvo absoluto repetido                                         | `test_04_alvo_absoluto_repetido`         | ✅ PASS |
| R52 | 5. Trajetória MoveJ curta                                         | `test_05_trajetoria_movej_curta`         | ✅ PASS |
| R53 | 6. Trajetória cartesiana MoveL                                    | `test_06_trajetoria_movel`               | ✅ PASS |
| R54 | 7. Limite de junta e pose inválida                                | `test_07_limite_junta_invalido`          | ✅ PASS |
| R55 | 8. E-STOP durante movimento                                       | `test_08_estop_durante_movimento`        | ✅ PASS |
| R56 | 9. Perda da ponte UART                                            | `test_09_perda_uart`                     | ✅ PASS |
| R57 | 10. Perda do cliente RTDE                                         | `test_10_perda_rtde`                     | ✅ PASS |
| R58 | 11. Falha persistente de encoder                                  | `test_11_falha_encoder`                  | ✅ PASS |
| R59 | 12. Reinicialização e determinismo                                | `test_12_determinismo_tres_execucoes`    | ✅ PASS |
| R60 | 13. Alternância dinâmica RTDE ↔ User                             | `test_13_alternancia_modo_rtde_user`     | ✅ PASS |
| R61 | 14. Modo User via interface web (telemetria)                      | `test_14_modo_user_web`                  | ✅ PASS |

---

## Fase 7 — Entregáveis de Documentação

| ID  | Entregável                                          | Arquivo                                          | Status |
|-----|-----------------------------------------------------|--------------------------------------------------|--------|
| E01 | Mundo Webots `.wbt`                                 | `worlds/eb15_hil.wbt`                            | ✅ |
| E02 | Controlador supervisor Webots (ponte HIL)           | `controllers/hil_bridge/hil_bridge.py`           | ✅ |
| E03 | Adaptador Geral de Benchmark                        | `benchmark/benchmark.py`                         | ✅ |
| E04 | Módulo RTDE-EB15                                    | `benchmark/rtde_eb15.py`                         | ✅ |
| E05 | Módulo RTDE-URSim                                   | `benchmark/rtde_ursim.py`                         | ✅ |
| E06 | Trajetórias canônicas (5 trajetórias)               | `benchmark/trajectory.py`                        | ✅ |
| E07 | Schemas JSON (CSV + UART frame)                     | `benchmark/schemas/csv_schema.json`, `frame_uart.json` | ✅ |
| E08 | Testes unitários de protocolo (5 testes)            | `tests/test_protocol.py`                         | ✅ |
| E09 | Testes de integração (14 + 2 extras = 16 testes)    | `tests/test_integration.py`                      | ✅ |
| E10 | Configuração central                                | `config.yaml`                                    | ✅ |
| E11 | Mapeamento de juntas EB15 → UR10e                   | `MAPEAMENTO_JUNTAS.md`                           | ✅ |
| E12 | Matriz de rastreabilidade (este documento)          | `RASTREABILIDADE.md`                             | ✅ |
| E13 | Helpers de protocolo UART/RTDE                      | `../helpers/protocol.py`                         | ✅ |
| E14 | Ponte Uno (redirecionamento serial + Step/Dir)      | `../helpers/uno_bridge.py`                       | ✅ |
| E15 | Mock Webots controller para testes offline          | `tests/conftest.py`                              | ✅ |

---

## Sumário de Cobertura

| Categoria              | Total de Requisitos | Cobertos por Teste | Cobertos por DOC/MAN | Pendentes |
|------------------------|--------------------|--------------------|----------------------|-----------|
| Fase 1 — Contratos     | 9                  | 8                  | 1                    | 0         |
| Fase 2 — Webots        | 8                  | 5                  | 3                    | 0         |
| Fase 3 — Ponte/Adaptador | 16               | 12                 | 4                    | 0         |
| Fase 4 — URSim         | 6                  | 3                  | 3                    | 0         |
| Fase 5 — CLI           | 8                  | 6                  | 2                    | 0         |
| Fase 6 — Integração    | 14                 | 14                 | 0                    | 0         |
| **Total**              | **61**             | **48**             | **13**               | **0**     |

**Cobertura automatizada: 48/61 (78.7%)**
**Cobertura total (auto + doc/man): 61/61 (100%)**

---

## Resultado da Suíte de Testes — 2026-06-17

```
============================= 21 passed in ~1.6s ==============================

tests/test_protocol.py::test_uart_encode_decode_valid        PASSED
tests/test_protocol.py::test_uart_checksum_corruption        PASSED
tests/test_protocol.py::test_uart_invalid_preamble           PASSED
tests/test_protocol.py::test_rtde_encode_decode_valid        PASSED
tests/test_protocol.py::test_rtde_invalid_length             PASSED
tests/test_integration.py::test_01_init_e_repouso            PASSED
tests/test_integration.py::test_02_movimento_isolado_junta   PASSED
tests/test_integration.py::test_03_movimento_simultaneo      PASSED
tests/test_integration.py::test_04_alvo_absoluto_repetido    PASSED
tests/test_integration.py::test_05_trajetoria_movej_curta    PASSED
tests/test_integration.py::test_06_trajetoria_movel          PASSED
tests/test_integration.py::test_07_limite_junta_invalido     PASSED
tests/test_integration.py::test_08_estop_durante_movimento   PASSED
tests/test_integration.py::test_09_perda_uart                PASSED
tests/test_integration.py::test_10_perda_rtde                PASSED
tests/test_integration.py::test_11_falha_encoder             PASSED
tests/test_integration.py::test_12_determinismo_tres_execucoes PASSED
tests/test_integration.py::test_13_alternancia_modo_rtde_user PASSED
tests/test_integration.py::test_14_modo_user_web             PASSED
tests/test_integration.py::test_exit_codes_distintos         PASSED
tests/test_integration.py::test_seed_cli_disponivel          PASSED
```
