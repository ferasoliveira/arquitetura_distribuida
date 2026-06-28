# EB15 Robotic Arm — Co-Simulação HIL e Benchmark
===================================================
TCC Fernando Oliveira — Passo 3: Ambiente Virtual, Pontes e Simulação

Este diretório contém a infraestrutura de co-simulação Hardware-in-the-Loop (HIL) do braço robótico EB15, usando o modelo do robô UR10e como proxy dinâmico e cinemático no Webots R2025a.

## Estrutura de Pastas

*   `worlds/`: Contém os mundos do Webots, incluindo `eb15_hil.wbt`.
*   `controllers/`: Controladores do Webots, incluindo a ponte HIL `hil_bridge/hil_bridge.py`.
*   `benchmark/`: Orquestrador de benchmark, trajetórias canônicas e clientes RTDE.
*   `tests/`: Testes unitários e de fumaça (smoke tests).
*   `config.yaml`: Arquivo de configuração centralizado (portas, frequências e ruídos).

## Como Executar

### 1. Compilação e Build do Firmware ESP32-S3
Compile o firmware do ESP32-S3 com ESP-IDF e gere a imagem de flash para o emulador QEMU.
No PowerShell, execute:
```powershell
powershell -ExecutionPolicy Bypass -File Códigos/firmware/qemu/build_idf.ps1
```
> **Arquitetura canônica (ver `wiki/plano_acao_v2.md`):** o Arduino UNO executa o firmware **`.ino`
> real no simavr** (compilado por `arduino-cli`), **não** como bridge Python. O Python atua apenas
> como **pontes elétricas** (relés): UART ESP↔Uno (+trigger), ESP↔Webots (passo/encoder J1-J3) e
> Uno↔Webots (passo/encoder J4-J6). É proibido mock/reimplementação de firmware. *(Observação: os
> scripts `helpers/uno_bridge.py` e `firmware_sim.py` são legado da fase anterior e estão sendo
> substituídos pela ponte simavr — não usar em validações novas.)*

### 2. Execução do Benchmark
O adaptador de benchmark gerencia automaticamente a inicialização de todos os processos secundários (`uno_bridge.py`, emulador QEMU para o ESP32-S3, e o simulador Webots) em background e garante a limpeza deles após a conclusão ou interrupção.

Para executar a trajetória canônica no target EB15:
```bash
python benchmark/benchmark.py --target eb15 --trajectory canonical --headless
```

Para rodar com interface gráfica no Webots:
```bash
python benchmark/benchmark.py --target eb15 --trajectory canonical --no-headless
```

As coletas de telemetria serão salvas na pasta `output/` como arquivos CSV em conformidade com o schema estabelecido em `benchmark/schemas/csv_schema.json`.
