# Arquitetura Distribuída para Controle do Manipulador EB15

Códigos-fonte do firmware embarcado e da infraestrutura de co-simulação
*Hardware-in-the-Loop* (HIL) desenvolvidos no Trabalho de Conclusão de Curso
de **Fernando Oliveira**, sobre uma arquitetura de controle distribuído para o
braço robótico **EB15**.

O sistema particiona o controle das seis juntas entre dois microcontroladores:

- **Nó mestre — ESP32-S3:** planejamento de trajetória (Curva-S), cinemática
  (Denavit–Hartenberg direta / Pieper inversa), laço de controle PID a 200 Hz,
  servidor HTTP e endpoint síncrono *lockstep*. Controla as juntas **J1–J3**.
- **Nó escravo — Arduino Uno (ATmega328P):** acionamento por DDA e malha
  fechada do punho sobre encoder AS5600. Controla as juntas **J4–J6**.

Os dois nós conversam por um protocolo serial binário de 10 bytes
(ver [`firmware/PROTOCOLO.md`](firmware/PROTOCOLO.md)).

---

## Arquitetura da co-simulação

Os firmwares **reais** são executados em emuladores e conectados ao simulador
robótico Webots por pontes elétricas em Python (relés que reproduzem os
barramentos físicos do protótipo):

```
  ESP32-S3 (QEMU) ──UART── Arduino Uno (simavr)
       │  ponte1_uart.py            │
       │ ponte2 (J1–J3)             │ ponte3 (J4–J6)
       └──────────► Webots ◄────────┘
                      │
                  benchmark.py ──RTDE── URSim (UR10e, referência)
```

- **Ponte 1** — barramento serial entre o ESP (QEMU) e o Uno (simavr).
- **Ponte 2** — troca síncrona (*lockstep*) Webots ↔ ESP (passo/encoder J1–J3).
- **Ponte 3** — aplicação do comando do punho ao modelo no Webots (J4–J6).
- **Benchmark** — orquestra o ensaio comparativo entre o EB15 e o URSim.

---

## Estrutura do repositório

```
firmware/
  esp32_mestre/        Firmware do nó mestre (ESP-IDF / C++)
    main/              main.cpp, pid_tanh.h, kinematics.h, trajectory.h, uart_master.h
  arduino_escravo/     Firmware do nó escravo (.ino + headers)
  qemu/                Scripts de build/emulação do ESP32 e ponte 1 (UART)
  simavr/              Runner do Uno no simavr (emulação AVR)
  tests/               Testes de unidade do protocolo e da matemática
  PROTOCOLO.md         Especificação congelada do frame UART mestre-escravo
webots/
  worlds/              Mundos do Webots (eb15_hil.wbt e variantes de ensaio)
  controllers/         Controladores Webots (ponte HIL)
  benchmark/           Orquestrador, trajetórias canônicas e clientes RTDE
  protos/              Modelo 3D do robô UR10e (ver licença abaixo)
  ponte2_webots.py     Ponte 2 (lockstep J1–J3)
  ponte3_webots.py     Ponte 3 (punho J4–J6)
  config.yaml          Fonte única de portas, frequências e parâmetros
  tests/               Testes de integração e smoke
helpers/
  protocol.py          Definição compartilhada do formato dos quadros
```

---

## Pré-requisitos

| Componente            | Ferramenta                                            |
|-----------------------|-------------------------------------------------------|
| Firmware ESP32-S3     | [ESP-IDF](https://docs.espressif.com/projects/esp-idf/) + [QEMU](https://www.qemu.org/) |
| Firmware Arduino Uno  | [arduino-cli](https://arduino.github.io/arduino-cli/) + [simavr](https://github.com/buserror/simavr) |
| Co-simulação / pontes | Python 3.11+                                          |
| Simulador robótico    | [Webots R2025a](https://cyberbotics.com/)             |
| Referência (opcional) | [URSim](https://www.universal-robots.com/download/) (RTDE) |

Dependências Python: `pip install pyyaml pyserial` (mais o módulo `controller`
do Webots, disponível na instalação do simulador).

---

## Como executar

> Os comandos abaixo assumem o diretório raiz do repositório. As portas e
> frequências são lidas de [`webots/config.yaml`](webots/config.yaml).

### 1. Compilar o firmware do ESP32-S3 (QEMU)

```powershell
powershell -ExecutionPolicy Bypass -File firmware/qemu/build_idf.ps1
```

### 2. Compilar e emular o firmware do Arduino Uno (simavr)

O `.ino` em `firmware/arduino_escravo/` é compilado com `arduino-cli` e
executado no simavr — **não** há reimplementação em Python do firmware.

### 3. Rodar o benchmark de co-simulação

O orquestrador sobe automaticamente os processos secundários (QEMU, simavr,
pontes e Webots) e os encerra ao final:

```bash
# Trajetória canônica no alvo EB15, sem interface gráfica
python webots/benchmark/benchmark.py --target eb15 --trajectory canonical --headless

# Com interface gráfica do Webots
python webots/benchmark/benchmark.py --target eb15 --trajectory canonical --no-headless
```

As telemetrias são gravadas como CSV conforme
`webots/benchmark/schemas/csv_schema.json`.

### 4. Testes

```bash
# Testes do firmware (protocolo + matemática)
powershell -ExecutionPolicy Bypass -File firmware/tests/run_tests.ps1

# Testes da co-simulação
pytest webots/tests
```

Detalhes adicionais da camada de simulação estão em
[`webots/README.md`](webots/README.md).

---

## Licença e créditos

O firmware, as pontes e a infraestrutura de co-simulação são de autoria de
Fernando Oliveira, no contexto do TCC.

O modelo 3D do manipulador **UR10e** em `webots/protos/` é propriedade da
**Universal Robots** e acompanha o simulador Webots, sendo usado neste projeto
apenas como referência cinemática/dinâmica, sob os termos de licença do
fabricante.
