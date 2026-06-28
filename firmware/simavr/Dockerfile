# EB15 — Runner do firmware escravo (Arduino UNO .ino real) no simavr
# Arquitetura canônica v2 (wiki/plano_acao_v2.md): o .elf compilado por arduino-cli
# roda cycle-accurate no simavr dentro deste container; o harness C (uno_runner)
# expoe UART, pinos Step/Dir/Trigger e o I2C bit-bang (AS5600) via TCP para as
# pontes eletricas Python no host. NENHUM mock/reimplementacao de firmware.
FROM debian:12-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
        simavr \
        libsimavr-dev \
        libelf-dev \
        gcc \
        make \
        libc6-dev \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
# O harness uno_runner.c e o .elf sao montados/copidados em etapas seguintes.
CMD ["bash"]
