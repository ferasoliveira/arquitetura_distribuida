# smoke.ps1 — Build + smoke test do firmware escravo REAL (.ino .elf) no simavr.
#
# Reproduz a Etapa 2 do plano_acao_v2: compila o harness uno_runner.c contra
# libsimavr (no container eb15-simavr) e executa o self-test (--smoke):
#   frame valido -> ACK + passos ; checksum invalido -> NAK ; trigger -> DDA ; DONE.
#
# Pre-condicoes: imagem Docker eb15-simavr:latest construida
#   (cd Códigos\firmware\simavr ; docker build -t eb15-simavr:latest .)
#   e o .elf do escravo compilado (arduino-cli).
#
# Uso: .\smoke.ps1 [-Elf <caminho.elf>]

param(
  [string]$Elf = "C:\Users\ferna\OneDrive\Área de Trabalho\TCC_Fernando\Códigos\firmware\arduino_escravo\build\arduino_escravo.ino.elf",
  [string]$AsciiDir = "C:\eb15_sim"
)

$ErrorActionPreference = "Continue"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path

New-Item -ItemType Directory -Force $AsciiDir | Out-Null
Copy-Item "$here\uno_runner.c" "$AsciiDir\uno_runner.c" -Force
Copy-Item $Elf "$AsciiDir\arduino_escravo.ino.elf" -Force

Write-Host "=== build + smoke no container eb15-simavr ===" -ForegroundColor Cyan
$env:MSYS_NO_PATHCONV = "1"
docker run --rm -v "${AsciiDir}:/work" eb15-simavr:latest bash -c @"
cd /work
gcc -O2 -Wall -o uno_runner uno_runner.c -lsimavr -lsimavrparts -lpthread -lelf || { echo 'BUILD FALHOU'; exit 3; }
timeout 30 ./uno_runner --elf arduino_escravo.ino.elf --smoke
echo RUNNER_EXIT=\$?
"@
