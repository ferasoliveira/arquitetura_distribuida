# launch_qemu.ps1 — Inicia emulacao QEMU do ESP32-S3 com firmware EB15 Mestre
#
# Uso:
#   .\launch_qemu.ps1                    # lanca emulacao interativa
#   .\launch_qemu.ps1 -Headless          # sem janela grafica (apenas serial stdio)
#   .\launch_qemu.ps1 -WaitReady         # aguarda mensagem [BOOT OK] no serial
#
# Pre-requisitos:
#   1. Firmware compilado com build_idf.ps1 -Qemu
#   2. QEMU Espressif instalado:
#      https://github.com/espressif/qemu/releases
#   3. Variavel QEMU_ESP_PATH apontando para qemu-system-xtensa.exe
#      Exemplo: $env:QEMU_ESP_PATH = "C:\tools\qemu-esp\bin\qemu-system-xtensa.exe"
#
# Port-forwarding TCP (acesso do host ao firmware emulado):
#   - :8080  -> porta HTTP REST do ESP32
#   - :30003 -> porta RTDE-EB15 (streaming de telemetria)
#   - :30101 -> ponte UART2 para Arduino Uno emulado (uno_bridge.py)
#
# Serial: redirecionado para stdio (logs do firmware visiveis no terminal).

param(
    [switch]$Headless  = $false,
    [switch]$WaitReady = $false,
    [switch]$AllowStale = $false,
    [string]$AsciiRunDir = "C:\tmp\eb15_qemu_run"
)

$ErrorActionPreference = "Stop"

$FirmwareDir = Split-Path -Parent $PSScriptRoot
$ProjectDir  = Join-Path $FirmwareDir "esp32_mestre"
$BuildDir    = Join-Path $ProjectDir ".build"
$BinPath     = Join-Path $BuildDir "eb15_mestre.bin"

# Localiza o binario mergeado (bootloader + partition table + app)
$MergedBin = Join-Path $BuildDir "eb15_mestre.merged.bin"
if (-not (Test-Path $MergedBin)) {
    # Tenta o binario padrao
    if (-not (Test-Path $BinPath)) {
        Write-Error "Binario nao encontrado. Execute build_idf.ps1 -Qemu primeiro."
        exit 1
    }
    $MergedBin = $BinPath
}

# Impede que uma imagem antiga masque alteracoes recentes do firmware.
$FirmwareSources = Get-ChildItem $ProjectDir -Recurse -File |
    Where-Object { $_.Extension -in @(".c", ".cpp", ".h", ".hpp", ".txt") }
$NewestSource = $FirmwareSources | Sort-Object LastWriteTime -Descending | Select-Object -First 1
$ImageInfo = Get-Item $MergedBin
if (-not $AllowStale -and $NewestSource -and $NewestSource.LastWriteTime -gt $ImageInfo.LastWriteTime) {
    Write-Error "Imagem QEMU desatualizada: $($NewestSource.FullName) e mais recente que $MergedBin. Execute build_idf.ps1 -Qemu."
    exit 1
}

# QEMU/Start-Process no Windows nao preserva com confiabilidade argumentos de
# -drive com espacos e caracteres Unicode. Executa sempre a partir de ASCII.
New-Item -ItemType Directory -Path $AsciiRunDir -Force | Out-Null
$RuntimeImage = Join-Path $AsciiRunDir "eb15_mestre.merged.bin"
Copy-Item -LiteralPath $MergedBin -Destination $RuntimeImage -Force
$MergedBin = $RuntimeImage

# Localiza o QEMU Espressif
$QemuExe = $env:QEMU_ESP_PATH
if (-not $QemuExe) {
    # Tenta localizacao padrao
    $Candidates = @(
        "C:\tools\qemu-esp\bin\qemu-system-xtensa.exe",
        "C:\Program Files\qemu-esp\bin\qemu-system-xtensa.exe",
        "$env:USERPROFILE\.espressif\tools\qemu-esp32\bin\qemu-system-xtensa.exe",
        "$env:USERPROFILE\.espressif\tools\qemu-xtensa\esp_develop_9.2.2_20250817\qemu\bin\qemu-system-xtensa.exe"
    )
    foreach ($c in $Candidates) {
        if (Test-Path $c) { $QemuExe = $c; break }
    }
}
if (-not $QemuExe -or -not (Test-Path $QemuExe)) {
    Write-Error @"
QEMU Espressif nao encontrado.
Defina: `$env:QEMU_ESP_PATH = '<caminho>\qemu-system-xtensa.exe'
Download: https://github.com/espressif/qemu/releases
"@
    exit 1
}

Write-Host "=== EB15 Mestre - Emulacao QEMU ===" -ForegroundColor Cyan
Write-Host "Binario  : $MergedBin"
Write-Host "SHA-256  : $((Get-FileHash $MergedBin -Algorithm SHA256).Hash)"
Write-Host "QEMU     : $QemuExe"
Write-Host ""
Write-Host "Port-forwarding TCP:" -ForegroundColor Yellow
Write-Host "  :8080  -> HTTP REST ESP32"
Write-Host "  :30003 -> RTDE-EB15"
Write-Host "  :30101 -> UART2 (Arduino Uno bridge)"
Write-Host ""

$QemuArgs = @(
    "-machine",  "esp32s3",
    "-nographic",
    "-drive",    "file=$MergedBin,if=mtd,format=raw",
    # UART0 -> stdio (logs do firmware / console IDF)
    "-serial",   "stdio",
    # UART1 -> TCP :30101 (bridge para Arduino Uno emulado por uno_bridge.py)
    # uno_bridge.py conecta como cliente a este servidor TCP
    # Nota: no hardware real usa-se UART2 (GPIO19/20); QEMU usa UART1 (mais confiável)
    "-serial",   "tcp::30100,server,nowait",
    # O modelo esp32s3 cria a controladora OpenCores internamente. -nic associa
    # o backend SLIRP a ela; o firmware usa CONFIG_ETH_USE_OPENETH no QEMU.
    "-nic",      "user,model=open_eth,hostfwd=tcp::8080-:80,hostfwd=tcp::30003-:30003,hostfwd=tcp::30101-:30101"
)

if ($WaitReady) {
    Write-Host "Aguardando [NETWORK OK] no serial..." -ForegroundColor Green
    $StdoutLog = Join-Path $AsciiRunDir "qemu_stdout.log"
    $StderrLog = Join-Path $AsciiRunDir "qemu_stderr.log"
    $proc = Start-Process -FilePath $QemuExe -ArgumentList $QemuArgs `
                          -PassThru -WindowStyle Hidden `
                          -RedirectStandardOutput $StdoutLog `
                          -RedirectStandardError $StderrLog

    $deadline = (Get-Date).AddSeconds(30)
    $ready = $false
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Milliseconds 500
        if ($proc.HasExited) {
            $details = Get-Content $StderrLog -Raw -ErrorAction SilentlyContinue
            Write-Error "QEMU encerrou antes do boot (codigo $($proc.ExitCode)): $details"
            exit 1
        }
        if (Test-Path $StdoutLog) {
            $content = Get-Content $StdoutLog -Raw -ErrorAction SilentlyContinue
            if ($content -match "\[NETWORK OK\]") {
                Write-Host "QEMU iniciado com sucesso - rede do firmware respondendo." -ForegroundColor Green
                $ready = $true
                break
            }
        }
    }
    if (-not $ready) {
        Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
        $details = Get-Content $StderrLog -Raw -ErrorAction SilentlyContinue
        Write-Error "Timeout aguardando [NETWORK OK] do QEMU. $details"
        exit 1
    }
} else {
    Write-Host "Iniciando QEMU (Ctrl+A X para encerrar)..." -ForegroundColor Green
    & $QemuExe @QemuArgs
}
