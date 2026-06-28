# run_tests.ps1 - Compila e executa testes nativos da Parte A
param([string]$Gpp = "")

$ScriptDir   = $PSScriptRoot
$FirmwareDir = Split-Path -Parent $ScriptDir
$IncDir      = Join-Path $FirmwareDir "esp32_mestre\main"
$BuildDir    = Join-Path $ScriptDir "build"

if (-not $Gpp) {
    $GppCmd = Get-Command g++ -ErrorAction SilentlyContinue
    if ($GppCmd) { $Gpp = $GppCmd.Source }
}
if (-not $Gpp -or -not (Test-Path $Gpp)) {
    $Gpp = "C:\Program Files\Webots\msys64\mingw64\bin\g++.exe"
}
if (-not (Test-Path $Gpp)) {
    Write-Error "g++ nao encontrado."
    exit 1
}

New-Item -ItemType Directory -Force $BuildDir | Out-Null

Write-Host "=== EB15 - Testes Nativos de Firmware ===" -ForegroundColor Cyan

$Passed = 0; $Failed = 0

foreach ($TestName in @("validate_math", "protocol_core_test", "rtde_core_test")) {
    $Src = Join-Path $ScriptDir "$TestName.cpp"
    $Exe = Join-Path $BuildDir "$TestName.exe"

    Write-Host "Compilando $TestName.cpp..." -NoNewline
    & $Gpp -O2 -std=c++14 -Wall -Wextra -DEB15_NATIVE_TEST "-I$IncDir" -o $Exe $Src 2>&1 | Out-Null
    if ($LASTEXITCODE -ne 0) {
        Write-Host " ERRO" -ForegroundColor Red
        $Failed++
    } else {
        Write-Host " OK" -ForegroundColor Green
        $output = & $Exe 2>&1
        $output | ForEach-Object { Write-Host "  $_" }
        if ($LASTEXITCODE -eq 0) { Write-Host "  -> PASS" -ForegroundColor Green; $Passed++ }
        else                     { Write-Host "  -> FAIL" -ForegroundColor Red;   $Failed++ }
    }
}

# test_uno_bridge.py foi arquivado em Codigos/_legado_mock_v1/ (mock comportamental
# do Uno aposentado na migracao v2; o .ino real roda no simavr - ver changelog_v2.md).

Write-Host ""
if ($Failed -eq 0) {
    Write-Host "TODOS PASSARAM ($Passed testes)" -ForegroundColor Green
    exit 0
} else {
    Write-Host "FALHAS: $Failed" -ForegroundColor Red
    exit 1
}
