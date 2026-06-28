# build_idf.ps1 — Compila o firmware ESP32-S3 Mestre com ESP-IDF
#
# Uso:
#   .\build_idf.ps1              # build normal
#   .\build_idf.ps1 -Qemu        # build com flag EB15_QEMU=1 (para emulacao QEMU)
#   .\build_idf.ps1 -Clean       # limpa e recompila
#
# Requer:
#   - ESP-IDF instalado e configurado (idf_exports.ps1 ou idf.ps1 no PATH)
#   - Variavel IDF_PATH definida
#   - Python 3.x no PATH
#
# Saida: firmware/esp32_mestre/.build/eb15_mestre.bin (e .elf, .merged.bin)

param(
    [switch]$Qemu   = $false,
    [switch]$Clean  = $false,
    [string]$AsciiWorkDir = "C:\tmp\eb15_esp32_qemu"
)

$ErrorActionPreference = "Stop"

$FirmwareDir = Split-Path -Parent $PSScriptRoot
$ProjectDir  = Join-Path $FirmwareDir "esp32_mestre"
if (-not (Test-Path $ProjectDir)) {
    Write-Error "Pasta do projeto nao encontrada: $ProjectDir"
    exit 1
}
$OutputBuildDir = Join-Path $ProjectDir ".build"
$NeedsAsciiStaging = $ProjectDir.ToCharArray() | Where-Object { [int]$_ -gt 127 } | Select-Object -First 1
if ($NeedsAsciiStaging) {
    $StagingProjectDir = Join-Path $AsciiWorkDir "esp32_mestre"
    if ($Clean -and (Test-Path $StagingProjectDir)) {
        Remove-Item -LiteralPath $StagingProjectDir -Recurse -Force
    }
    New-Item -ItemType Directory -Path $StagingProjectDir -Force | Out-Null
    Get-ChildItem -LiteralPath $ProjectDir -Force |
        Where-Object { $_.Name -ne ".build" } |
        Copy-Item -Destination $StagingProjectDir -Recurse -Force
    $ProjectDir = $StagingProjectDir
    Write-Host "Workspace possui caracteres nao ASCII; build em staging: $ProjectDir" -ForegroundColor Yellow
}
$BuildDir = Join-Path $ProjectDir ".build"

if (-not $env:IDF_PATH) {
    Write-Error "Variavel IDF_PATH nao definida. Execute o script de exportacao do ESP-IDF primeiro."
    exit 1
}

Write-Host "=== EB15 Mestre - Build ESP-IDF ===" -ForegroundColor Cyan
Write-Host "Projeto  : $ProjectDir"
Write-Host "Build    : $BuildDir"
Write-Host "QEMU mode: $Qemu"

Set-Location $ProjectDir

if ($Clean -and (Test-Path $BuildDir)) {
    Write-Host "Limpando build anterior..." -ForegroundColor Yellow
    Remove-Item -Recurse -Force $BuildDir
}

# Opcoes globais precisam vir antes do subcomando no idf.py 5.x.
$IdfArgs = @("-B", $BuildDir)
if ($Qemu) {
    $IdfArgs += @("-DEB15_QEMU=1")
}
$IdfArgs += "build"

Write-Host "Executando: idf.py $($IdfArgs -join ' ')" -ForegroundColor Green
& python "$env:IDF_PATH\tools\idf.py" @IdfArgs

if ($LASTEXITCODE -ne 0) {
    Write-Error "Build FALHOU (codigo $LASTEXITCODE)"
    exit $LASTEXITCODE
}

$BinPath = Join-Path $BuildDir "eb15_mestre.bin"
if (-not (Test-Path $BinPath)) {
    Write-Warning "Binario nao encontrado em $BinPath - verifique o nome do projeto."
    exit 1
}

$Size = (Get-Item $BinPath).Length
Write-Host "BUILD OK: $BinPath ($Size bytes)" -ForegroundColor Green

# Gera imagem mesclada (bootloader + partition table + app) para o QEMU
$MergedBin  = Join-Path $BuildDir "eb15_mestre.merged.bin"
$Bootloader = Join-Path $BuildDir "bootloader\bootloader.bin"
$PartTable  = Join-Path $BuildDir "partition_table\partition-table.bin"

Write-Host "Gerando imagem mesclada para QEMU..." -ForegroundColor Yellow
$EsptoolArgs = @(
    "-m", "esptool",
    "--chip", "esp32s3",
    "merge_bin",
    "--fill-flash-size", "2MB",
    "--output", $MergedBin,
    "--target-offset", "0x0",
    "0x0",       $Bootloader,
    "0x8000",    $PartTable,
    "0x10000",   $BinPath
)
& python @EsptoolArgs
if ($LASTEXITCODE -eq 0 -and (Test-Path $MergedBin)) {
    $MergedSize = (Get-Item $MergedBin).Length
    Write-Host "MERGED OK: $MergedBin ($MergedSize bytes)" -ForegroundColor Green
    if ($NeedsAsciiStaging) {
        New-Item -ItemType Directory -Path $OutputBuildDir -Force | Out-Null
        foreach ($Artifact in @("eb15_mestre.bin", "eb15_mestre.elf", "eb15_mestre.map", "eb15_mestre.merged.bin")) {
            $ArtifactPath = Join-Path $BuildDir $Artifact
            if (Test-Path $ArtifactPath) {
                Copy-Item -LiteralPath $ArtifactPath -Destination $OutputBuildDir -Force
            }
        }
        Write-Host "Artefatos copiados para: $OutputBuildDir" -ForegroundColor Green
    }
    Write-Host "  Para emular: .\launch_qemu.ps1" -ForegroundColor Cyan
} else {
    Write-Error "Falha ao gerar imagem mesclada para QEMU. O build nao e utilizavel para emulacao."
    exit 1
}
