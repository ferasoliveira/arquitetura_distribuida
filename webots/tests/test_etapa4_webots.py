"""Validação observável da Etapa 4 (metade Uno/Webots), sem mock.

Executa ponte 2, ponte 3, Webots real e um simavr fresco. O estímulo UART
envia um frame ao `.ino` real; o teste exige ACK, passos, DONE, alteração do
PositionSensor devolvida em :30104 e leituras I2C AS5600 no log do runner.
"""
from __future__ import annotations

import hashlib
import os
from pathlib import Path
import shutil
import socket
import struct
import subprocess
import sys
import time
import urllib.request
import urllib.error

ROOT = Path(__file__).resolve().parents[3]
WEBOTS_DIR = ROOT / "Códigos" / "webots"
PYTHON = sys.executable
WEBOTS_EXE = Path(r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe")
STAGE = Path(r"C:\tmp\eb15_webots_etapa4")
CONTAINER = "eb15_simavr_serve"
QEMU_EXE = Path.home() / ".espressif" / "tools" / "qemu-xtensa" / "esp_develop_9.2.2_20250817" / "qemu" / "bin" / "qemu-system-xtensa.exe"


def wait_port(port: int, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                return
        except OSError:
            time.sleep(0.1)
    raise TimeoutError(f"porta :{port} indisponivel")


def read_encoder(port: int) -> tuple[int, int, int]:
    with socket.create_connection(("127.0.0.1", port), timeout=1) as sock:
        sock.sendall(b"\x01")
        data = b""
        while len(data) < 6:
            part = sock.recv(6 - len(data))
            if not part:
                break
            data += part
    if len(data) != 6:
        raise RuntimeError(f"encoder :{port} retornou {len(data)} bytes")
    return struct.unpack("<3H", data)


def wait_encoder(port: int, timeout: float) -> tuple[int, int, int]:
    deadline = time.monotonic() + timeout
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            return read_encoder(port)
        except (OSError, RuntimeError) as exc:
            last_error = exc
            time.sleep(0.2)
    raise TimeoutError(f"encoder :{port} sem ground truth: {last_error}")


def make_frame(j4: int, j5: int, j6: int, duration_ms: int) -> bytes:
    body = struct.pack("<BhhhH", 0xAA, j4, j5, j6, duration_ms)
    checksum = 0
    for byte in body:
        checksum ^= byte
    return body + bytes([checksum])


def port_owner(port: int) -> str:
    cmd = (
        f"$c=Get-NetTCPConnection -State Listen -LocalPort {port} "
        "-ErrorAction Stop | Select-Object -First 1; "
        '"PID=$($c.OwningProcess) NAME=$((Get-Process -Id $c.OwningProcess).ProcessName)"'
    )
    return subprocess.check_output(["powershell", "-NoProfile", "-Command", cmd], text=True).strip()


def http_post(path: str, data: bytes, attempts: int = 8, timeout: float = 8.0) -> bytes:
    """POST resiliente. O EMAC OpenEth do QEMU dropa frames RX intermitentemente
    (W opencores.emac: RX frame dropped) — uma requisicao pode se perder no
    transporte. Reenvia ate obter HTTP 200. Nao mascara movimento: so repassa
    o comando real ao ESP real; o movimento ainda e medido depois."""
    last = None
    for i in range(attempts):
        try:
            req = urllib.request.Request(f"http://127.0.0.1:8080{path}", data=data, method="POST")
            with urllib.request.urlopen(req, timeout=timeout) as response:
                if response.status == 200:
                    return response.read()
                last = f"HTTP {response.status}"
        except (urllib.error.URLError, OSError, TimeoutError) as exc:
            last = repr(exc)
        time.sleep(0.5)
    raise RuntimeError(f"POST {path} falhou apos {attempts} tentativas: {last}")


def main() -> int:
    processes: list[subprocess.Popen] = []
    logs = []
    try:
        subprocess.run(["docker", "rm", "-f", CONTAINER], capture_output=True)
        subprocess.run(["taskkill", "/F", "/IM", "qemu-system-xtensa.exe"], capture_output=True)
        if STAGE.exists():
            shutil.rmtree(STAGE)
        (STAGE / "worlds").mkdir(parents=True)
        (STAGE / "controllers").mkdir()
        shutil.copytree(WEBOTS_DIR / "protos", STAGE / "protos")
        shutil.copytree(WEBOTS_DIR / "controllers" / "hil_bridge", STAGE / "controllers" / "hil_bridge")
        python_path = str(PYTHON).replace("\\", "/")
        (STAGE / "controllers" / "hil_bridge" / "runtime.ini").write_text(
            f"[python]\nCOMMAND = {python_path}\n", encoding="utf-8")
        shutil.copy2(WEBOTS_DIR / "worlds" / "eb15_hil.wbt", STAGE / "worlds" / "eb15_hil.wbt")

        for script, name in (("ponte2_webots.py", "ponte2"), ("ponte3_webots.py", "ponte3")):
            handle = open(STAGE / f"{name}.log", "w", encoding="utf-8")
            logs.append(handle)
            processes.append(subprocess.Popen(
                [PYTHON, str(WEBOTS_DIR / script)], stdout=handle, stderr=subprocess.STDOUT))

        webots_log = open(STAGE / "webots.log", "w", encoding="utf-8")
        logs.append(webots_log)
        # Headless no Windows: --mode=fast (desacopla o stepping do timer de
        # wall-clock do GUI, que fica faminto sem display e trava o loop em
        # realtime) + --minimize (garante janela Win32 com message pump) +
        # --no-rendering. NAO usar QT_QPA_PLATFORM=offscreen no Windows: o
        # plugin Qt offscreen pode impedir o Webots de iniciar.
        env_webots = os.environ.copy()
        python_dir = os.path.dirname(PYTHON)
        env_webots["PATH"] = f"{python_dir};{env_webots.get('PATH', '')}"
        processes.append(subprocess.Popen([
            str(WEBOTS_EXE), "--batch", "--mode=fast", "--minimize", "--no-rendering",
            "--stdout", "--stderr",
            str(STAGE / "worlds" / "eb15_hil.wbt")], stdout=webots_log, stderr=subprocess.STDOUT, env=env_webots))

        for port in (30103, 30104, 30301, 30302):
            wait_port(port, 30)

        subprocess.run([
            "docker", "run", "-d", "--rm", "--name", CONTAINER,
            "-p", "30200:30200", "-p", "30201:30201", "-p", "30202:30202", "-p", "30203:30203",
            "-v", r"C:\eb15_sim:/work", "eb15-simavr:latest", "bash", "-c",
            "cd /work && ./uno_runner --elf arduino_escravo.ino.elf --serve"
        ], check=True)
        wait_port(30200, 30)
        wait_port(30201, 10)

        before = wait_encoder(30104, 30)
        before_esp = wait_encoder(30103, 30)

        qemu_image = STAGE / "eb15_mestre.merged.bin"
        shutil.copy2(ROOT / "Códigos" / "firmware" / "esp32_mestre" / ".build" / "eb15_mestre.merged.bin", qemu_image)
        qemu_log = open(STAGE / "qemu.log", "w", encoding="utf-8")
        logs.append(qemu_log)
        # stdin=DEVNULL é CRÍTICO: o QEMU -nographic toma posse do stdin herdado
        # (usa-o para o monitor/console). Sem isolar o stdin, qualquer processo
        # filho spawnado DEPOIS (a ponte 1) que herda o mesmo handle de stdin
        # trava na inicialização e nunca conecta — causando ACK timeout/E-STOP.
        processes.append(subprocess.Popen([
            str(QEMU_EXE), "-machine", "esp32s3", "-nographic",
            "-drive", f"file={qemu_image},if=mtd,format=raw",
            "-serial", f"file:{STAGE / 'qemu_serial.log'}",
            "-serial", "tcp::30100,server,nowait",
            "-nic", "user,model=open_eth,hostfwd=tcp::8080-:80,hostfwd=tcp::30003-:30003,hostfwd=tcp::30101-:30101",
        ], stdout=qemu_log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL))
        # CRÍTICO: a ponte 1 só pode conectar a :30100 DEPOIS de o ESP terminar o
        # boot. Ao inicializar o driver UART_NUM_1, o ESP/QEMU reseta a conexão do
        # chardev :30100; se a ponte 1 conectar durante o boot, sua conexão cai e o
        # relé morre sem reconectar (Uno nunca recebe o frame -> ACK timeout ->
        # E-STOP). Por isso espera-se :8080 (HTTP ativo = boot concluído) primeiro.
        wait_port(8080, 60)
        time.sleep(2)  # margem para o driver UART1 estabilizar após o boot

        ponte1_log = open(STAGE / "ponte1.log", "w", encoding="utf-8")
        logs.append(ponte1_log)
        ponte1_self_log = STAGE / "ponte1_self.log"
        ponte1_env = os.environ.copy()
        ponte1_env["EB15_PONTE1_LOG"] = str(ponte1_self_log)  # log confiavel via FileHandler
        ponte1_proc = subprocess.Popen([
            PYTHON, "-u", str(ROOT / "Códigos" / "firmware" / "qemu" / "ponte1_uart.py")
        ], stdout=ponte1_log, stderr=subprocess.STDOUT, env=ponte1_env, stdin=subprocess.DEVNULL)
        processes.append(ponte1_proc)
        # Confirma que a ponte 1 conectou nas 3 pontas (QEMU/simavr UART+CTRL)
        # antes de comandar o movimento — senão o frame J4-J6 não é relayado.
        deadline_p1 = time.monotonic() + 30
        while time.monotonic() < deadline_p1:
            if ponte1_self_log.exists() and "ponte 1 ATIVA" in ponte1_self_log.read_text(
                    encoding="utf-8", errors="replace"):
                break
            time.sleep(0.3)
        else:
            raise RuntimeError("ponte 1 nao ficou ATIVA (nao conectou QEMU/simavr UART)")

        mode_body = http_post("/mode", b"user")
        if mode_body != b"USER":
            raise RuntimeError(f"nao foi possivel colocar o ESP em modo USER: {mode_body!r}")
        move_body = http_post("/move_j", b"20,0,0,20,0,0,30")
        if move_body != b"OK":
            raise RuntimeError(f"/move_j falhou: {move_body!r}")

        deadline = time.monotonic() + 60
        after = before
        after_esp = before_esp
        while time.monotonic() < deadline:
            after = wait_encoder(30104, 2)
            after_esp = wait_encoder(30103, 2)
            if after[0] != before[0] and after_esp[0] != before_esp[0]:
                break
            time.sleep(0.5)
        if after[0] == before[0] or after_esp[0] == before_esp[0]:
            raise RuntimeError(
                f"movimento incompleto: J1 {before_esp}->{after_esp}; J4 {before}->{after}")

        webots_pids = subprocess.check_output(
            ["powershell", "-NoProfile", "-Command", "(Get-Process webots-bin -ErrorAction Stop).Id"],
            text=True).strip().splitlines()
        owners = {port: port_owner(port) for port in (30103, 30104)}
        runner_log = subprocess.check_output(["docker", "logs", CONTAINER], text=True, stderr=subprocess.STDOUT)
        if "[i2c] AS5600 J4" not in runner_log:
            raise RuntimeError("log nao comprova leitura I2C AS5600 J4")
        ponte1_log.flush()
        ponte1_text = (STAGE / "ponte1.log").read_text(encoding="utf-8", errors="replace")
        if "ACK(0x06)" not in ponte1_text or "DONE(0x04)" not in ponte1_text:
            raise RuntimeError("ponte 1 nao registrou ACK e DONE reais do Uno")
        evidence = ROOT / "Códigos" / "firmware" / "simavr" / "evidencia_etapa4_webots.log"
        text = (
            f"WEBOTS_PIDS={webots_pids}\nOWNERS={owners}\n"
            f"ENCODER_J1_BEFORE={before_esp}\nENCODER_J1_AFTER={after_esp}\n"
            f"ENCODER_J4_BEFORE={before}\nENCODER_J4_AFTER={after}\n"
            f"HTTP_MOVE_J=200 OK\n\n--- PONTE1 ---\n{ponte1_text}\n--- SIMAVR ---\n{runner_log}"
        )
        evidence.write_text(text, encoding="utf-8")
        digest = hashlib.sha256(evidence.read_bytes()).hexdigest().upper()
        print(f"WEBOTS_PIDS={webots_pids}")
        print(f"OWNERS={owners}")
        print(f"ENCODER_J1_BEFORE={before_esp} ENCODER_J1_AFTER={after_esp}")
        print(f"ENCODER_J4_BEFORE={before} ENCODER_J4_AFTER={after}")
        print(f"EVIDENCE={evidence} SHA256={digest}")
        print("RESULT: PASS (QEMU .cpp + simavr .ino + 3 pontes + Webots real)")
        return 0
    except Exception as exc:
        print(f"RESULT: FAIL: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            rlog = subprocess.run(["docker", "logs", CONTAINER], capture_output=True, text=True)
            (STAGE / "runner.log").write_text(
                (rlog.stdout or "") + (rlog.stderr or ""), encoding="utf-8", errors="replace")
        except Exception:
            pass
        subprocess.run(["docker", "rm", "-f", CONTAINER], capture_output=True)
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
        for handle in logs:
            handle.close()


if __name__ == "__main__":
    raise SystemExit(main())
