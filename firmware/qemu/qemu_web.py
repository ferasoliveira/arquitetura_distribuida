#!/usr/bin/env python3
"""
qemu_web.py — Servidor HTTP local para interface web do EB15 em modo QEMU.

Serve em http://localhost:8080 uma página HTML de monitoramento/controle.
Em hardware real, o ESP32-S3 serve o próprio HTTP server em porta 80.

Uso:
    python qemu_web.py [--port 8080]
"""

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

# Estado compartilhado (atualizado por uno_bridge via socket ou internamente)
_state = {
    "seg_ack":   0,
    "seg_done":  0,
    "seg_total": 33,
    "estop":     False,
    "q":         [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "mode":      "QEMU",
    "bridge_ok": False,
}
_lock = threading.Lock()

HTML_PAGE = r"""<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>EB15 — Painel de Controle (QEMU)</title>
<style>
  :root { --green:#2ecc71; --red:#e74c3c; --blue:#3498db; --bg:#1a1a2e; --card:#16213e; --text:#eee; }
  * { box-sizing:border-box; margin:0; padding:0; }
  body { background:var(--bg); color:var(--text); font-family:'Segoe UI',sans-serif; padding:20px; }
  h1 { color:var(--blue); margin-bottom:4px; }
  .subtitle { color:#888; font-size:0.85em; margin-bottom:20px; }
  .grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(260px,1fr)); gap:16px; }
  .card { background:var(--card); border-radius:10px; padding:20px; }
  .card h2 { font-size:0.9em; text-transform:uppercase; letter-spacing:1px; color:#888; margin-bottom:12px; }
  .value { font-size:2em; font-weight:700; color:var(--blue); }
  .status { display:inline-block; padding:4px 12px; border-radius:20px; font-size:0.85em; font-weight:600; }
  .ok   { background:#1e4d3a; color:var(--green); }
  .warn { background:#4d3a1e; color:#f39c12; }
  .err  { background:#4d1e1e; color:var(--red); }
  .progress-bar { background:#0d0d1a; border-radius:8px; height:18px; margin-top:8px; overflow:hidden; }
  .progress-fill { height:100%; background:var(--blue); border-radius:8px; transition:width 0.4s; }
  .joints { display:grid; grid-template-columns:repeat(3,1fr); gap:8px; margin-top:8px; }
  .joint { background:#0d0d1a; border-radius:6px; padding:8px; text-align:center; }
  .joint label { font-size:0.7em; color:#888; display:block; }
  .joint span  { font-size:1.1em; font-weight:600; color:var(--blue); }
  button { border:none; border-radius:6px; padding:10px 20px; font-size:0.9em;
           font-weight:600; cursor:pointer; transition:opacity 0.2s; }
  button:hover { opacity:0.85; }
  .btn-blue  { background:var(--blue); color:#fff; }
  .btn-red   { background:var(--red);  color:#fff; }
  .btn-green { background:var(--green);color:#000; }
  .actions   { display:flex; gap:8px; flex-wrap:wrap; margin-top:12px; }
  .log { font-family:monospace; font-size:0.78em; background:#0d0d1a;
         border-radius:6px; padding:10px; max-height:160px; overflow-y:auto;
         white-space:pre-wrap; color:#aaa; margin-top:8px; }
  #timestamp { font-size:0.75em; color:#555; margin-top:4px; }
</style>
</head>
<body>
<h1>EB15 — Braço Robótico 6-DOF</h1>
<p class="subtitle">Painel de Monitoramento &amp; Controle · Modo QEMU · ESP32-S3 Mestre</p>

<div class="grid">

  <!-- Status Geral -->
  <div class="card">
    <h2>Status do Sistema</h2>
    <span id="estop-badge" class="status ok">NORMAL</span>
    <br><br>
    <table style="width:100%;font-size:0.85em;border-collapse:collapse;">
      <tr><td style="color:#888;padding:3px 0">Bridge UART</td>
          <td id="bridge-status" style="text-align:right">—</td></tr>
      <tr><td style="color:#888;padding:3px 0">Modo</td>
          <td id="mode-val" style="text-align:right;color:var(--blue)">QEMU</td></tr>
    </table>
    <p id="timestamp"></p>
  </div>

  <!-- Progresso da Trajetória -->
  <div class="card">
    <h2>Trajetória MoveJ</h2>
    <span class="value"><span id="seg-done">0</span> / <span id="seg-total">33</span></span>
    <span style="color:#888;font-size:0.8em;margin-left:6px">segmentos</span>
    <div class="progress-bar">
      <div class="progress-fill" id="prog-fill" style="width:0%"></div>
    </div>
    <p style="margin-top:8px;font-size:0.8em;color:#888">
      ACK recebidos: <span id="seg-ack">0</span>
    </p>
  </div>

  <!-- Ângulos das Juntas -->
  <div class="card">
    <h2>Ângulos das Juntas (°)</h2>
    <div class="joints">
      <div class="joint"><label>J1</label><span id="q0">0.0</span></div>
      <div class="joint"><label>J2</label><span id="q1">0.0</span></div>
      <div class="joint"><label>J3</label><span id="q2">0.0</span></div>
      <div class="joint"><label>J4</label><span id="q3">0.0</span></div>
      <div class="joint"><label>J5</label><span id="q4">0.0</span></div>
      <div class="joint"><label>J6</label><span id="q5">0.0</span></div>
    </div>
  </div>

  <!-- Controles -->
  <div class="card">
    <h2>Controles</h2>
    <div class="actions">
      <button class="btn-green" onclick="sendMove()">&#9654; Demo MoveJ</button>
      <button class="btn-red"   onclick="sendEstop()">&#9632; E-STOP</button>
    </div>
    <div class="log" id="log">Sistema iniciado...\n</div>
  </div>

</div>

<script>
function addLog(msg) {
  var el = document.getElementById('log');
  el.textContent += new Date().toLocaleTimeString('pt-BR') + ' ' + msg + '\n';
  el.scrollTop = el.scrollHeight;
}

function poll() {
  fetch('/status').then(r => r.json()).then(d => {
    document.getElementById('seg-done').textContent  = d.seg_done;
    document.getElementById('seg-total').textContent = d.seg_total;
    document.getElementById('seg-ack').textContent   = d.seg_ack;
    var pct = d.seg_total > 0 ? (d.seg_done / d.seg_total * 100).toFixed(1) : 0;
    document.getElementById('prog-fill').style.width = pct + '%';
    document.getElementById('mode-val').textContent  = d.mode;
    document.getElementById('bridge-status').textContent = d.bridge_ok ? '✔ Conectado' : '✘ Aguardando';
    document.getElementById('bridge-status').style.color = d.bridge_ok ? 'var(--green)' : 'var(--red)';
    ['q0','q1','q2','q3','q4','q5'].forEach((id,i) => {
      document.getElementById(id).textContent = (d.q[i]||0).toFixed(2);
    });
    var badge = document.getElementById('estop-badge');
    if (d.estop) { badge.textContent='E-STOP'; badge.className='status err'; }
    else         { badge.textContent='NORMAL'; badge.className='status ok'; }
    document.getElementById('timestamp').textContent = 'Atualizado: ' + new Date().toLocaleTimeString('pt-BR');
  }).catch(() => {});
}

function sendMove() {
  addLog('[→] Enviando demo MoveJ...');
  fetch('/move_j', {method:'POST', body:'30,20,-15,45,0,0,50',
                   headers:{'Content-Type':'text/plain'}})
    .then(r => r.text()).then(t => addLog('[←] ' + t))
    .catch(e => addLog('[!] Erro: ' + e));
}

function sendEstop() {
  addLog('[!] E-STOP enviado');
  fetch('/estop', {method:'POST'})
    .then(r => r.text()).then(t => addLog('[←] ' + t))
    .catch(e => addLog('[!] Erro: ' + e));
}

setInterval(poll, 500);
poll();
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def address_string(self):
        return str(self.client_address[0])

    def log_message(self, fmt, *args):
        pass  # silencia log do servidor

    def _send_json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", len(body))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _send_text(self, text, code=200):
        body = text.encode()
        self.send_response(code)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", len(body))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            body = HTML_PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", len(body))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/status":
            with _lock:
                self._send_json(dict(_state))
        else:
            self._send_text("Not found", 404)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body   = self.rfile.read(length).decode(errors="replace") if length else ""

        if self.path == "/move_j":
            self._send_text("RTDE/QEMU bridge not connected; command not sent", 503)
        elif self.path == "/estop":
            with _lock:
                _state["estop"] = True
            self._send_text("ESTOP")
        elif self.path == "/bridge_update":
            try:
                data = json.loads(body)
                with _lock:
                    _state.update(data)
                self._send_text("OK")
            except Exception:
                self._send_text("bad json", 400)
        else:
            self._send_text("Not found", 404)


def update_state(**kwargs):
    """Chamado externamente (ex: uno_bridge.py) para atualizar estado."""
    with _lock:
        _state.update(kwargs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    server = HTTPServer(("0.0.0.0", args.port), Handler)
    print(f"[WEB] EB15 painel disponivel em http://localhost:{args.port}")
    print(f"[WEB] Ctrl+C para encerrar")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[WEB] Servidor encerrado")


if __name__ == "__main__":
    main()
