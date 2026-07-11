#!/usr/bin/env python3
"""
DEOS Log Monitor - Web tabanlı gerçek zamanlı log takip sistemi

Kurulum (Raspberry Pi host'unda, Docker dışında çalıştır):
  cd ~/DEOS_MimariF
  python3 ops/scripts/log_monitor_server.py

Bağlantı (yerel ağdaki herhangi bir cihazdan):
  http://<raspi-ip>:9000

Ortam değişkenleri:
  DEOS_LOG_ROOT       - log dizini (varsayılan: ~/DEOS_MimariF/logs)
  DEOS_MONITOR_PORT   - port (varsayılan: 9000)

Önkoşul: docker-compose.yml'de ./logs:/ros2_ws/logs volume mevcut olmalı.
"""

import json
import os
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

LOG_ROOT = Path(os.environ.get("DEOS_LOG_ROOT", str(Path.home() / "DEOS_MimariF" / "logs")))
PORT = int(os.environ.get("DEOS_MONITOR_PORT", "9000"))
MAX_HISTORY_LINES = 300
MAX_DISPLAY_LINES = 2000


# ---------------------------------------------------------------------------
# Log watcher — arka planda log dosyalarını tail eder ve SSE client'larına iletir
# ---------------------------------------------------------------------------

class LogWatcher:
    def __init__(self, root: Path) -> None:
        self.root = root
        self._clients: list[queue.Queue] = []
        self._lock = threading.Lock()
        self._positions: dict[Path, int] = {}
        threading.Thread(target=self._loop, daemon=True, name="log-watcher").start()

    def subscribe(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(maxsize=1000)
        with self._lock:
            self._clients.append(q)
        return q

    def unsubscribe(self, q: queue.Queue) -> None:
        with self._lock:
            try:
                self._clients.remove(q)
            except ValueError:
                pass

    def get_nodes(self) -> list[str]:
        if not self.root.exists():
            return []
        return sorted(p.name for p in self.root.iterdir() if p.is_dir())

    def get_recent_lines(self, node: str | None, n: int = MAX_HISTORY_LINES) -> list[dict]:
        lines: list[dict] = []
        if node:
            dirs = [self.root / node]
        else:
            dirs = [self.root / nd for nd in self.get_nodes()]
        for d in dirs:
            if not d.is_dir():
                continue
            logs = sorted(d.glob("*.log"))
            if not logs:
                continue
            nd_name = d.name
            try:
                with open(logs[-1], encoding="utf-8", errors="replace") as f:
                    for line in f:
                        line = line.rstrip()
                        if line:
                            lines.append({"node": nd_name, "line": line})
            except OSError:
                pass
        return lines[-n:]

    def _loop(self) -> None:
        while True:
            self._scan()
            time.sleep(0.15)

    def _scan(self) -> None:
        if not self.root.exists():
            return
        try:
            node_dirs = list(self.root.iterdir())
        except OSError:
            return
        for node_dir in node_dirs:
            if not node_dir.is_dir():
                continue
            logs = sorted(node_dir.glob("*.log"))
            if not logs:
                continue
            latest = logs[-1]
            node_name = node_dir.name
            try:
                with open(latest, encoding="utf-8", errors="replace") as f:
                    if latest not in self._positions:
                        f.seek(0, 2)
                        self._positions[latest] = f.tell()
                        continue
                    f.seek(self._positions[latest])
                    new_lines = f.readlines()
                    self._positions[latest] = f.tell()
                for raw in new_lines:
                    line = raw.rstrip()
                    if line:
                        self._broadcast(json.dumps(
                            {"node": node_name, "line": line},
                            ensure_ascii=False,
                        ))
            except OSError:
                self._positions.pop(latest, None)

    def _broadcast(self, msg: str) -> None:
        with self._lock:
            dead: list[queue.Queue] = []
            for q in self._clients:
                try:
                    q.put_nowait(msg)
                except queue.Full:
                    dead.append(q)
            for q in dead:
                try:
                    self._clients.remove(q)
                except ValueError:
                    pass


# ---------------------------------------------------------------------------
# HTML (tek dosyada gömülü)
# ---------------------------------------------------------------------------

_HTML = r"""<!DOCTYPE html>
<html lang="tr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>DEOS Log Monitor</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Courier New',monospace;background:#0d1117;color:#c9d1d9;height:100vh;display:flex;flex-direction:column;overflow:hidden}
#header{background:#161b22;border-bottom:1px solid #30363d;padding:8px 14px;display:flex;align-items:center;gap:10px;flex-shrink:0}
#header h1{font-size:13px;color:#58a6ff;letter-spacing:1.5px;text-transform:uppercase;white-space:nowrap}
#status-dot{width:8px;height:8px;border-radius:50%;background:#3fb950;flex-shrink:0;transition:background .3s}
#status-dot.off{background:#f85149}
#status-text{font-size:11px;color:#8b949e;white-space:nowrap}
#filter-wrap{margin-left:auto;display:flex;align-items:center;gap:6px}
#filter-input{background:#0d1117;border:1px solid #30363d;color:#c9d1d9;padding:3px 9px;border-radius:4px;font-size:11px;font-family:inherit;width:220px}
#filter-input:focus{outline:none;border-color:#58a6ff}
#filter-input.err{border-color:#f85149}
#main{display:flex;flex:1;overflow:hidden}
#sidebar{width:160px;flex-shrink:0;background:#161b22;border-right:1px solid #30363d;overflow-y:auto;padding:6px 0}
#sidebar h2{font-size:9px;color:#6e7681;text-transform:uppercase;letter-spacing:1.5px;padding:4px 10px 6px}
.node-btn{display:block;width:100%;text-align:left;background:none;border:none;color:#8b949e;padding:4px 10px;cursor:pointer;font-size:11px;font-family:inherit;border-left:2px solid transparent;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.node-btn:hover{color:#c9d1d9;background:rgba(255,255,255,.04)}
.node-btn.active{color:#58a6ff;border-left-color:#58a6ff}
#log-area{flex:1;overflow-y:scroll;padding:6px 10px;display:flex;flex-direction:column;gap:0}
.log-line{display:grid;grid-template-columns:130px 80px 56px 1fr;gap:6px;font-size:11px;line-height:1.6;padding:0 2px;border-radius:2px}
.log-line:hover{background:#161b22}
.col-node{color:#6e7681;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.col-ts{color:#484f58;white-space:nowrap;font-size:10px;padding-top:1px}
.col-lvl{font-weight:700;text-align:center}
.col-msg{white-space:pre-wrap;word-break:break-all;color:#c9d1d9}
.INFO .col-lvl{color:#3fb950}
.DEBUG .col-lvl{color:#484f58}
.WARNING .col-lvl,.WARN .col-lvl{color:#d29922}
.ERROR .col-lvl{color:#f85149}
.CRITICAL .col-lvl{color:#ff7b72}
.ERROR{background:rgba(248,81,73,.05)}
.CRITICAL{background:rgba(255,123,114,.07)}
.WARNING,.WARN{background:rgba(210,153,34,.04)}
#footer{background:#161b22;border-top:1px solid #30363d;padding:4px 14px;display:flex;align-items:center;gap:12px;font-size:10px;color:#6e7681;flex-shrink:0}
#btn-autoscroll{background:none;border:1px solid #30363d;color:#6e7681;padding:2px 7px;border-radius:3px;cursor:pointer;font-size:10px;font-family:inherit}
#btn-autoscroll.on{border-color:#3fb950;color:#3fb950}
#btn-clear{background:none;border:1px solid #30363d;color:#6e7681;padding:2px 7px;border-radius:3px;cursor:pointer;font-size:10px;font-family:inherit}
#btn-clear:hover{border-color:#f85149;color:#f85149}
#lc{margin-left:auto}
</style>
</head>
<body>
<div id="header">
  <div id="status-dot" class="off"></div>
  <h1>DEOS Log Monitor</h1>
  <span id="status-text">Bağlanıyor…</span>
  <div id="filter-wrap">
    <span style="font-size:10px;color:#484f58">Filtre:</span>
    <input id="filter-input" type="text" placeholder="regex…" spellcheck="false" autocomplete="off">
  </div>
</div>
<div id="main">
  <div id="sidebar">
    <h2>Nodes</h2>
    <button class="node-btn active" data-node="">Tümü</button>
  </div>
  <div id="log-area"></div>
</div>
<div id="footer">
  <button id="btn-autoscroll" class="on">Auto-scroll: ON</button>
  <button id="btn-clear">Temizle</button>
  <span id="lc">0 satır</span>
</div>
<script>
const logArea = document.getElementById('log-area');
const dot = document.getElementById('status-dot');
const statusText = document.getElementById('status-text');
const sidebar = document.getElementById('sidebar');
const filterInput = document.getElementById('filter-input');
const lc = document.getElementById('lc');
const btnAS = document.getElementById('btn-autoscroll');
const btnClear = document.getElementById('btn-clear');

let autoScroll = true;
let activeNode = '';
let filterRe = null;
let total = 0;

btnAS.onclick = () => {
  autoScroll = !autoScroll;
  btnAS.textContent = `Auto-scroll: ${autoScroll ? 'ON' : 'OFF'}`;
  btnAS.className = autoScroll ? 'on' : '';
  if (autoScroll) logArea.scrollTop = logArea.scrollHeight;
};

btnClear.onclick = () => { logArea.innerHTML = ''; total = 0; lc.textContent = '0 satır'; };

logArea.addEventListener('wheel', () => {
  const atBottom = logArea.scrollHeight - logArea.scrollTop - logArea.clientHeight < 40;
  if (!atBottom && autoScroll) {
    autoScroll = false;
    btnAS.textContent = 'Auto-scroll: OFF';
    btnAS.className = '';
  }
});

filterInput.addEventListener('input', () => {
  filterInput.classList.remove('err');
  try {
    filterRe = filterInput.value ? new RegExp(filterInput.value, 'i') : null;
  } catch {
    filterRe = null;
    filterInput.classList.add('err');
  }
  applyFilter();
});

function applyFilter() {
  for (const el of logArea.children) {
    const nd = el.dataset.node;
    const txt = el.dataset.txt;
    el.style.display = (!activeNode || nd === activeNode) && (!filterRe || filterRe.test(txt)) ? '' : 'none';
  }
}

sidebar.addEventListener('click', e => {
  const btn = e.target.closest('.node-btn');
  if (!btn) return;
  sidebar.querySelectorAll('.node-btn').forEach(b => b.classList.remove('active'));
  btn.classList.add('active');
  activeNode = btn.dataset.node;
  applyFilter();
});

function esc(s) { return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;'); }

function parseLine(raw) {
  // DeosLogger: "2026-07-11 14:22:01.234 | INFO | message"
  const m1 = raw.match(/^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})(?:\.\d+)? \| (\w+) \| (.*)$/s);
  if (m1) return [m1[1], m1[2], m1[3]];
  // ROS2 console: "[node-N] [INFO] [1234567.89] [logger]: message"
  const m2 = raw.match(/^\[([^\]]+)\] \[(INFO|WARN|WARNING|ERROR|DEBUG|FATAL)\] \[\d+[\.\d]*\] \[([^\]]+)\]: (.*)$/s);
  if (m2) {
    const lvl = m2[2] === 'FATAL' ? 'CRITICAL' : m2[2] === 'WARN' ? 'WARNING' : m2[2];
    return ['', lvl, `[${m2[1]}] ${m2[4]}`];
  }
  return ['', 'INFO', raw];
}

function addLine(node, raw, skipFilter) {
  const [ts, lvl, msg] = parseLine(raw);
  if (!skipFilter) {
    if (activeNode && node !== activeNode) return;
    if (filterRe && !filterRe.test(raw)) return;
  }
  const div = document.createElement('div');
  div.className = `log-line ${lvl}`;
  div.dataset.node = node;
  div.dataset.txt = raw;
  const show = (!activeNode || node === activeNode) && (!filterRe || filterRe.test(raw));
  if (!show) div.style.display = 'none';
  div.innerHTML =
    `<span class="col-node" title="${esc(node)}">${esc(node)}</span>` +
    `<span class="col-ts">${esc(ts.slice(11))}</span>` +
    `<span class="col-lvl">${lvl}</span>` +
    `<span class="col-msg">${esc(msg)}</span>`;
  logArea.appendChild(div);
  total++;
  lc.textContent = `${total} satır`;
  while (logArea.children.length > """ + str(MAX_DISPLAY_LINES) + r""") logArea.removeChild(logArea.firstChild);
  if (autoScroll) logArea.scrollTop = logArea.scrollHeight;
}

function loadNodes() {
  fetch('/nodes').then(r => r.json()).then(nodes => {
    const existing = new Set([...sidebar.querySelectorAll('.node-btn')].map(b => b.dataset.node));
    nodes.forEach(n => {
      if (existing.has(n)) return;
      const btn = document.createElement('button');
      btn.className = 'node-btn';
      btn.dataset.node = n;
      btn.textContent = n.replace(/_node$/, '');
      btn.title = n;
      sidebar.appendChild(btn);
    });
  }).catch(() => {});
}

function loadHistory() {
  const url = activeNode ? `/logs?node=${encodeURIComponent(activeNode)}` : '/logs';
  fetch(url).then(r => r.json()).then(lines => {
    logArea.innerHTML = '';
    total = 0;
    lines.forEach(({node, line}) => addLine(node, line, true));
    applyFilter();
    if (autoScroll) logArea.scrollTop = logArea.scrollHeight;
  }).catch(() => {});
}

function connect() {
  const es = new EventSource('/events');
  es.onopen = () => {
    dot.className = '';
    statusText.textContent = 'Bağlandı — canlı izleniyor';
    loadNodes();
    loadHistory();
  };
  es.onmessage = e => {
    const {node, line} = JSON.parse(e.data);
    addLine(node, line, false);
  };
  es.onerror = () => {
    dot.className = 'off';
    statusText.textContent = 'Bağlantı kesildi — 3s sonra tekrar denenecek…';
    es.close();
    setTimeout(connect, 3000);
  };
}

connect();
setInterval(loadNodes, 8000);
</script>
</body>
</html>
"""


# ---------------------------------------------------------------------------
# HTTP request handler
# ---------------------------------------------------------------------------

class Handler(BaseHTTPRequestHandler):
    watcher: "LogWatcher"

    def log_message(self, fmt, *args) -> None:
        pass  # erişim loglarını bastır

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        qs = parse_qs(parsed.query)

        if parsed.path in ("/", "/index.html"):
            body = _HTML.encode("utf-8")
            self._respond(200, "text/html; charset=utf-8", body)

        elif parsed.path == "/nodes":
            body = json.dumps(self.watcher.get_nodes(), ensure_ascii=False).encode()
            self._respond(200, "application/json", body)

        elif parsed.path == "/logs":
            node = qs.get("node", [None])[0]
            lines = self.watcher.get_recent_lines(node)
            body = json.dumps(lines, ensure_ascii=False).encode()
            self._respond(200, "application/json", body)

        elif parsed.path == "/events":
            self._sse()

        else:
            self._respond(404, "text/plain", b"Not Found")

    def _respond(self, code: int, ct: str, body: bytes) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ct)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _sse(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        q = self.watcher.subscribe()
        try:
            self.wfile.write(b": connected\n\n")
            self.wfile.flush()
            while True:
                try:
                    msg = q.get(timeout=20)
                    self.wfile.write(f"data: {msg}\n\n".encode())
                    self.wfile.flush()
                except queue.Empty:
                    self.wfile.write(b": ka\n\n")
                    self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            self.watcher.unsubscribe(q)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    watcher = LogWatcher(LOG_ROOT)

    class BoundHandler(Handler):
        pass

    BoundHandler.watcher = watcher

    server = HTTPServer(("0.0.0.0", PORT), BoundHandler)
    raspi_ip = os.popen("hostname -I 2>/dev/null | awk '{print $1}'").read().strip() or "<raspi-ip>"
    print(f"DEOS Log Monitor başlatıldı")
    print(f"  Yerel erişim : http://localhost:{PORT}")
    print(f"  Ağdan erişim : http://{raspi_ip}:{PORT}")
    print(f"  Log dizini   : {LOG_ROOT}")
    print("Durdurmak için Ctrl+C")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nDurduruldu.")


if __name__ == "__main__":
    main()
