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

Not: Tarayıcının Tailwind CDN / Google Fonts için internet erişimi olması gerekir.
     Raspi'nin internete ihtiyacı yoktur.
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


# ---------------------------------------------------------------------------
# Log watcher
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
        dirs = [self.root / node] if node else [self.root / nd for nd in self.get_nodes()]
        for d in dirs:
            if not d.is_dir():
                continue
            logs = sorted(d.glob("*.log"))
            if not logs:
                continue
            try:
                with open(logs[-1], encoding="utf-8", errors="replace") as f:
                    for line in f:
                        line = line.rstrip()
                        if line:
                            lines.append({"node": d.name, "line": line})
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
                        self._broadcast(json.dumps({"node": node_name, "line": line}, ensure_ascii=False))
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
# HTML — Stitch tabanli tasarim + SSE entegrasyonu
# ---------------------------------------------------------------------------

_HTML = r"""<!DOCTYPE html>
<html class="dark" lang="en">
<head>
<meta charset="utf-8"/>
<meta content="width=device-width, initial-scale=1.0" name="viewport"/>
<title>DEOS Log Monitor</title>
<script src="https://cdn.tailwindcss.com?plugins=forms,container-queries"></script>
<link href="https://fonts.googleapis.com/css2?family=Material+Symbols+Outlined:wght,FILL@100..700,0..1&display=swap" rel="stylesheet"/>
<link href="https://fonts.googleapis.com/css2?family=Geist:wght@400;500;600&family=Poppins:wght@400;600;700&display=swap" rel="stylesheet"/>
<script id="tailwind-config">
tailwind.config = {
  darkMode: "class",
  theme: {
    extend: {
      colors: {
        "background":               "#131313",
        "surface":                  "#131313",
        "surface-container-lowest": "#0e0e0e",
        "surface-container-low":    "#1b1b1b",
        "surface-container":        "#1f1f1f",
        "surface-container-high":   "#2a2a2a",
        "surface-container-highest":"#353535",
        "surface-bright":           "#393939",
        "on-surface":               "#e2e2e2",
        "on-surface-variant":       "#c4c7c8",
        "border-subtle":            "#262626",
        "outline":                  "#8e9192",
        "outline-variant":          "#444748",
        "text-muted":               "#A3A3A3",
        "primary":                  "#ffffff",
        "error-container":          "#93000a",
        "status-success":           "#34C759",
        "status-warning":           "#FFCC00",
        "status-error":             "#FF3B30"
      },
      borderRadius: {
        "DEFAULT": "0.125rem",
        "lg":      "0.25rem",
        "xl":      "0.5rem",
        "full":    "0.75rem"
      },
      fontFamily: {
        "label-sm":   ["Geist","monospace"],
        "mono-code":  ["Geist","monospace"],
        "headline-md":["Poppins","sans-serif"],
        "label-md":   ["Geist","monospace"],
        "body-md":    ["Poppins","sans-serif"]
      },
      fontSize: {
        "label-sm":  ["12px",{"lineHeight":"16px","letterSpacing":"0.05em","fontWeight":"500"}],
        "mono-code": ["13px",{"lineHeight":"20px","fontWeight":"400"}],
        "headline-md":["24px",{"lineHeight":"32px","fontWeight":"600"}],
        "label-md":  ["14px",{"lineHeight":"20px","letterSpacing":"0.02em","fontWeight":"500"}]
      }
    }
  }
}
</script>
<style>
body { background-color: #131313; }
.glow-critical { box-shadow: 0 0 8px rgba(255,59,48,.2); border: 1px solid rgba(255,59,48,.5) !important; }
.border-error-faint { border: 1px solid rgba(255,59,48,.3) !important; }
.pulse-dot { animation: pulse-ring 2s infinite; }
@keyframes pulse-ring {
  0%  { box-shadow: 0 0 0 0   rgba(52,199,89,.7); }
  70% { box-shadow: 0 0 0 6px rgba(52,199,89,0);  }
  100%{ box-shadow: 0 0 0 0   rgba(52,199,89,0);  }
}
::-webkit-scrollbar       { width:6px; height:6px; }
::-webkit-scrollbar-track { background:#131313; }
::-webkit-scrollbar-thumb { background:#2a2a2a; border-radius:4px; }
::-webkit-scrollbar-thumb:hover { background:#393939; }
</style>
</head>
<body class="text-on-surface h-screen flex flex-col overflow-hidden bg-background">

<!-- ── HEADER ── -->
<header class="bg-surface fixed top-0 w-full z-50 border-b border-border-subtle flex justify-between items-center h-16 px-4">
  <div class="flex items-center gap-4">
    <div class="flex items-center gap-3">
      <span id="status-dot" class="w-2 h-2 bg-status-error rounded-full"></span>
      <span class="font-label-md text-label-md text-primary tracking-widest uppercase font-semibold">DEOS</span>
      <span class="font-headline-md text-[15px] text-on-surface font-semibold">Log Monitor</span>
    </div>
    <div class="hidden md:flex flex-col ml-4 border-l border-border-subtle pl-4">
      <span id="raspi-ip" class="font-label-sm text-label-sm uppercase text-text-muted">---.---.---.---</span>
      <span id="uptime-display" class="font-mono-code text-mono-code text-text-muted opacity-60">Uptime: 00:00:00</span>
    </div>
  </div>
  <div class="relative w-64 hidden md:block">
    <span class="material-symbols-outlined absolute left-3 top-1/2 -translate-y-1/2 text-text-muted text-sm select-none" style="font-variation-settings:'FILL' 0;font-size:16px">search</span>
    <input id="filter-input" class="w-full bg-surface-container-low border border-border-subtle rounded font-mono-code text-mono-code text-on-surface pl-9 py-1.5 focus:border-outline-variant outline-none transition-colors placeholder:text-text-muted" placeholder="Search / Regex..." type="text" autocomplete="off" spellcheck="false"/>
  </div>
  <div class="flex items-center gap-2 text-text-muted">
    <button class="p-2 rounded hover:bg-surface-container-low transition-colors hover:text-primary">
      <span class="material-symbols-outlined text-[20px]">sensors</span>
    </button>
    <button class="p-2 rounded hover:bg-surface-container-low transition-colors hover:text-primary">
      <span class="material-symbols-outlined text-[20px]">settings</span>
    </button>
  </div>
</header>

<!-- ── MAIN ── -->
<main class="flex-1 flex pt-16 pb-8 overflow-hidden">

  <!-- Sidebar -->
  <aside class="w-[180px] bg-surface border-r border-border-subtle flex flex-col h-full shrink-0">
    <div class="p-4 border-b border-border-subtle">
      <h2 class="font-label-sm text-label-sm uppercase tracking-widest text-text-muted">ACTIVE NODES</h2>
    </div>
    <nav id="node-nav" class="flex-1 overflow-y-auto py-2 flex flex-col gap-1">
      <button class="text-primary bg-surface-container-high rounded mx-2 px-3 py-2 text-left font-mono-code text-mono-code flex items-center justify-between border-l-2 border-primary w-full" data-node="">
        <span>ALL NODES</span>
        <span class="w-1.5 h-1.5 rounded-full bg-primary flex-shrink-0"></span>
      </button>
    </nav>
  </aside>

  <!-- Log stream -->
  <section class="flex-1 flex flex-col h-full bg-background min-w-0 relative">
    <div class="grid grid-cols-[140px_90px_90px_1fr] gap-4 px-6 py-3 border-b border-border-subtle bg-surface font-label-sm text-label-sm uppercase tracking-widest text-text-muted shrink-0">
      <div>NODE</div><div>TIME</div><div>LEVEL</div><div>MESSAGE</div>
    </div>
    <div id="log-container" class="flex-1 overflow-y-auto font-mono-code text-mono-code px-4 py-2 space-y-0.5"></div>
    <!-- Empty state -->
    <div id="empty-state" class="absolute inset-0 flex flex-col items-center justify-center gap-3 pointer-events-none" style="top:48px">
      <span class="material-symbols-outlined text-border-subtle select-none" style="font-size:48px">terminal</span>
      <span class="text-text-muted font-label-sm text-label-sm uppercase tracking-widest">Bağlanıyor…</span>
    </div>
    <!-- Jump to bottom -->
    <button id="jump-btn" style="display:none" class="absolute bottom-12 right-4 bg-surface-container-high border border-border-subtle text-text-muted hover:text-primary px-3 py-1.5 rounded font-label-sm text-label-sm flex items-center gap-1.5 transition-colors z-10">
      <span class="material-symbols-outlined text-[14px]">arrow_downward</span>Jump to bottom
    </button>
  </section>

  <!-- Camera panel -->
  <aside class="w-[380px] bg-surface border-l border-border-subtle flex flex-col shrink-0">
    <div class="p-4 border-b border-border-subtle flex justify-between items-center">
      <div class="flex items-center gap-2">
        <span id="camera-dot" class="w-1.5 h-1.5 rounded-full bg-status-error animate-pulse"></span>
        <h2 class="font-label-sm text-label-sm uppercase tracking-widest text-on-surface">CAMERA FEED</h2>
      </div>
      <button id="camera-fs-btn" class="text-text-muted hover:text-primary transition-colors" title="Open in new tab">
        <span class="material-symbols-outlined text-[16px]">open_in_new</span>
      </button>
    </div>
    <div class="p-4 flex-1 flex flex-col gap-4">
      <div class="w-full aspect-video bg-black rounded border border-border-subtle relative overflow-hidden">
        <img id="camera-feed" style="display:none" class="w-full h-full object-cover absolute inset-0" alt="Camera Feed"/>
        <div id="camera-placeholder" class="absolute inset-0 flex flex-col items-center justify-center gap-2">
          <span class="material-symbols-outlined text-border-subtle select-none" style="font-size:40px">videocam_off</span>
          <span class="font-mono-code text-[11px] text-text-muted">No Signal</span>
          <span class="font-mono-code text-[10px] text-text-muted opacity-50">web_video_server :8080</span>
        </div>
        <div class="absolute top-2 left-2 bg-black/70 px-1.5 py-0.5 rounded font-mono-code text-[10px] text-on-surface border border-border-subtle pointer-events-none">CAM_FRONT_CENTER</div>
        <div class="absolute inset-0 flex items-center justify-center pointer-events-none opacity-10">
          <div class="w-16 h-px bg-primary"></div>
          <div class="w-px h-16 bg-primary absolute"></div>
        </div>
      </div>
      <div class="grid grid-cols-2 gap-2 mt-auto">
        <div class="bg-surface-container-low p-3 rounded border border-border-subtle">
          <div class="font-label-sm text-label-sm uppercase tracking-widest text-text-muted mb-1">RESOLUTION</div>
          <div id="cam-res" class="font-mono-code text-mono-code text-on-surface">1280x720</div>
        </div>
        <div class="bg-surface-container-low p-3 rounded border border-border-subtle">
          <div class="font-label-sm text-label-sm uppercase tracking-widest text-text-muted mb-1">FRAMERATE</div>
          <div id="cam-fps" class="font-mono-code text-mono-code text-text-muted">-- FPS</div>
        </div>
      </div>
    </div>
  </aside>

</main>

<!-- ── FOOTER ── -->
<footer class="bg-surface font-mono-code text-[11px] fixed bottom-0 left-[180px] right-0 h-8 border-t border-border-subtle flex justify-between items-center px-4 z-50">
  <div class="flex items-center gap-6">
    <span class="text-text-muted">DEOS Log Monitor</span>
    <span id="line-count" class="text-text-muted">0 lines | 0 lps</span>
    <span id="node-count" class="text-text-muted">Nodes: 0</span>
  </div>
  <div class="flex items-center gap-4">
    <label class="flex items-center gap-2 cursor-pointer text-text-muted hover:text-primary transition-colors">
      <input id="autoscroll-cb" checked class="form-checkbox h-3 w-3 text-primary rounded-sm border-border-subtle bg-surface-container-low focus:ring-0 focus:ring-offset-0" type="checkbox"/>
      <span class="uppercase tracking-widest">Auto-scroll</span>
    </label>
    <button id="clear-btn" class="text-text-muted hover:text-primary transition-colors flex items-center gap-1 uppercase tracking-widest">
      <span class="material-symbols-outlined text-[14px]">delete</span>Clear Logs
    </button>
    <div class="flex items-center gap-2 ml-4 pl-4 border-l border-border-subtle">
      <span id="conn-dot" class="w-1.5 h-1.5 rounded-full bg-status-error"></span>
      <span id="conn-label" class="text-text-muted">Disconnected</span>
    </div>
  </div>
</footer>

<script>
const CAMERA_TOPIC = '/deos/sensors/camera/color';
const CAMERA_URL   = `http://${window.location.hostname}:8080/stream?topic=${CAMERA_TOPIC}`;

let autoScroll = true, activeNode = '', filterRe = null;
let total = 0, lpsCount = 0;
let startTime = Date.now();
let knownNodes = new Set(['']);

const $ = id => document.getElementById(id);
const logContainer  = $('log-container');
const filterInput   = $('filter-input');
const statusDot     = $('status-dot');
const connDot       = $('conn-dot');
const connLabel     = $('conn-label');
const nodeNav       = $('node-nav');
const lineCountEl   = $('line-count');
const nodeCountEl   = $('node-count');
const uptimeEl      = $('uptime-display');
const cameraDot     = $('camera-dot');
const cameraFeed    = $('camera-feed');
const cameraPlaceholder = $('camera-placeholder');
const autoscrollCb  = $('autoscroll-cb');
const clearBtn      = $('clear-btn');
const jumpBtn       = $('jump-btn');
const emptyState    = $('empty-state');
const raspiIpEl     = $('raspi-ip');
const cameraFsBtn   = $('camera-fs-btn');

raspiIpEl.textContent = window.location.hostname;

// ── Uptime ──
const pad2 = n => String(n).padStart(2,'0');
setInterval(() => {
  const e = Math.floor((Date.now()-startTime)/1000);
  uptimeEl.textContent = `Uptime: ${pad2(Math.floor(e/3600))}:${pad2(Math.floor(e%3600/60))}:${pad2(e%60)}`;
}, 1000);

// ── LPS ──
setInterval(() => {
  lineCountEl.textContent = `${total} lines | ${lpsCount} lps`;
  lpsCount = 0;
}, 1000);

// ── Camera ──
cameraFeed.src = CAMERA_URL;
cameraFeed.onload = () => {
  cameraPlaceholder.style.display = 'none';
  cameraFeed.style.display = 'block';
  cameraDot.className = 'w-1.5 h-1.5 rounded-full bg-status-success animate-pulse';
  $('cam-fps').textContent = '30 FPS';
  $('cam-fps').className = 'font-mono-code text-mono-code text-status-success';
};
cameraFeed.onerror = () => {
  cameraFeed.style.display = 'none';
  cameraPlaceholder.style.display = 'flex';
  cameraDot.className = 'w-1.5 h-1.5 rounded-full bg-status-error animate-pulse';
};
cameraFsBtn.addEventListener('click', () => window.open(CAMERA_URL, '_blank'));

// ── Filter ──
filterInput.addEventListener('input', () => {
  filterInput.classList.remove('border-status-error');
  try { filterRe = filterInput.value ? new RegExp(filterInput.value,'i') : null; }
  catch { filterRe = null; filterInput.classList.add('border-status-error'); }
  applyFilter();
});
function applyFilter() {
  for (const el of logContainer.children) {
    el.style.display = (!activeNode||el.dataset.node===activeNode)&&(!filterRe||filterRe.test(el.dataset.txt)) ? '' : 'none';
  }
}

// ── Node nav ──
const ACT_CLS  = 'text-primary bg-surface-container-high rounded mx-2 px-3 py-2 text-left font-mono-code text-mono-code flex items-center justify-between border-l-2 border-primary w-full';
const INACT_CLS = 'text-text-muted hover:text-on-surface hover:bg-surface-container-low transition-all mx-2 px-3 py-2 text-left font-mono-code text-mono-code flex items-center justify-between rounded border-l-2 border-transparent hover:border-border-subtle w-full';

nodeNav.addEventListener('click', e => {
  const btn = e.target.closest('[data-node]');
  if (!btn) return;
  nodeNav.querySelectorAll('[data-node]').forEach(b => b.className = INACT_CLS);
  btn.className = ACT_CLS;
  activeNode = btn.dataset.node;
  applyFilter();
});

// ── Auto-scroll ──
autoscrollCb.addEventListener('change', () => {
  autoScroll = autoscrollCb.checked;
  if (autoScroll) { logContainer.scrollTop = logContainer.scrollHeight; jumpBtn.style.display = 'none'; }
});
logContainer.addEventListener('scroll', () => {
  const atBottom = logContainer.scrollHeight - logContainer.scrollTop - logContainer.clientHeight < 60;
  if (!atBottom && autoScroll) { autoScroll = false; autoscrollCb.checked = false; jumpBtn.style.display = 'flex'; }
  if (atBottom) jumpBtn.style.display = 'none';
});
jumpBtn.addEventListener('click', () => {
  logContainer.scrollTop = logContainer.scrollHeight;
  autoScroll = true; autoscrollCb.checked = true; jumpBtn.style.display = 'none';
});

// ── Clear ──
clearBtn.addEventListener('click', () => {
  logContainer.innerHTML = '';
  total = 0;
  emptyState.style.display = 'flex';
});

// ── Helpers ──
function esc(s) { return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;'); }

function parseLine(raw) {
  const m1 = raw.match(/^(\d{4}-\d{2}-\d{2} (\d{2}:\d{2}:\d{2}))(?:\.\d+)? \| (\w+) \| (.*)$/s);
  if (m1) return [m1[2], m1[3], m1[4]];
  const m2 = raw.match(/^\[([^\]]+)\] \[(INFO|WARN|WARNING|ERROR|DEBUG|FATAL)\] \[\d+[\.\d]*\] \[([^\]]+)\]: (.*)$/s);
  if (m2) {
    const l = m2[2]==='FATAL'?'CRITICAL':m2[2]==='WARN'?'WARNING':m2[2];
    return ['', l, `[${m2[1]}] ${m2[4]}`];
  }
  return ['','INFO',raw];
}

function badge(lvl) {
  const m = {
    INFO:    'bg-surface-bright text-on-surface border-border-subtle',
    DEBUG:   'bg-surface-container-low text-text-muted border-border-subtle',
    WARNING: 'bg-surface-container text-status-warning border-status-warning/20',
    ERROR:   'bg-error-container/20 text-status-error border-status-error/30',
    CRITICAL:'bg-status-error text-surface font-semibold animate-pulse',
  };
  return `<span class="px-1.5 py-0.5 rounded ${m[lvl]||m.INFO} border text-[10px] tracking-wider uppercase">${lvl}</span>`;
}

function addLine(node, raw, skipFilter) {
  const [ts, lvl, msg] = parseLine(raw);
  if (!skipFilter) {
    if (activeNode && node !== activeNode) return;
    if (filterRe && !filterRe.test(raw)) return;
  }
  const visible = (!activeNode||node===activeNode)&&(!filterRe||filterRe.test(raw));
  const isCrit = lvl==='CRITICAL', isErr = lvl==='ERROR', isWarn = lvl==='WARNING', isDbg = lvl==='DEBUG';

  const rowCls = 'grid grid-cols-[140px_90px_90px_1fr] gap-4 px-2 py-1.5 rounded hover:bg-surface-container-low transition-colors items-center border '
    + (isCrit ? 'bg-error-container/20 glow-critical'
       : isErr  ? 'bg-error-container/10 border-error-faint'
       : isDbg  ? 'bg-surface-container-lowest border-transparent'
       : 'border-transparent');
  const nodeCls = 'truncate ' + (isCrit||isErr ? 'text-status-error font-medium' : 'text-on-surface');
  const tsCls   = isCrit ? 'text-status-error/80' : isErr ? 'text-status-error/70' : 'text-text-muted';
  const msgCls  = 'truncate ' + (isCrit||isErr ? 'text-status-error font-medium' : isWarn ? 'text-status-warning' : isDbg ? 'text-text-muted' : 'text-on-surface');

  const d = document.createElement('div');
  d.className = rowCls;
  d.dataset.node = node;
  d.dataset.txt  = raw;
  if (!visible) d.style.display = 'none';
  d.innerHTML = `<div class="${nodeCls}" title="${esc(node)}">${esc(node.replace('_node',''))}</div>`
              + `<div class="${tsCls}">${esc(ts)}</div>`
              + `<div>${badge(lvl)}</div>`
              + `<div class="${msgCls}">${esc(msg)}</div>`;

  logContainer.appendChild(d);
  if (logContainer.children.length === 1) emptyState.style.display = 'none';
  total++; lpsCount++;
  while (logContainer.children.length > 2000) logContainer.removeChild(logContainer.firstChild);
  if (autoScroll) logContainer.scrollTop = logContainer.scrollHeight;
}

// ── Node loader ──
function loadNodes() {
  fetch('/nodes').then(r=>r.json()).then(nodes => {
    nodeCountEl.textContent = `Nodes: ${nodes.length}`;
    nodes.forEach(n => {
      if (knownNodes.has(n)) return;
      knownNodes.add(n);
      const b = document.createElement('button');
      b.dataset.node = n;
      b.className    = INACT_CLS;
      b.innerHTML    = `<span>${esc(n.replace('_node',''))}</span><span class="w-1.5 h-1.5 rounded-full bg-status-success flex-shrink-0"></span>`;
      nodeNav.appendChild(b);
    });
  }).catch(()=>{});
}

// ── History ──
function loadHistory() {
  const url = activeNode ? `/logs?node=${encodeURIComponent(activeNode)}` : '/logs';
  fetch(url).then(r=>r.json()).then(lines => {
    logContainer.innerHTML = ''; total = 0;
    lines.forEach(({node,line}) => addLine(node, line, true));
    applyFilter();
    if (autoScroll) logContainer.scrollTop = logContainer.scrollHeight;
  }).catch(()=>{});
}

// ── SSE ──
function connect() {
  const es = new EventSource('/events');
  es.onopen = () => {
    statusDot.className = 'w-2 h-2 bg-status-success rounded-full pulse-dot';
    connDot.className   = 'w-1.5 h-1.5 rounded-full bg-status-success';
    connLabel.textContent = 'Live';
    loadNodes();
    loadHistory();
  };
  es.onmessage = e => {
    const {node, line} = JSON.parse(e.data);
    addLine(node, line, false);
  };
  es.onerror = () => {
    statusDot.className = 'w-2 h-2 bg-status-error rounded-full';
    connDot.className   = 'w-1.5 h-1.5 rounded-full bg-status-error';
    connLabel.textContent = 'Reconnecting…';
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
        pass

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
    print(f"DEOS Log Monitor baslatildi")
    print(f"  Yerel erisim : http://localhost:{PORT}")
    print(f"  Agdan erisim : http://{raspi_ip}:{PORT}")
    print(f"  Log dizini   : {LOG_ROOT}")
    print("Durdurmak icin Ctrl+C")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nDurduruldu.")


if __name__ == "__main__":
    main()
