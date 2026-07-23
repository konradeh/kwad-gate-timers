from flask import Flask, request, jsonify, render_template_string
from collections import deque
from datetime import datetime
import hashlib
import math
import threading
import time

app = Flask(__name__)

# In-memory event log - list of dicts: {node_id, timestamp, received_at, drone_id, rssi}
events = []
events_lock = threading.Lock()

# Last-contact registry per node_id, updated by both /checkpoint and
# /api/heartbeat. This is what powers the debug panel's online/offline dots.
# {"checkpoint-1": {"last_seen": <epoch>, "last_type": "heartbeat", "ip": "...", "wifi_rssi": -50}}
node_registry = {}
node_registry_lock = threading.Lock()

# Rolling log of every request that touched /checkpoint or /api/heartbeat,
# newest last. Shown raw in the debug panel.
raw_log = deque(maxlen=50)
raw_log_lock = threading.Lock()

# A node is considered "online" if we've heard from it (heartbeat or
# checkpoint event) within this many seconds.
NODE_ONLINE_TIMEOUT_S = 8.0

def record_contact(node_id, kind, **extra):
    now = time.time()
    with node_registry_lock:
        node_registry[node_id] = {
            "last_seen": now,
            "last_type": kind,
            "ip": request.remote_addr,
            **extra,
        }
    with raw_log_lock:
        raw_log.append({
            "time_str": datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "node_id": node_id,
            "kind": kind,
            "ip": request.remote_addr,
            "detail": extra,
        })

# Latest known state per drone, keyed by drone_id (as string).
# {"1": {"node_id": "checkpoint-1", "rssi": -58, "last_seen": <epoch>, "sequence": 123}}
drone_state = {}
drone_state_lock = threading.Lock()

# Where each checkpoint node sits on the radar square, as a percentage
# (0-100) from the top-left. Edit this to match your physical track layout.
NODE_POSITIONS = {
    "checkpoint-1": (18, 50),
    "checkpoint-2": (82, 50),
}

# Any node_id not listed above gets placed automatically on a circle so it
# still shows up on the radar instead of being dropped.
def get_node_position(node_id):
    if node_id in NODE_POSITIONS:
        return NODE_POSITIONS[node_id]
    h = int(hashlib.md5(node_id.encode()).hexdigest(), 16)
    angle = math.radians(h % 360)
    return (50 + 35 * math.cos(angle), 50 + 35 * math.sin(angle))

# Colors cycled through for distinguishing multiple drones on the radar.
DRONE_COLORS = ["#e0902f", "#3fb3c9", "#c94fd6", "#7fc93f", "#e04f4f", "#4f6fe0"]

def get_drone_color(drone_id):
    try:
        idx = int(drone_id)
    except (TypeError, ValueError):
        idx = abs(hash(drone_id))
    return DRONE_COLORS[idx % len(DRONE_COLORS)]

# How long (seconds) a drone stays visible on the radar after its last
# reported pass before fading out entirely.
DRONE_TIMEOUT_S = 6.0

NAV = """
<div class="nav">
    <a href="/" class="{active_leaderboard}">Leaderboard</a>
    <a href="/radar" class="{active_radar}">Radar</a>
</div>
"""

BASE_STYLE = """
        :root {
            --bg: #14171a;
            --panel: #1a1e22;
            --hairline: #282e33;
            --paper: #ece7de;
            --muted: #7c8790;
            --accent: #e0902f;
            --accent-soft: rgba(224, 144, 47, 0.14);
        }

        * { box-sizing: border-box; margin: 0; padding: 0; }

        body {
            font-family: 'JetBrains Mono', monospace;
            background: var(--bg);
            color: var(--paper);
            padding: 48px 20px;
            min-height: 100vh;
        }

        .wrap { max-width: 880px; margin: 0 auto; }

        .panel {
            position: relative;
            border: 1px solid var(--hairline);
            background: var(--panel);
            padding: 28px 30px;
            margin-bottom: 22px;
        }

        .panel::before, .panel::after {
            content: '';
            position: absolute;
            width: 12px; height: 12px;
            border: 1.5px solid var(--muted);
            opacity: 0.6;
        }
        .panel::before { top: -1px; left: -1px; border-right: none; border-bottom: none; }
        .panel::after  { bottom: -1px; right: -1px; border-left: none; border-top: none; }

        .eyebrow {
            font-family: 'JetBrains Mono', monospace;
            font-size: 11px;
            font-weight: 500;
            letter-spacing: 3px;
            text-transform: uppercase;
            color: var(--muted);
        }

        .title-row {
            display: flex;
            align-items: baseline;
            gap: 12px;
            margin-top: 8px;
            flex-wrap: wrap;
        }

        h1 {
            font-family: 'Space Grotesk', sans-serif;
            font-size: 30px;
            font-weight: 700;
            letter-spacing: 0.3px;
        }

        .live {
            display: inline-flex;
            align-items: center;
            gap: 7px;
            font-size: 11px;
            letter-spacing: 2px;
            color: var(--accent);
            text-transform: uppercase;
            font-weight: 500;
        }

        .live-dot {
            width: 7px; height: 7px;
            border-radius: 50%;
            background: var(--accent);
            box-shadow: 0 0 6px rgba(224, 144, 47, 0.7);
            animation: pulse 1.6s ease-in-out infinite;
        }

        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.3; }
        }

        @media (prefers-reduced-motion: reduce) {
            .live-dot { animation: none; }
        }

        .subhead {
            margin-top: 10px;
            font-size: 13px;
            color: var(--muted);
            max-width: 46ch;
        }

        .nav {
            display: flex;
            gap: 4px;
            max-width: 880px;
            margin: 0 auto 18px;
        }

        .nav a {
            font-family: 'JetBrains Mono', monospace;
            font-size: 11px;
            letter-spacing: 2px;
            text-transform: uppercase;
            color: var(--muted);
            text-decoration: none;
            padding: 8px 16px;
            border: 1px solid var(--hairline);
            background: var(--panel);
        }

        .nav a.active {
            color: var(--accent);
            border-color: var(--accent);
        }

        .stats {
            display: flex;
            gap: 14px;
            margin-top: 24px;
            flex-wrap: wrap;
        }

        .stat {
            border: 1px solid var(--hairline);
            padding: 14px 18px;
            min-width: 140px;
            flex: 1;
        }

        .stat-label {
            font-size: 10px;
            letter-spacing: 2px;
            color: var(--muted);
            text-transform: uppercase;
        }

        .stat-value {
            font-family: 'Space Grotesk', sans-serif;
            font-size: 22px;
            font-weight: 700;
            color: var(--paper);
            margin-top: 6px;
        }

        /* split-flap style digit counter for total passes */
        .flap-row {
            display: flex;
            gap: 4px;
            margin-top: 8px;
        }

        .flap-digit {
            position: relative;
            width: 26px;
            height: 34px;
            background: var(--bg);
            border: 1px solid var(--hairline);
            border-radius: 2px;
            display: flex;
            align-items: center;
            justify-content: center;
            font-family: 'JetBrains Mono', monospace;
            font-size: 20px;
            font-weight: 700;
            color: var(--accent);
            overflow: hidden;
        }

        .flap-digit::after {
            content: '';
            position: absolute;
            left: 0; right: 0; top: 50%;
            height: 1px;
            background: var(--hairline);
        }

        .node-pill {
            display: inline-block;
            padding: 3px 10px;
            border: 1px solid var(--hairline);
            font-size: 12px;
            letter-spacing: 0.5px;
            color: var(--paper);
        }

        .table-scroll { overflow-x: auto; }

        table { width: 100%; border-collapse: collapse; min-width: 480px; }

        thead th {
            text-align: left;
            font-size: 11px;
            letter-spacing: 2px;
            text-transform: uppercase;
            color: var(--muted);
            font-weight: 500;
            padding: 10px 12px;
            border-bottom: 1px solid var(--hairline);
            white-space: nowrap;
        }

        tbody td {
            padding: 12px;
            font-size: 13px;
            border-bottom: 1px solid var(--hairline);
            white-space: nowrap;
        }

        tbody tr:hover { background: rgba(255,255,255,0.02); }

        tbody tr:first-child td { border-left: 2px solid var(--accent); }
        tbody tr:first-child td:first-child { padding-left: 10px; }
        tbody tr td:first-child { border-left: 2px solid transparent; }

        .idx { color: var(--muted); width: 32px; }

        .empty {
            padding: 44px 14px;
            text-align: center;
            color: var(--muted);
            letter-spacing: 1px;
            font-size: 12px;
            border: 1px dashed var(--hairline);
        }

        .debug-tab {
            position: fixed;
            top: 50%;
            right: 0;
            transform: translateY(-50%);
            writing-mode: vertical-rl;
            background: var(--panel);
            border: 1px solid var(--hairline);
            border-right: none;
            color: var(--muted);
            font-family: 'JetBrains Mono', monospace;
            font-size: 11px;
            letter-spacing: 2px;
            text-transform: uppercase;
            padding: 14px 8px;
            cursor: pointer;
            z-index: 30;
        }

        .debug-tab:hover { color: var(--accent); border-color: var(--accent); }

        .debug-panel {
            position: fixed;
            top: 0;
            right: 0;
            bottom: 0;
            width: 340px;
            max-width: 90vw;
            background: var(--panel);
            border-left: 1px solid var(--hairline);
            transform: translateX(100%);
            transition: transform 0.25s ease;
            z-index: 40;
            display: flex;
            flex-direction: column;
        }

        .debug-panel.open { transform: translateX(0); }

        .debug-header {
            padding: 18px 18px 14px;
            border-bottom: 1px solid var(--hairline);
            display: flex;
            align-items: center;
            justify-content: space-between;
        }

        .debug-header .eyebrow { margin: 0; }

        .debug-close {
            background: none;
            border: 1px solid var(--hairline);
            color: var(--muted);
            font-family: 'JetBrains Mono', monospace;
            font-size: 11px;
            padding: 4px 9px;
            cursor: pointer;
        }

        .debug-close:hover { color: var(--accent); border-color: var(--accent); }

        .debug-section {
            padding: 14px 18px;
            border-bottom: 1px solid var(--hairline);
            overflow-y: auto;
        }

        .debug-section-label {
            font-size: 10px;
            letter-spacing: 2px;
            text-transform: uppercase;
            color: var(--muted);
            margin-bottom: 10px;
        }

        .debug-node-row {
            display: flex;
            align-items: center;
            gap: 8px;
            padding: 6px 0;
            font-size: 12px;
        }

        .debug-dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            flex-shrink: 0;
        }

        .debug-dot.online { background: #7fc93f; box-shadow: 0 0 5px #7fc93f; }
        .debug-dot.offline { background: #e04f4f; box-shadow: 0 0 5px #e04f4f; }

        .debug-node-id { flex: 1; }

        .debug-node-meta { color: var(--muted); font-size: 11px; }

        .debug-log {
            flex: 1;
            overflow-y: auto;
            padding: 10px 18px;
            font-size: 11px;
        }

        .debug-log-row {
            padding: 7px 0;
            border-bottom: 1px solid var(--hairline);
            line-height: 1.5;
        }

        .debug-log-time { color: var(--muted); }

        .debug-log-kind {
            display: inline-block;
            padding: 1px 6px;
            margin-left: 6px;
            font-size: 10px;
            letter-spacing: 0.5px;
            text-transform: uppercase;
            border: 1px solid var(--hairline);
        }

        .debug-log-kind.checkpoint { color: var(--accent); border-color: var(--accent); }
        .debug-log-kind.heartbeat { color: #3fb3c9; border-color: #3fb3c9; }

        .debug-empty {
            color: var(--muted);
            font-size: 12px;
            padding: 8px 0;
        }
"""

DEBUG_PANEL_HTML = """
    <div class="debug-tab" onclick="document.getElementById('debugPanel').classList.add('open')">Debug</div>
    <div class="debug-panel" id="debugPanel">
        <div class="debug-header">
            <div class="eyebrow">Node connectivity</div>
            <button class="debug-close" onclick="document.getElementById('debugPanel').classList.remove('open')">Close</button>
        </div>
        <div class="debug-section" id="debugNodes">
            <div class="debug-section-label">Nodes</div>
        </div>
        <div class="debug-section-label" style="padding: 12px 18px 0;">Raw request log</div>
        <div class="debug-log" id="debugLog"></div>
    </div>
    <script>
        // Sub-second polling: fetches are cheap in-memory reads on the Pi,
        // and this guard skips a tick if the previous fetch hasn't landed
        // yet so a slow network hiccup can't pile up overlapping requests.
        const DEBUG_POLL_MS = 150;
        let debugTickInFlight = false;

        function formatAge(ageSeconds) {
            const ms = ageSeconds * 1000;
            if (ms < 1000) return Math.round(ms) + 'ms ago';
            return ageSeconds.toFixed(2) + 's ago';
        }

        async function debugTick() {
            if (debugTickInFlight) return;
            debugTickInFlight = true;

            let data;
            try {
                const res = await fetch('/api/debug');
                data = await res.json();
            } catch (e) {
                debugTickInFlight = false;
                return;
            }

            const nodesEl = document.getElementById('debugNodes');
            nodesEl.innerHTML = '<div class="debug-section-label">Nodes</div>';
            if (data.nodes.length === 0) {
                nodesEl.innerHTML += '<div class="debug-empty">No nodes have contacted the server yet.</div>';
            }
            data.nodes.forEach(n => {
                const row = document.createElement('div');
                row.className = 'debug-node-row';
                row.innerHTML =
                    '<span class="debug-dot ' + (n.online ? 'online' : 'offline') + '"></span>' +
                    '<span class="debug-node-id">' + n.node_id + '</span>' +
                    '<span class="debug-node-meta">' + formatAge(n.age) + ' &middot; ' + n.ip + '</span>';
                nodesEl.appendChild(row);
            });

            const logEl = document.getElementById('debugLog');
            logEl.innerHTML = '';
            if (data.log.length === 0) {
                logEl.innerHTML = '<div class="debug-empty">No requests logged yet.</div>';
            }
            data.log.forEach(e => {
                const row = document.createElement('div');
                row.className = 'debug-log-row';
                let detail = '';
                if (e.kind === 'checkpoint') {
                    detail = 'drone ' + e.detail.drone_id + ' &middot; ' + e.detail.rssi + ' dBm';
                } else if (e.kind === 'heartbeat') {
                    detail = e.detail.wifi_rssi !== null && e.detail.wifi_rssi !== undefined
                        ? 'wifi ' + e.detail.wifi_rssi + ' dBm' : '';
                }
                row.innerHTML =
                    '<span class="debug-log-time">' + e.time_str + '</span>' +
                    '<span class="debug-log-kind ' + e.kind + '">' + e.kind + '</span><br>' +
                    '<strong>' + e.node_id + '</strong> ' + detail;
                logEl.appendChild(row);
            });

            debugTickInFlight = false;
        }

        debugTick();
        setInterval(debugTick, DEBUG_POLL_MS);
    </script>
"""

LEADERBOARD_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>KWAD // Live Track</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta charset="UTF-8">
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
    <link href="https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@500;700&family=JetBrains+Mono:wght@400;500;700&display=swap" rel="stylesheet">
    <style>
""" + BASE_STYLE + """
    </style>
</head>
<body>
    <div class="nav">
        <a href="/" class="active">Leaderboard</a>
        <a href="/radar">Radar</a>
    </div>
    <div class="wrap">
        <div class="panel">
            <div class="eyebrow">Checkpoint telemetry</div>
            <div class="title-row">
                <h1>KWAD // Live Track</h1>
                <span class="live"><span class="live-dot"></span>Live</span>
            </div>
            <p class="subhead">Real-time checkpoint passes reported by race nodes, relayed to base over the timing network.</p>

            <div class="stats">
                <div class="stat">
                    <div class="stat-label">Total passes</div>
                    <div class="flap-row" id="flapRow"></div>
                </div>
                <div class="stat">
                    <div class="stat-label">Last node</div>
                    <div class="stat-value" id="lastNode">—</div>
                </div>
                <div class="stat">
                    <div class="stat-label">Refresh interval</div>
                    <div class="stat-value">Live</div>
                </div>
            </div>
        </div>

        <div class="panel">
            <div class="table-scroll">
                <table>
                    <thead>
                        <tr><th>#</th><th>Node</th><th>Drone</th><th>RSSI</th><th>Sent</th><th>Received &middot; Pi</th></tr>
                    </thead>
                    <tbody id="eventsBody"></tbody>
                </table>
            </div>
            <div class="empty" id="emptyState" style="display:none;">Waiting for checkpoint signal &hellip;</div>
        </div>
    </div>
""" + DEBUG_PANEL_HTML + """
    <script>
        const flapRowEl = document.getElementById('flapRow');
        const lastNodeEl = document.getElementById('lastNode');
        const bodyEl = document.getElementById('eventsBody');
        const emptyEl = document.getElementById('emptyState');

        function esc(s) {
            return String(s).replace(/[&<>"']/g, c => ({
                '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
            }[c]));
        }

        async function leaderboardTick() {
            let data;
            try {
                const res = await fetch('/api/leaderboard');
                data = await res.json();
            } catch (e) {
                return;
            }

            const digits = String(data.count).padStart(3, '0').split('');
            flapRowEl.innerHTML = digits.map(d => '<span class="flap-digit">' + d + '</span>').join('');

            lastNodeEl.textContent = data.events.length ? data.events[0].node_id : '—';

            emptyEl.style.display = data.events.length ? 'none' : 'block';
            bodyEl.innerHTML = data.events.map((e, i) => (
                '<tr>' +
                '<td class="idx">' + String(i + 1).padStart(2, '0') + '</td>' +
                '<td><span class="node-pill">' + esc(e.node_id) + '</span></td>' +
                '<td>' + esc(e.drone_id) + '</td>' +
                '<td>' + esc(e.rssi) + '</td>' +
                '<td>' + esc(e.timestamp) + '</td>' +
                '<td>' + esc(e.received_at) + '</td>' +
                '</tr>'
            )).join('');
        }

        leaderboardTick();
        setInterval(leaderboardTick, 1000);
    </script>
</body>
</html>
"""

RADAR_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>KWAD // Radar</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta charset="UTF-8">
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
    <link href="https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@500;700&family=JetBrains+Mono:wght@400;500;700&display=swap" rel="stylesheet">
    <style>
""" + BASE_STYLE + """
        .radar-square {
            position: relative;
            width: 100%;
            aspect-ratio: 1 / 1;
            background:
                linear-gradient(var(--hairline) 1px, transparent 1px) 0 0 / 10% 10%,
                linear-gradient(90deg, var(--hairline) 1px, transparent 1px) 0 0 / 10% 10%,
                var(--bg);
            border: 1px solid var(--hairline);
            overflow: hidden;
        }

        .node-marker {
            position: absolute;
            width: 14px;
            height: 14px;
            transform: translate(-50%, -50%);
            border: 1.5px solid var(--muted);
            background: var(--panel);
            display: flex;
            align-items: center;
            justify-content: center;
        }

        .node-label {
            position: absolute;
            transform: translate(-50%, 10px);
            top: 100%;
            font-size: 10px;
            letter-spacing: 1px;
            color: var(--muted);
            white-space: nowrap;
            text-transform: uppercase;
        }

        .drone-marker {
            position: absolute;
            width: 12px;
            height: 12px;
            border-radius: 50%;
            transform: translate(-50%, -50%);
            transition: left 0.4s ease, top 0.4s ease, opacity 0.4s ease;
        }

        .drone-marker .ring {
            position: absolute;
            inset: -14px;
            border-radius: 50%;
            border: 1px solid currentColor;
            opacity: 0.5;
            animation: radar-ping 1.6s ease-out infinite;
        }

        @keyframes radar-ping {
            0% { transform: scale(0.4); opacity: 0.6; }
            100% { transform: scale(2.4); opacity: 0; }
        }

        .drone-tag {
            position: absolute;
            left: 16px;
            top: -6px;
            font-size: 10px;
            letter-spacing: 1px;
            white-space: nowrap;
            font-weight: 700;
        }

        .legend {
            display: flex;
            gap: 16px;
            margin-top: 16px;
            flex-wrap: wrap;
        }

        .legend-item {
            display: flex;
            align-items: center;
            gap: 8px;
            font-size: 12px;
            color: var(--muted);
        }

        .legend-dot {
            width: 10px;
            height: 10px;
            border-radius: 50%;
        }
    </style>
</head>
<body>
    <div class="nav">
        <a href="/">Leaderboard</a>
        <a href="/radar" class="active">Radar</a>
    </div>
    <div class="wrap">
        <div class="panel">
            <div class="eyebrow">Proximity radar</div>
            <div class="title-row">
                <h1>KWAD // Radar</h1>
                <span class="live"><span class="live-dot"></span>Live</span>
            </div>
            <p class="subhead">Drones snap to the checkpoint that most recently detected them and fade out after a few seconds of silence. This reflects discrete pass events, not continuous position.</p>
        </div>

        <div class="panel">
            <div class="radar-square" id="radar"></div>
            <div class="legend" id="legend"></div>
        </div>
    </div>

    <script>
        const radarEl = document.getElementById('radar');
        const legendEl = document.getElementById('legend');

        async function tick() {
            let data;
            try {
                const res = await fetch('/api/radar');
                data = await res.json();
            } catch (e) {
                return;
            }

            radarEl.querySelectorAll('.node-marker, .node-label, .drone-marker').forEach(el => el.remove());

            data.nodes.forEach(n => {
                const marker = document.createElement('div');
                marker.className = 'node-marker';
                marker.style.left = n.x + '%';
                marker.style.top = n.y + '%';
                radarEl.appendChild(marker);

                const label = document.createElement('div');
                label.className = 'node-label';
                label.style.left = n.x + '%';
                label.style.top = n.y + '%';
                label.textContent = n.id;
                radarEl.appendChild(label);
            });

            legendEl.innerHTML = '';
            data.drones.forEach(d => {
                const marker = document.createElement('div');
                marker.className = 'drone-marker';
                marker.style.left = d.x + '%';
                marker.style.top = d.y + '%';
                marker.style.color = d.color;
                marker.style.background = d.color;
                marker.style.opacity = d.opacity;
                marker.style.boxShadow = '0 0 ' + (6 + d.strength * 10) + 'px ' + d.color;

                const ring = document.createElement('div');
                ring.className = 'ring';
                marker.appendChild(ring);

                const tag = document.createElement('div');
                tag.className = 'drone-tag';
                tag.style.color = d.color;
                tag.textContent = 'DRONE ' + d.id;
                marker.appendChild(tag);

                radarEl.appendChild(marker);

                const item = document.createElement('div');
                item.className = 'legend-item';
                item.innerHTML = '<span class="legend-dot" style="background:' + d.color + '"></span>' +
                    'Drone ' + d.id + ' &middot; last @ ' + d.last_node + ' &middot; ' + d.rssi + ' dBm &middot; ' + d.age.toFixed(1) + 's ago';
                legendEl.appendChild(item);
            });

            if (data.drones.length === 0) {
                legendEl.innerHTML = '<div class="legend-item">No drones detected yet.</div>';
            }
        }

        tick();
        setInterval(tick, 400);
    </script>
""" + DEBUG_PANEL_HTML + """
</body>
</html>
"""

@app.route("/")
def leaderboard():
    return render_template_string(LEADERBOARD_PAGE)

@app.route("/api/leaderboard")
def api_leaderboard():
    with events_lock:
        # show most recent first
        recent = list(reversed(events))
    return jsonify({"events": recent, "count": len(events)})

@app.route("/radar")
def radar():
    return render_template_string(RADAR_PAGE)

@app.route("/api/radar")
def api_radar():
    now = time.time()

    with events_lock:
        known_nodes = sorted({e["node_id"] for e in events} | set(NODE_POSITIONS.keys()))

    nodes = []
    for node_id in known_nodes:
        x, y = get_node_position(node_id)
        nodes.append({"id": node_id, "x": round(x, 1), "y": round(y, 1)})

    drones = []
    with drone_state_lock:
        for drone_id, state in list(drone_state.items()):
            age = now - state["last_seen"]
            if age > DRONE_TIMEOUT_S:
                continue
            x, y = get_node_position(state["node_id"])
            # Fresher sightings render more opaque; fades out toward the timeout.
            opacity = max(0.15, 1 - (age / DRONE_TIMEOUT_S))
            # RSSI roughly -40 (very strong) to -90 (weak) -> 1.0 to 0.1 strength.
            strength = max(0.1, min(1.0, (state["rssi"] + 90) / 50))
            drones.append({
                "id": drone_id,
                "x": round(x, 1),
                "y": round(y, 1),
                "color": get_drone_color(drone_id),
                "opacity": round(opacity, 2),
                "strength": round(strength, 2),
                "last_node": state["node_id"],
                "rssi": state["rssi"],
                "age": age,
            })

    return jsonify({"nodes": nodes, "drones": drones})

@app.route("/checkpoint", methods=["POST"])
def checkpoint():
    data = request.get_json(force=True, silent=True)
    if not data or "node_id" not in data:
        return jsonify({"error": "expected JSON with at least a node_id field"}), 400

    node_id = data.get("node_id")
    drone_id = str(data.get("drone_id", "1"))
    try:
        rssi = int(data.get("rssi"))
    except (TypeError, ValueError):
        rssi = None

    event = {
        "node_id": node_id,
        "drone_id": drone_id,
        "rssi": rssi if rssi is not None else "n/a",
        "timestamp": data.get("timestamp", "n/a"),
        "received_at": datetime.now().strftime("%H:%M:%S.%f")[:-3],
    }

    with events_lock:
        events.append(event)

    if rssi is not None:
        with drone_state_lock:
            drone_state[drone_id] = {
                "node_id": node_id,
                "rssi": rssi,
                "last_seen": time.time(),
                "sequence": data.get("sequence"),
            }

    record_contact(node_id, "checkpoint", drone_id=drone_id, rssi=event["rssi"])

    print(f"[checkpoint] {event}")
    return jsonify({"status": "ok", "event": event}), 200

@app.route("/api/heartbeat", methods=["POST"])
def heartbeat():
    data = request.get_json(force=True, silent=True) or {}
    node_id = data.get("node_id")
    if not node_id:
        return jsonify({"error": "expected JSON with a node_id field"}), 400

    wifi_rssi = data.get("wifi_rssi")
    record_contact(node_id, "heartbeat", wifi_rssi=wifi_rssi)

    return jsonify({"status": "ok"}), 200

@app.route("/api/debug")
def api_debug():
    now = time.time()

    nodes = []
    with node_registry_lock:
        for node_id, state in sorted(node_registry.items()):
            age = now - state["last_seen"]
            nodes.append({
                "node_id": node_id,
                "online": age <= NODE_ONLINE_TIMEOUT_S,
                "age": round(age, 1),
                "last_type": state["last_type"],
                "ip": state["ip"],
                "wifi_rssi": state.get("wifi_rssi"),
            })

    with raw_log_lock:
        log = list(reversed(raw_log))

    return jsonify({"nodes": nodes, "log": log})

@app.route("/health")
def health():
    return jsonify({"status": "alive"}), 200

if __name__ == "__main__":
    # 0.0.0.0 so ESP32s on the same WiFi can reach it, not just localhost
    app.run(host="0.0.0.0", port=5000, debug=True)
