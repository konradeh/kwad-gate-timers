from flask import Flask, request, jsonify, render_template_string
from collections import defaultdict, deque
from datetime import datetime
from pathlib import Path
import hashlib
import json
import math
import os
import threading
import time

app = Flask(__name__)

APP_VERSION = "1.1.0"

# In-memory event log - list of dicts: {node_id, timestamp, received_at, drone_id, rssi}
events = []
events_lock = threading.Lock()

# Last-contact registry per node_id, updated by both /checkpoint and
# /api/heartbeat. This is what powers the debug panel's online/offline dots.
# {"checkpoint-1": {"last_seen": <epoch>, "last_type": "heartbeat", "ip": "...", "wifi_rssi": -50}}
node_registry = {}
node_registry_lock = threading.Lock()

# Rolling log of requests, kept PER NODE (not one shared buffer) so a node
# heartbeating fast can't crowd another node's entries out of view. Each
# node gets its own capped history; the debug panel merges/filters them.
raw_log_by_node = defaultdict(lambda: deque(maxlen=30))
raw_log_lock = threading.Lock()

# A node is considered "online" if we've heard from it (heartbeat or
# checkpoint event) within this many seconds.
NODE_ONLINE_TIMEOUT_S = 8.0

# A node's live drone-proximity reading (piggybacked on its heartbeat) is
# considered current if the sample is fresher than this, in milliseconds.
DRONE_LIVE_TIMEOUT_MS = 3000

# ESP32 WiFi disconnect reason codes -> human labels, so the debug panel can
# explain WHY a node dropped instead of just showing a bare number. Only the
# common ones are mapped; anything else falls back to "reason N".
WIFI_DISCONNECT_REASONS = {
    1: "unspecified",
    2: "auth expired",
    3: "auth leave",
    4: "assoc expired (idle/kicked)",
    5: "too many assoc",
    6: "not authed",
    7: "not assoced",
    8: "assoc leave (AP kicked it)",
    15: "4-way handshake timeout",
    200: "beacon timeout (weak signal)",
    201: "no AP found (out of range/channel)",
    202: "auth fail",
    203: "assoc fail",
    204: "handshake timeout",
}

def disconnect_reason_label(reason):
    if reason is None or reason == 0:
        return None
    return WIFI_DISCONNECT_REASONS.get(reason, f"reason {reason}")

def record_contact(node_id, kind, **extra):
    now = time.time()
    with node_registry_lock:
        # Merge onto the previous entry rather than replacing it outright -
        # a /checkpoint pass event doesn't carry wifi_rssi/fw_version, and
        # a plain replace would wipe those out until the next heartbeat.
        previous = node_registry.get(node_id, {})
        node_registry[node_id] = {
            **previous,
            "last_seen": now,
            "last_type": kind,
            "ip": request.remote_addr,
            **extra,
        }
    with raw_log_lock:
        raw_log_by_node[node_id].append({
            "ts": now,
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

# Every drone_id ever reported, so a drone that has gone quiet still shows
# in the UI as offline rather than silently vanishing from the list.
known_drone_ids = set()
known_drone_lock = threading.Lock()

def remember_drone(drone_id):
    if drone_id is None:
        return
    with known_drone_lock:
        known_drone_ids.add(str(drone_id))

# ---- Per-checkpoint gate-timing settings ----
# These mirror the tunable constants in wroom_code_v2.ino. Firmware fetches
# its own settings from GET /api/settings/<node_id> on boot and re-polls
# periodically, so changes made here take effect without re-flashing.
DEFAULT_SETTINGS = {
    "enter_rssi": -62,
    "exit_rssi": -72,
    "required_weak_samples": 5,
    "pass_timeout_ms": 400,
    "event_cooldown_ms": 2000,
    "heartbeat_interval_ms": 1000,
}

# Bounds used to sanity-check values coming from the settings form before
# they're handed to a physical board.
SETTINGS_BOUNDS = {
    "enter_rssi": (-100, 0),
    "exit_rssi": (-100, 0),
    "required_weak_samples": (1, 255),
    "pass_timeout_ms": (50, 60000),
    "event_cooldown_ms": (0, 60000),
    "heartbeat_interval_ms": (200, 60000),
}

SETTINGS_FILE = Path(__file__).parent / "node_settings.json"
settings_lock = threading.Lock()

def load_node_settings():
    try:
        return json.loads(SETTINGS_FILE.read_text())
    except (FileNotFoundError, ValueError):
        return {}

def save_node_settings(data):
    SETTINGS_FILE.write_text(json.dumps(data, indent=2))

# Per-node overrides on top of DEFAULT_SETTINGS, persisted to disk so a Pi
# reboot doesn't silently reset every checkpoint back to defaults.
node_settings = load_node_settings()

def get_effective_settings(node_id):
    effective = dict(DEFAULT_SETTINGS)
    with settings_lock:
        effective.update(node_settings.get(node_id, {}))
    return effective

# Where each checkpoint node sits on the radar square, as a percentage
# (0-100) from the top-left. Edit this to match your physical track layout.
NODE_POSITIONS = {
    "checkpoint-1": (50.0, 15.0),
    "checkpoint-2": (83.3, 39.2),
    "checkpoint-3": (70.6, 78.3),
    "checkpoint-4": (29.4, 78.3),
    "checkpoint-5": (16.7, 39.2),
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
# Green is reserved for brand chrome, so drones stay visually distinct from
# the site's own accent color.
DRONE_COLORS = ["#ffb545", "#3fb3c9", "#c94fd6", "#ff6f59", "#8f7cff", "#f2d94e"]

def get_drone_color(drone_id):
    try:
        idx = int(drone_id)
    except (TypeError, ValueError):
        idx = abs(hash(drone_id))
    return DRONE_COLORS[idx % len(DRONE_COLORS)]

# How long (seconds) a drone stays visible on the radar after its last
# reported pass before fading out entirely.
DRONE_TIMEOUT_S = 6.0

# Hand-traced from the kwad brand sheet: rounded-square gate + diagonal
# flight path + dot. Uses currentColor so CSS controls the color per page.
KWAD_LOGO_SVG = """<svg class="brand-mark" viewBox="0 0 100 100" xmlns="http://www.w3.org/2000/svg" fill="none">
<rect x="26" y="26" width="46" height="46" rx="12" stroke="currentColor" stroke-width="9"/>
<path d="M18 84C22 80 26 78 30 72L76 26" stroke="currentColor" stroke-width="9" stroke-linecap="round" stroke-linejoin="round"/>
<circle cx="81" cy="21" r="7" fill="currentColor"/>
</svg>"""

def topbar(active):
    links = [("/", "Leaderboard"), ("/radar", "Radar"), ("/settings", "Settings")]
    nav_html = "\n".join(
        f'<a href="{href}" class="{"active" if label == active else ""}">{label}</a>'
        for href, label in links
    )
    return f"""
    <div class="topbar">
        <a href="/" class="brand">
            {KWAD_LOGO_SVG}
            <span class="brand-name">kwad</span>
            <span class="brand-version">v{APP_VERSION}</span>
        </a>
        <div class="nav">
            {nav_html}
        </div>
    </div>
    """

BASE_STYLE = """
        :root {
            --bg: #0e1210;
            --panel: #151b17;
            --hairline: #26302a;
            --paper: #ece7de;
            --muted: #7c8790;
            --accent: #2ee06f;
            --accent-soft: rgba(46, 224, 111, 0.14);
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
            box-shadow: 0 0 6px rgba(46, 224, 111, 0.7);
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

        .topbar {
            display: flex;
            align-items: center;
            justify-content: space-between;
            gap: 16px;
            max-width: 880px;
            margin: 0 auto 18px;
            flex-wrap: wrap;
        }

        .brand {
            display: flex;
            align-items: center;
            gap: 9px;
            text-decoration: none;
            color: var(--paper);
        }

        .brand-mark {
            width: 24px;
            height: 24px;
            color: var(--accent);
            flex-shrink: 0;
        }

        .brand-name {
            font-family: 'Space Grotesk', sans-serif;
            font-size: 17px;
            font-weight: 700;
            letter-spacing: 0.2px;
            color: var(--paper);
        }

        .brand-version {
            font-family: 'JetBrains Mono', monospace;
            font-size: 10px;
            letter-spacing: 1px;
            color: var(--muted);
            border: 1px solid var(--hairline);
            padding: 2px 6px;
            border-radius: 3px;
        }

        .nav {
            display: flex;
            gap: 4px;
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

        .debug-filter {
            padding: 12px 18px;
            border-bottom: 1px solid var(--hairline);
            display: flex;
            gap: 8px;
        }

        .debug-filter select {
            flex: 1;
            min-width: 0;
            background: var(--bg);
            color: var(--paper);
            border: 1px solid var(--hairline);
            font-family: 'JetBrains Mono', monospace;
            font-size: 12px;
            padding: 7px 8px;
        }

        .debug-filter select:focus { outline: none; border-color: var(--accent); }

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
            flex-wrap: wrap;
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

        .debug-drone-rssi {
            width: 100%;
            margin-top: 3px;
            font-size: 11px;
            color: var(--accent);
        }

        .debug-drop-info {
            width: 100%;
            margin-top: 3px;
            font-size: 11px;
            color: #ff6f59;
        }

        .fw-badge {
            display: inline-block;
            font-size: 9px;
            letter-spacing: 0.5px;
            color: var(--muted);
            border: 1px solid var(--hairline);
            padding: 1px 5px;
            border-radius: 2px;
        }

        .fw-badge.fw-unknown {
            color: #ff6f59;
            border-color: #ff6f59;
        }

        .closest-badge {
            display: inline-block;
            font-size: 9px;
            letter-spacing: 1px;
            font-weight: 700;
            color: #08130c;
            background: var(--accent);
            padding: 1px 5px;
            margin-left: 4px;
            border-radius: 2px;
        }

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
        <div class="debug-section" id="debugDrones">
            <div class="debug-section-label">Drones</div>
        </div>
        <div class="debug-filter">
            <select id="debugNodeFilter" onchange="debugTick()">
                <option value="__all__">All checkpoints</option>
            </select>
            <select id="debugSortMode" onchange="debugTick()">
                <option value="proximity">Sort: closest first</option>
                <option value="node_id">Sort: node ID</option>
            </select>
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

        function formatMs(ms) {
            if (ms < 1000) return Math.round(ms) + 'ms ago';
            return (ms / 1000).toFixed(1) + 's ago';
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

            const dronesEl = document.getElementById('debugDrones');
            const droneList = data.drones || [];
            const onlineCount = droneList.filter(d => d.online).length;
            dronesEl.innerHTML = '<div class="debug-section-label">Drones &middot; ' +
                onlineCount + '/' + droneList.length + ' online</div>';

            if (droneList.length === 0) {
                dronesEl.innerHTML += '<div class="debug-empty">No drones detected yet. Power one on within range of a checkpoint.</div>';
            }
            droneList.forEach(d => {
                const row = document.createElement('div');
                row.className = 'debug-node-row';
                const detail = d.online
                    ? '<span class="debug-node-meta">@ ' + d.closest_node + ' &middot; ' + d.rssi + ' dBm</span>'
                    : '<span class="debug-node-meta">no beacon heard</span>';
                row.innerHTML =
                    '<span class="debug-dot ' + (d.online ? 'online' : 'offline') + '"></span>' +
                    '<span class="debug-node-id" style="color:' + d.color + '">Drone ' + d.id + '</span>' +
                    detail;
                dronesEl.appendChild(row);
            });

            const filterEl = document.getElementById('debugNodeFilter');
            const selected = filterEl.value || '__all__';
            const knownIds = data.nodes.map(n => n.node_id);
            const optionIds = Array.from(filterEl.options).map(o => o.value);
            if (optionIds.length !== knownIds.length + 1 || knownIds.some(id => !optionIds.includes(id))) {
                filterEl.innerHTML = '<option value="__all__">All checkpoints</option>' +
                    knownIds.map(id => '<option value="' + id + '">' + id + '</option>').join('');
                filterEl.value = optionIds.includes(selected) || selected === '__all__' ? selected : '__all__';
            }
            const activeFilter = filterEl.value || '__all__';
            const sortMode = document.getElementById('debugSortMode').value || 'proximity';

            let visibleNodes = activeFilter === '__all__'
                ? data.nodes.slice()
                : data.nodes.filter(n => n.node_id === activeFilter);

            if (sortMode === 'proximity') {
                visibleNodes.sort((a, b) => {
                    const aLive = a.drone_age_ms !== null && a.drone_age_ms <= 3000;
                    const bLive = b.drone_age_ms !== null && b.drone_age_ms <= 3000;
                    if (aLive && bLive) return b.drone_rssi - a.drone_rssi;
                    if (aLive) return -1;
                    if (bLive) return 1;
                    return a.node_id.localeCompare(b.node_id);
                });
            } else {
                visibleNodes.sort((a, b) => a.node_id.localeCompare(b.node_id));
            }

            const nodesEl = document.getElementById('debugNodes');
            nodesEl.innerHTML = '<div class="debug-section-label">Nodes</div>';
            if (visibleNodes.length === 0) {
                nodesEl.innerHTML += '<div class="debug-empty">No nodes have contacted the server yet.</div>';
            }
            visibleNodes.forEach(n => {
                const row = document.createElement('div');
                row.className = 'debug-node-row';
                const droneLive = n.drone_age_ms !== null && n.drone_age_ms <= 3000;
                const isClosest = data.drone.online && n.node_id === data.drone.closest_node;
                const droneInfo = droneLive
                    ? '<span class="debug-drone-rssi">drone ' + n.drone_rssi + ' dBm &middot; ' + formatMs(n.drone_age_ms) +
                      (isClosest ? ' <span class="closest-badge">CLOSEST</span>' : '') + '</span>'
                    : '';
                const fwBadge = n.fw_version
                    ? '<span class="fw-badge">fw ' + n.fw_version + '</span>'
                    : '<span class="fw-badge fw-unknown">fw ?</span>';
                const dropInfo = (n.disconnect_count)
                    ? '<span class="debug-drop-info">dropped ' + n.disconnect_count + '&times;' +
                      (n.last_disc_label ? ' &middot; last: ' + n.last_disc_label : '') + '</span>'
                    : '';
                row.innerHTML =
                    '<span class="debug-dot ' + (n.online ? 'online' : 'offline') + '"></span>' +
                    '<span class="debug-node-id">' + n.node_id + '</span>' +
                    fwBadge +
                    '<span class="debug-node-meta">' + formatAge(n.age) + ' &middot; ' + n.ip + '</span>' +
                    droneInfo +
                    dropInfo;
                nodesEl.appendChild(row);
            });

            const visibleLog = activeFilter === '__all__'
                ? data.log
                : data.log.filter(e => e.node_id === activeFilter);

            const logEl = document.getElementById('debugLog');
            logEl.innerHTML = '';
            if (visibleLog.length === 0) {
                logEl.innerHTML = '<div class="debug-empty">No requests logged yet.</div>';
            }
            visibleLog.slice(0, 60).forEach(e => {
                const row = document.createElement('div');
                row.className = 'debug-log-row';
                let detail = '';
                if (e.kind === 'checkpoint') {
                    detail = 'drone ' + e.detail.drone_id + ' &middot; ' + e.detail.rssi + ' dBm';
                } else if (e.kind === 'heartbeat') {
                    detail = e.detail.wifi_rssi !== null && e.detail.wifi_rssi !== undefined
                        ? 'wifi ' + e.detail.wifi_rssi + ' dBm' : '';
                    if (e.detail.drone_rssi !== null && e.detail.drone_rssi !== undefined) {
                        detail += ' &middot; drone ' + e.detail.drone_rssi + ' dBm';
                    }
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
    <title>kwad — Live Track</title>
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
    {{ topbar_html|safe }}
    <div class="wrap">
        <div class="panel">
            <div class="eyebrow">Checkpoint telemetry</div>
            <div class="title-row">
                <h1>Live Track</h1>
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
    <title>kwad — Radar</title>
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
    {{ topbar_html|safe }}
    <div class="wrap">
        <div class="panel">
            <div class="eyebrow">Proximity radar</div>
            <div class="title-row">
                <h1>Radar</h1>
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

SETTINGS_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>kwad — Settings</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta charset="UTF-8">
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
    <link href="https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@500;700&family=JetBrains+Mono:wght@400;500;700&display=swap" rel="stylesheet">
    <style>
""" + BASE_STYLE + """
        .settings-select-row {
            display: flex;
            gap: 10px;
            flex-wrap: wrap;
            align-items: center;
        }

        .settings-select-row select {
            flex: 1;
            min-width: 200px;
            background: var(--bg);
            color: var(--paper);
            border: 1px solid var(--hairline);
            font-family: 'JetBrains Mono', monospace;
            font-size: 13px;
            padding: 10px 12px;
        }

        .settings-select-row select:focus { outline: none; border-color: var(--accent); }

        .btn {
            font-family: 'JetBrains Mono', monospace;
            font-size: 12px;
            letter-spacing: 1px;
            text-transform: uppercase;
            padding: 10px 16px;
            border: 1px solid var(--hairline);
            background: var(--panel);
            color: var(--paper);
            cursor: pointer;
        }

        .btn:hover { border-color: var(--accent); color: var(--accent); }

        .btn-primary {
            background: var(--accent);
            border-color: var(--accent);
            color: #08130c;
            font-weight: 700;
        }

        .btn-primary:hover { color: #08130c; opacity: 0.9; }

        .btn:disabled { opacity: 0.4; cursor: not-allowed; }

        .field-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
            gap: 18px;
            margin-top: 22px;
        }

        .field label {
            display: block;
            font-size: 11px;
            letter-spacing: 1.5px;
            text-transform: uppercase;
            color: var(--paper);
            margin-bottom: 6px;
        }

        .field input {
            width: 100%;
            background: var(--bg);
            color: var(--paper);
            border: 1px solid var(--hairline);
            font-family: 'JetBrains Mono', monospace;
            font-size: 14px;
            padding: 9px 10px;
        }

        .field input:focus { outline: none; border-color: var(--accent); }

        .field-hint {
            margin-top: 5px;
            font-size: 11px;
            color: var(--muted);
            line-height: 1.4;
        }

        .settings-actions {
            display: flex;
            gap: 10px;
            margin-top: 24px;
            align-items: center;
        }

        .settings-status {
            font-size: 12px;
            letter-spacing: 0.5px;
        }

        .settings-status.ok { color: var(--accent); }
        .settings-status.err { color: #ff6f59; }

        .override-badge {
            font-size: 10px;
            letter-spacing: 1px;
            text-transform: uppercase;
            color: var(--accent);
            border: 1px solid var(--accent);
            padding: 2px 7px;
        }
    </style>
</head>
<body>
    {{ topbar_html|safe }}
    <div class="wrap">
        <div class="panel">
            <div class="eyebrow">Checkpoint configuration</div>
            <div class="title-row">
                <h1>Settings</h1>
            </div>
            <p class="subhead">Gate-timing and heartbeat parameters per checkpoint. Boards fetch these from the Pi on boot and re-poll periodically, so changes here apply live without re-flashing &mdash; as long as the board is already running firmware that fetches settings.</p>

            <div class="settings-select-row" style="margin-top: 22px;">
                <select id="nodeSelect"></select>
                <span id="overrideBadge" class="override-badge" style="display:none;">Customized</span>
                <button class="btn" id="resetBtn" type="button">Reset to defaults</button>
            </div>
        </div>

        <div class="panel">
            <form id="settingsForm">
                <div class="field-grid" id="fieldGrid"></div>
                <div class="settings-actions">
                    <button class="btn btn-primary" type="submit">Save</button>
                    <span class="settings-status" id="statusMsg"></span>
                </div>
            </form>
        </div>
    </div>
""" + DEBUG_PANEL_HTML + """
    <script>
        const FIELD_META = [
            { key: 'enter_rssi', label: 'Enter RSSI (dBm)', hint: 'Threshold to begin tracking a pass. Closer to 0 = drone must be closer.' },
            { key: 'exit_rssi', label: 'Exit RSSI (dBm)', hint: 'Threshold considered outside the gate zone.' },
            { key: 'required_weak_samples', label: 'Required weak samples', hint: 'Consecutive weak readings needed to close out a pass.' },
            { key: 'pass_timeout_ms', label: 'Pass timeout (ms)', hint: 'Force-close a pass if the signal drops out completely.' },
            { key: 'event_cooldown_ms', label: 'Event cooldown (ms)', hint: 'Minimum time between valid passes for the same drone.' },
            { key: 'heartbeat_interval_ms', label: 'Heartbeat interval (ms)', hint: 'How often this node pings the Pi to prove it is alive. Keep this at 1000ms or higher — faster intervals multiply TCP connection churn and can cause "connection refused" errors on weak-signal boards without making the debug panel feel any more live (it already polls independently).' },
        ];

        const nodeSelect = document.getElementById('nodeSelect');
        const fieldGrid = document.getElementById('fieldGrid');
        const overrideBadge = document.getElementById('overrideBadge');
        const statusMsg = document.getElementById('statusMsg');
        const resetBtn = document.getElementById('resetBtn');

        let currentData = null;

        function renderFields(values) {
            fieldGrid.innerHTML = FIELD_META.map(f => `
                <div class="field">
                    <label for="f_${f.key}">${f.label}</label>
                    <input type="number" id="f_${f.key}" name="${f.key}" value="${values[f.key]}">
                    <div class="field-hint">${f.hint}</div>
                </div>
            `).join('');
        }

        // clearStatus=false is used when reloading right after a save/reset,
        // so the just-set confirmation message survives the refresh instead
        // of being wiped out immediately.
        async function loadAll(clearStatus = true) {
            const res = await fetch('/api/settings');
            currentData = await res.json();

            const selected = nodeSelect.value;
            const nodeIds = Object.keys(currentData.nodes).sort();
            nodeSelect.innerHTML = nodeIds.map(id => `<option value="${id}">${id}</option>`).join('');
            if (nodeIds.includes(selected)) nodeSelect.value = selected;

            refreshFields(clearStatus);
        }

        function refreshFields(clearStatus) {
            const nodeId = nodeSelect.value;
            if (!nodeId || !currentData) return;
            renderFields(currentData.nodes[nodeId]);
            overrideBadge.style.display = currentData.overridden.includes(nodeId) ? 'inline-block' : 'none';
            if (clearStatus) statusMsg.textContent = '';
        }

        nodeSelect.addEventListener('change', () => refreshFields(true));

        document.getElementById('settingsForm').addEventListener('submit', async (ev) => {
            ev.preventDefault();
            const nodeId = nodeSelect.value;
            if (!nodeId) return;

            const payload = {};
            FIELD_META.forEach(f => {
                payload[f.key] = document.getElementById('f_' + f.key).value;
            });

            statusMsg.textContent = 'Saving...';
            statusMsg.className = 'settings-status';

            try {
                const res = await fetch('/api/settings/' + encodeURIComponent(nodeId), {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(payload)
                });
                const body = await res.json();
                if (!res.ok) throw new Error(body.error || 'Save failed');

                statusMsg.textContent = 'Saved — ' + nodeId + ' will pick this up on its next settings poll.';
                statusMsg.className = 'settings-status ok';
                await loadAll(false);
            } catch (e) {
                statusMsg.textContent = e.message;
                statusMsg.className = 'settings-status err';
            }
        });

        resetBtn.addEventListener('click', async () => {
            const nodeId = nodeSelect.value;
            if (!nodeId) return;

            await fetch('/api/settings/' + encodeURIComponent(nodeId), { method: 'DELETE' });
            statusMsg.textContent = 'Reset to defaults.';
            statusMsg.className = 'settings-status ok';
            await loadAll(false);
        });

        loadAll();
    </script>
</body>
</html>
"""

@app.route("/")
def leaderboard():
    return render_template_string(LEADERBOARD_PAGE, topbar_html=topbar("Leaderboard"))

@app.route("/api/leaderboard")
def api_leaderboard():
    with events_lock:
        # show most recent first
        recent = list(reversed(events))
    return jsonify({"events": recent, "count": len(events)})

@app.route("/radar")
def radar():
    return render_template_string(RADAR_PAGE, topbar_html=topbar("Radar"))

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

    remember_drone(drone_id)
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
    extra = {"wifi_rssi": wifi_rssi, "fw_version": data.get("fw_version")}
    # WiFi drop diagnostics - only sent by firmware that tracks them, so
    # guard rather than clobbering with None on older firmware.
    if "disconnect_count" in data:
        extra["disconnect_count"] = data.get("disconnect_count")
        extra["last_disc_reason"] = data.get("last_disc_reason")
    # Live drone-proximity fields are only present once a node has actually
    # heard an ESP-NOW beacon - not every heartbeat carries them.
    if "drone_rssi" in data:
        extra["drone_id"] = data.get("drone_id")
        extra["drone_rssi"] = data.get("drone_rssi")
        extra["drone_age_ms"] = data.get("drone_age_ms")
        remember_drone(data.get("drone_id"))

    # Newer firmware reports EVERY drone this node currently hears, which is
    # what per-drone online status is built from.
    if isinstance(data.get("drones"), list):
        extra["drones"] = data["drones"]
        for entry in data["drones"]:
            if isinstance(entry, dict):
                remember_drone(entry.get("id"))
    record_contact(node_id, "heartbeat", **extra)

    # Piggyback the node's current effective settings on the heartbeat
    # response instead of making firmware run a second, separate polling
    # task for /api/settings. Two concurrent HTTP tasks contending for one
    # ESP32's single WiFi radio was causing periodic multi-second stalls -
    # one request per heartbeat cycle does both jobs.
    return jsonify({"status": "ok", **get_effective_settings(node_id)}), 200

@app.route("/api/debug")
def api_debug():
    now = time.time()

    nodes = []
    # Flattened (node_id, drone_id, rssi, live_age_ms) rows gathered from
    # every node's per-drone report, used to build per-drone status below.
    drone_sightings = []

    with node_registry_lock:
        for node_id, state in sorted(node_registry.items()):
            age = now - state["last_seen"]

            # drone_age_ms is how stale the sample already was when the
            # node sent it; add time elapsed since the Pi received that
            # heartbeat so this keeps counting up live between heartbeats,
            # not just jump on each new one.
            drone_age_ms = state.get("drone_age_ms")
            live_drone_age_ms = None
            if drone_age_ms is not None:
                live_drone_age_ms = drone_age_ms + age * 1000

            for entry in (state.get("drones") or []):
                if not isinstance(entry, dict):
                    continue
                try:
                    sighting_age = float(entry.get("age_ms", 0)) + age * 1000
                    drone_sightings.append((
                        node_id,
                        str(entry.get("id")),
                        int(entry.get("rssi")),
                        sighting_age,
                    ))
                except (TypeError, ValueError):
                    continue

            nodes.append({
                "node_id": node_id,
                "online": age <= NODE_ONLINE_TIMEOUT_S,
                "age": round(age, 1),
                "last_type": state["last_type"],
                "ip": state["ip"],
                "wifi_rssi": state.get("wifi_rssi"),
                "fw_version": state.get("fw_version"),
                "disconnect_count": state.get("disconnect_count"),
                "last_disc_reason": state.get("last_disc_reason"),
                "last_disc_label": disconnect_reason_label(state.get("last_disc_reason")),
                "drone_id": state.get("drone_id"),
                "drone_rssi": state.get("drone_rssi"),
                "drone_age_ms": round(live_drone_age_ms) if live_drone_age_ms is not None else None,
            })

    # A checkpoint has "live" drone proximity data if it heard an ESP-NOW
    # sample recently - independent of whether that node's own heartbeat
    # cadence still counts it "online" for connectivity purposes.
    live_nodes = [n for n in nodes if n["drone_age_ms"] is not None and n["drone_age_ms"] <= DRONE_LIVE_TIMEOUT_MS]
    closest_node = max(live_nodes, key=lambda n: n["drone_rssi"], default=None)

    with raw_log_lock:
        # Merge every node's own capped buffer into one time-sorted list.
        # Because each node has its own deque, a fast-heartbeating node
        # can no longer push a slower node's entries out of the response.
        merged = [entry for entries in raw_log_by_node.values() for entry in entries]

    merged.sort(key=lambda e: e["ts"], reverse=True)
    log = merged[:200]

    drone_status = {
        "online": len(live_nodes) > 0,
        "closest_node": closest_node["node_id"] if closest_node else None,
        "closest_rssi": closest_node["drone_rssi"] if closest_node else None,
    }

    # Per-drone status: for each drone, the node currently hearing it
    # loudest wins as its "closest" node.
    best_by_drone = {}
    for node_id, drone_id, rssi, sighting_age in drone_sightings:
        if sighting_age > DRONE_LIVE_TIMEOUT_MS:
            continue
        current = best_by_drone.get(drone_id)
        if current is None or rssi > current["rssi"]:
            best_by_drone[drone_id] = {
                "closest_node": node_id,
                "rssi": rssi,
                "age_ms": round(sighting_age),
            }

    with known_drone_lock:
        all_drone_ids = set(known_drone_ids)
    all_drone_ids |= set(best_by_drone.keys())

    def drone_sort_key(value):
        # Numeric IDs sort naturally; anything else falls back to string.
        try:
            return (0, int(value), "")
        except (TypeError, ValueError):
            return (1, 0, str(value))

    drones = []
    for drone_id in sorted(all_drone_ids, key=drone_sort_key):
        seen = best_by_drone.get(drone_id)
        drones.append({
            "id": drone_id,
            "online": seen is not None,
            "closest_node": seen["closest_node"] if seen else None,
            "rssi": seen["rssi"] if seen else None,
            "age_ms": seen["age_ms"] if seen else None,
            "color": get_drone_color(drone_id),
        })

    return jsonify({"nodes": nodes, "log": log, "drone": drone_status, "drones": drones})

@app.route("/settings")
def settings_page():
    return render_template_string(SETTINGS_PAGE, topbar_html=topbar("Settings"))

@app.route("/api/settings")
def api_settings_all():
    with node_registry_lock:
        known_ids = sorted(node_registry.keys())
    with settings_lock:
        overridden_ids = sorted(node_settings.keys())
    all_ids = sorted(set(known_ids) | set(overridden_ids) | set(NODE_POSITIONS.keys()))

    return jsonify({
        "defaults": DEFAULT_SETTINGS,
        "bounds": SETTINGS_BOUNDS,
        "nodes": {node_id: get_effective_settings(node_id) for node_id in all_ids},
        "overridden": overridden_ids,
    })

@app.route("/api/settings/<node_id>", methods=["GET"])
def api_settings_get(node_id):
    return jsonify(get_effective_settings(node_id))

@app.route("/api/settings/<node_id>", methods=["POST"])
def api_settings_set(node_id):
    data = request.get_json(force=True, silent=True) or {}

    cleaned = {}
    for key, (lo, hi) in SETTINGS_BOUNDS.items():
        if key not in data:
            continue
        try:
            value = int(data[key])
        except (TypeError, ValueError):
            return jsonify({"error": f"{key} must be an integer"}), 400
        if not (lo <= value <= hi):
            return jsonify({"error": f"{key} must be between {lo} and {hi}"}), 400
        cleaned[key] = value

    with settings_lock:
        node_settings.setdefault(node_id, {}).update(cleaned)
        save_node_settings(node_settings)

    return jsonify(get_effective_settings(node_id))

@app.route("/api/settings/<node_id>", methods=["DELETE"])
def api_settings_reset(node_id):
    with settings_lock:
        node_settings.pop(node_id, None)
        save_node_settings(node_settings)

    return jsonify(get_effective_settings(node_id))

@app.route("/health")
def health():
    return jsonify({"status": "alive"}), 200

if __name__ == "__main__":
    # Debug (auto-reload + Werkzeug debugger) is handy when running by hand,
    # but the boot service sets KWAD_DEBUG=0 so the interactive debugger
    # isn't left exposed on 0.0.0.0 on every power-on.
    debug_mode = os.environ.get("KWAD_DEBUG", "1") != "0"

    # 0.0.0.0 so ESP32s on the same WiFi can reach it, not just localhost.
    # threaded=True is required once more than one node is talking to the
    # Pi: Flask's dev server handles one request at a time by default, so
    # concurrent heartbeats/checkpoints from multiple ESP32s start seeing
    # "connection refused" / "read Timeout" as they queue up and time out.
    app.run(host="0.0.0.0", port=5000, debug=debug_mode, threaded=True)
