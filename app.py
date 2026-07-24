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

APP_VERSION = "2.1.0"

# -----------------------------------------------------------------------------
# Global Locks and In-Memory State
# -----------------------------------------------------------------------------

# Telemetry log - list of raw pass dicts: {node_id, timestamp, received_at, drone_id, rssi}
events = []
events_lock = threading.Lock()

# Node connectivity registry
# {"checkpoint-1": {"last_seen": <epoch>, "last_type": "heartbeat", "ip": "...", "wifi_rssi": -50}}
node_registry = {}
node_registry_lock = threading.Lock()

# Rolling log of raw HTTP requests kept PER NODE
raw_log_by_node = defaultdict(lambda: deque(maxlen=30))
raw_log_lock = threading.Lock()

# Timeouts
NODE_ONLINE_TIMEOUT_S = 8.0
DRONE_LIVE_TIMEOUT_MS = 3000
DRONE_LEGACY_TIMEOUT_MS = 10000
DRONE_TIMEOUT_S = 6.0

# Sighting registry per (node_id, drone_id)
drone_sighting_registry = {}
drone_sighting_lock = threading.Lock()

def record_drone_sighting(node_id, drone_id, rssi, age_ms, from_array):
    if drone_id is None or rssi is None:
        return
    try:
        entry = {
            "rssi": int(rssi),
            "age_ms": float(age_ms or 0),
            "last_seen": time.time(),
            "from_array": from_array,
        }
    except (TypeError, ValueError):
        return
    with drone_sighting_lock:
        drone_sighting_registry[(node_id, str(drone_id))] = entry

# WiFi Disconnect Reasons
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

# Latest drone state
drone_state = {}
drone_state_lock = threading.Lock()

known_drone_ids = set()
known_drone_lock = threading.Lock()

def remember_drone(drone_id):
    if drone_id is None:
        return
    d_str = str(drone_id)
    with known_drone_lock:
        known_drone_ids.add(d_str)

# -----------------------------------------------------------------------------
# Checkpoint Settings Configuration
# -----------------------------------------------------------------------------

DEFAULT_SETTINGS = {
    "enter_rssi": -62,
    "exit_rssi": -72,
    "required_weak_samples": 5,
    "pass_timeout_ms": 400,
    "event_cooldown_ms": 2000,
    "heartbeat_interval_ms": 1000,
}

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

node_settings = load_node_settings()

def get_effective_settings(node_id):
    effective = dict(DEFAULT_SETTINGS)
    with settings_lock:
        effective.update(node_settings.get(node_id, {}))
    return effective

# Radar map layout positions
NODE_POSITIONS = {
    "checkpoint-1": (50.0, 15.0),
    "checkpoint-2": (83.3, 39.2),
    "checkpoint-3": (70.6, 78.3),
    "checkpoint-4": (29.4, 78.3),
    "checkpoint-5": (16.7, 39.2),
}

def get_node_position(node_id):
    if node_id in NODE_POSITIONS:
        return NODE_POSITIONS[node_id]
    h = int(hashlib.md5(node_id.encode()).hexdigest(), 16)
    angle = math.radians(h % 360)
    return (50 + 35 * math.cos(angle), 50 + 35 * math.sin(angle))

DRONE_COLORS = ["#3b82f6", "#10b981", "#f59e0b", "#ec4899", "#8b5cf6", "#06b6d4"]

def get_drone_color(drone_id):
    try:
        idx = int(drone_id)
    except (TypeError, ValueError):
        idx = abs(hash(str(drone_id)))
    return DRONE_COLORS[idx % len(DRONE_COLORS)]

# -----------------------------------------------------------------------------
# Race Engine State & Gate Order Enforcement
# -----------------------------------------------------------------------------

race_lock = threading.Lock()

# Global Race State
race_state = {
    "status": "STOPPED",  # "STOPPED", "RUNNING", "FINISHED"
    "start_time": None,
    "end_time": None,
    "target_laps": 3,
    "gate_order": ["checkpoint-1", "checkpoint-2", "checkpoint-3"],
    "enforce_gate_order": True,
    "min_lap_time_s": 1.0,
}

# Per-drone race telemetry
drone_race_data = {}

# Recent race audit log
race_log = deque(maxlen=100)

def init_drone_for_race(drone_id):
    drone_id = str(drone_id)
    if drone_id not in drone_race_data:
        drone_race_data[drone_id] = {
            "drone_id": drone_id,
            "status": "READY",
            "completed_laps": 0,
            "next_gate_index": 0,
            "last_gate": None,
            "last_pass_time": None,
            "lap_start_time": race_state["start_time"] or time.time(),
            "laps": [],
            "best_lap": None,
            "total_time": None,
            "invalid_passes": 0,
        }
    return drone_race_data[drone_id]

def process_race_checkpoint(node_id, drone_id, rssi, time_str):
    now = time.time()
    drone_id = str(drone_id)

    with race_lock:
        d_data = init_drone_for_race(drone_id)

        # 1. PRACTICE MODE (Race is STOPPED)
        if race_state["status"] != "RUNNING":
            rssi_str = f"{rssi} dBm" if rssi is not None else "n/a"
            log_entry = {
                "time": time_str,
                "ts": now,
                "drone_id": drone_id,
                "node_id": node_id,
                "type": "PRACTICE_PASS",
                "message": f"[PRACTICE] Drone {drone_id} passed {node_id} (RSSI: {rssi_str})",
                "valid": True
            }
            race_log.appendleft(log_entry)
            return {
                "valid": True,
                "mode": "PRACTICE",
                "reason": "Practice pass recorded (Race stopped)",
                "race_status": race_state["status"]
            }

        # 2. FINISHED DRONE
        if d_data["status"] == "FINISHED":
            return {
                "valid": False,
                "reason": f"Drone {drone_id} has already finished the race",
                "race_status": race_state["status"]
            }

        # 3. TIMED RACE MODE (RUNNING)
        gate_order = race_state["gate_order"]
        enforce = race_state["enforce_gate_order"]

        if not gate_order:
            gate_order = [node_id]

        expected_gate = gate_order[d_data["next_gate_index"] % len(gate_order)]

        is_valid = True
        if enforce and node_id != expected_gate:
            is_valid = False

        if not is_valid:
            d_data["invalid_passes"] += 1
            log_entry = {
                "time": time_str,
                "ts": now,
                "drone_id": drone_id,
                "node_id": node_id,
                "expected_gate": expected_gate,
                "type": "SKIPPED_GATE",
                "message": f"Drone {drone_id} hit {node_id} but expected {expected_gate} (Gate {d_data['next_gate_index'] + 1}/{len(gate_order)})",
                "valid": False
            }
            race_log.appendleft(log_entry)
            return {
                "valid": False,
                "reason": f"Skipped gate. Expected {expected_gate}, got {node_id}",
                "expected_gate": expected_gate,
                "actual_gate": node_id
            }

        d_data["status"] = "RACING"
        d_data["last_gate"] = node_id
        d_data["last_pass_time"] = now

        current_gate_idx = d_data["next_gate_index"]
        next_gate_idx = (current_gate_idx + 1) % len(gate_order)
        d_data["next_gate_index"] = next_gate_idx

        lap_completed = False
        lap_time = None

        if next_gate_idx == 0:
            lap_start = d_data["lap_start_time"] or race_state["start_time"] or now
            lap_time = round(now - lap_start, 3)

            if lap_time >= race_state["min_lap_time_s"]:
                lap_completed = True
                d_data["completed_laps"] += 1
                d_data["lap_start_time"] = now

                lap_record = {
                    "lap_num": d_data["completed_laps"],
                    "lap_time": lap_time,
                    "timestamp": time_str
                }
                d_data["laps"].append(lap_record)

                if d_data["best_lap"] is None or lap_time < d_data["best_lap"]:
                    d_data["best_lap"] = lap_time

                if d_data["completed_laps"] >= race_state["target_laps"]:
                    d_data["status"] = "FINISHED"
                    total_t = round(now - race_state["start_time"], 3)
                    d_data["total_time"] = total_t

        if lap_completed:
            msg = f"Drone {drone_id} completed LAP {d_data['completed_laps']}/{race_state['target_laps']} in {lap_time:.2f}s"
            if d_data["status"] == "FINISHED":
                msg += " - FINISHED RACE!"
            entry_type = "LAP_COMPLETE"
        else:
            gate_num = current_gate_idx + 1
            msg = f"Drone {drone_id} passed Gate {gate_num}/{len(gate_order)} ({node_id})"
            entry_type = "GATE_PASS"

        log_entry = {
            "time": time_str,
            "ts": now,
            "drone_id": drone_id,
            "node_id": node_id,
            "expected_gate": expected_gate,
            "type": entry_type,
            "message": msg,
            "valid": True,
            "lap_completed": lap_completed,
            "lap_time": lap_time,
            "completed_laps": d_data["completed_laps"]
        }
        race_log.appendleft(log_entry)

        return {
            "valid": True,
            "lap_completed": lap_completed,
            "lap_time": lap_time,
            "completed_laps": d_data["completed_laps"],
            "status": d_data["status"]
        }

# -----------------------------------------------------------------------------
# Base Layout & Templates
# -----------------------------------------------------------------------------

KWAD_LOGO_SVG = """<svg class="brand-mark" viewBox="0 0 100 100" xmlns="http://www.w3.org/2000/svg" fill="none">
<rect x="26" y="26" width="46" height="46" rx="12" stroke="currentColor" stroke-width="9"/>
<path d="M18 84C22 80 26 78 30 72L76 26" stroke="currentColor" stroke-width="9" stroke-linecap="round" stroke-linejoin="round"/>
<circle cx="81" cy="21" r="7" fill="currentColor"/>
</svg>"""

def topbar(active):
    links = [("/", "Race Control"), ("/radar", "Radar"), ("/settings", "Gate Settings")]
    nav_html = "\n".join(
        f'<a href="{href}" class="nav-item {"active" if label == active else ""}">{label}</a>'
        for href, label in links
    )
    return f"""
    <header class="topbar">
        <a href="/" class="brand">
            {KWAD_LOGO_SVG}
            <span class="brand-title">kwad <span class="brand-sub">race control</span></span>
            <span class="brand-version">v{APP_VERSION}</span>
        </a>
        <nav class="nav">
            {nav_html}
        </nav>
    </header>
    """

BASE_STYLE = """
    :root {
        --bg-main: #0b0f19;
        --bg-card: #111827;
        --bg-input: #1f2937;
        --border-color: #1f2937;
        --border-focus: #3b82f6;
        --text-primary: #f9fafb;
        --text-secondary: #9ca3af;
        --text-muted: #6b7280;
        
        --accent-green: #10b981;
        --accent-green-bg: rgba(16, 185, 129, 0.12);
        --accent-blue: #3b82f6;
        --accent-blue-bg: rgba(59, 130, 246, 0.12);
        --accent-amber: #f59e0b;
        --accent-amber-bg: rgba(245, 158, 11, 0.12);
        --accent-red: #ef4444;
        --accent-red-bg: rgba(239, 68, 68, 0.12);

        --radius-sm: 6px;
        --radius-md: 10px;
        --radius-lg: 14px;
        --font-mono: 'JetBrains Mono', ui-monospace, SFMono-Regular, monospace;
        --font-sans: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
    }

    * { box-sizing: border-box; margin: 0; padding: 0; }

    body {
        font-family: var(--font-sans);
        background-color: var(--bg-main);
        color: var(--text-primary);
        line-height: 1.5;
        padding: 24px 20px 60px;
        min-height: 100vh;
    }

    .container {
        max-width: 1080px;
        margin: 0 auto;
    }

    .topbar {
        display: flex;
        align-items: center;
        justify-content: space-between;
        margin-bottom: 24px;
        padding-bottom: 16px;
        border-bottom: 1px solid var(--border-color);
        flex-wrap: wrap;
        gap: 16px;
    }

    .brand {
        display: flex;
        align-items: center;
        gap: 10px;
        text-decoration: none;
        color: var(--text-primary);
    }

    .brand-mark {
        width: 28px;
        height: 28px;
        color: var(--accent-green);
    }

    .brand-title {
        font-weight: 700;
        font-size: 18px;
        letter-spacing: -0.3px;
    }

    .brand-sub {
        font-weight: 400;
        color: var(--text-secondary);
        font-size: 14px;
        margin-left: 4px;
    }

    .brand-version {
        font-family: var(--font-mono);
        font-size: 11px;
        color: var(--text-muted);
        background: var(--bg-input);
        padding: 2px 6px;
        border-radius: var(--radius-sm);
        border: 1px solid var(--border-color);
    }

    .nav {
        display: flex;
        gap: 8px;
    }

    .nav-item {
        text-decoration: none;
        color: var(--text-secondary);
        font-size: 13px;
        font-weight: 500;
        padding: 8px 14px;
        border-radius: var(--radius-sm);
        transition: all 0.15s ease;
        border: 1px solid transparent;
    }

    .nav-item:hover {
        color: var(--text-primary);
        background: var(--bg-card);
    }

    .nav-item.active {
        color: var(--accent-green);
        background: var(--accent-green-bg);
        border-color: rgba(16, 185, 129, 0.3);
    }

    .card {
        background: var(--bg-card);
        border: 1px solid var(--border-color);
        border-radius: var(--radius-md);
        padding: 20px 24px;
        margin-bottom: 20px;
    }

    .card-title {
        font-size: 16px;
        font-weight: 600;
        color: var(--text-primary);
        display: flex;
        align-items: center;
        gap: 10px;
        margin-bottom: 4px;
    }

    .card-subtitle {
        font-size: 13px;
        color: var(--text-secondary);
        margin-bottom: 16px;
    }

    .btn {
        display: inline-flex;
        align-items: center;
        justify-content: center;
        gap: 8px;
        font-family: var(--font-sans);
        font-size: 13px;
        font-weight: 600;
        padding: 9px 18px;
        border-radius: var(--radius-sm);
        border: 1px solid var(--border-color);
        background: var(--bg-input);
        color: var(--text-primary);
        cursor: pointer;
        transition: all 0.15s ease;
    }

    .btn:hover {
        border-color: var(--text-muted);
        background: #273548;
    }

    .btn-success {
        background: var(--accent-green);
        border-color: var(--accent-green);
        color: #042f1a;
    }
    .btn-success:hover { background: #059669; border-color: #059669; color: #fff; }

    .btn-danger {
        background: var(--accent-red);
        border-color: var(--accent-red);
        color: #ffffff;
    }
    .btn-danger:hover { background: #dc2626; border-color: #dc2626; }

    .btn-warning {
        background: var(--accent-amber);
        border-color: var(--accent-amber);
        color: #451a03;
    }

    .btn-sm {
        padding: 5px 10px;
        font-size: 12px;
    }

    .badge {
        display: inline-flex;
        align-items: center;
        gap: 6px;
        font-size: 12px;
        font-weight: 600;
        padding: 4px 10px;
        border-radius: 9999px;
        text-transform: uppercase;
        letter-spacing: 0.5px;
    }

    .badge-stopped {
        background: var(--bg-input);
        color: var(--text-secondary);
        border: 1px solid var(--border-color);
    }

    .badge-running {
        background: var(--accent-green-bg);
        color: var(--accent-green);
        border: 1px solid rgba(16, 185, 129, 0.4);
    }

    .badge-finished {
        background: var(--accent-blue-bg);
        color: var(--accent-blue);
        border: 1px solid rgba(59, 130, 246, 0.4);
    }

    .badge-dot {
        width: 7px;
        height: 7px;
        border-radius: 50%;
        background: currentColor;
    }

    .badge-running .badge-dot {
        animation: pulse 1.5s infinite;
    }

    @keyframes pulse {
        0%, 100% { opacity: 1; }
        50% { opacity: 0.3; }
    }

    .table-responsive {
        width: 100%;
        overflow-x: auto;
    }

    table {
        width: 100%;
        border-collapse: collapse;
        text-align: left;
        font-size: 13px;
    }

    th {
        font-weight: 600;
        color: var(--text-secondary);
        padding: 10px 12px;
        border-bottom: 1px solid var(--border-color);
        font-size: 12px;
        text-transform: uppercase;
        letter-spacing: 0.5px;
    }

    td {
        padding: 12px;
        border-bottom: 1px solid var(--border-color);
        color: var(--text-primary);
    }

    tr:last-child td {
        border-bottom: none;
    }

    .font-mono {
        font-family: var(--font-mono);
        font-variant-numeric: tabular-nums;
    }

    .form-group {
        display: flex;
        flex-direction: column;
        gap: 6px;
    }

    .form-label {
        font-size: 12px;
        font-weight: 600;
        color: var(--text-secondary);
        text-transform: uppercase;
        letter-spacing: 0.5px;
    }

    .form-control {
        background: var(--bg-input);
        border: 1px solid var(--border-color);
        border-radius: var(--radius-sm);
        padding: 8px 12px;
        color: var(--text-primary);
        font-family: var(--font-sans);
        font-size: 14px;
    }

    .form-control:focus {
        outline: none;
        border-color: var(--border-focus);
    }

    /* Telemetry Debug Drawer */
    .debug-drawer {
        position: fixed;
        top: 0; right: 0; bottom: 0;
        width: 420px;
        max-width: 90vw;
        background: var(--bg-card);
        border-left: 1px solid var(--border-color);
        transform: translateX(100%);
        transition: transform 0.25s ease-in-out;
        z-index: 100;
        display: flex;
        flex-direction: column;
        box-shadow: -4px 0 24px rgba(0,0,0,0.5);
    }

    .debug-drawer.open {
        transform: translateX(0);
    }

    .debug-toggle-btn {
        position: fixed;
        bottom: 20px; right: 20px;
        z-index: 90;
        border-radius: 9999px;
        box-shadow: 0 4px 12px rgba(0,0,0,0.4);
    }

    .debug-sec-header {
        font-size: 11px;
        font-weight: 700;
        text-transform: uppercase;
        letter-spacing: 0.8px;
        color: var(--text-secondary);
        margin-bottom: 8px;
    }

    .debug-dot {
        display: inline-block;
        width: 8px; height: 8px;
        border-radius: 50%;
        margin-right: 6px;
    }
    .debug-dot.online { background: var(--accent-green); box-shadow: 0 0 6px var(--accent-green); }
    .debug-dot.offline { background: var(--accent-red); box-shadow: 0 0 6px var(--accent-red); }

    .fw-tag {
        display: inline-block;
        font-family: var(--font-mono);
        font-size: 10px;
        padding: 1px 5px;
        border-radius: 3px;
        border: 1px solid var(--border-color);
        color: var(--text-secondary);
        margin-left: 6px;
    }

    .closest-tag {
        font-family: var(--font-mono);
        font-size: 10px;
        font-weight: 700;
        color: #042f1a;
        background: var(--accent-green);
        padding: 1px 5px;
        border-radius: 3px;
        margin-left: 6px;
    }

    .log-kind-tag {
        font-family: var(--font-mono);
        font-size: 10px;
        padding: 1px 5px;
        border-radius: 3px;
        text-transform: uppercase;
        margin-left: 6px;
        font-weight: 600;
    }
    .log-kind-tag.checkpoint { background: var(--accent-green-bg); color: var(--accent-green); border: 1px solid rgba(16,185,129,0.3); }
    .log-kind-tag.heartbeat { background: var(--accent-blue-bg); color: var(--accent-blue); border: 1px solid rgba(59,130,246,0.3); }

    /* Node Network Grid Banner */
    .nodes-grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
        gap: 12px;
        margin-bottom: 20px;
    }

    .node-card {
        background: var(--bg-card);
        border: 1px solid var(--border-color);
        border-radius: var(--radius-sm);
        padding: 12px 14px;
        display: flex;
        flex-direction: column;
        gap: 4px;
    }

    .node-card-header {
        display: flex;
        justify-content: space-between;
        align-items: center;
    }

    .node-card-id {
        font-weight: 700;
        font-size: 13px;
    }

    .node-card-sub {
        font-size: 11px;
        color: var(--text-secondary);
        font-family: var(--font-mono);
    }
"""

DEBUG_DRAWER_HTML = """
    <button class="btn btn-sm debug-toggle-btn" onclick="document.getElementById('debugDrawer').classList.toggle('open')">
        Telemetry Debug
    </button>

    <div class="debug-drawer" id="debugDrawer">
        <div style="padding: 16px; border-bottom: 1px solid var(--border-color); display: flex; justify-content: space-between; align-items: center;">
            <h3 style="font-size: 14px; font-weight: 600;">Hardware Telemetry</h3>
            <button class="btn btn-sm" onclick="document.getElementById('debugDrawer').classList.remove('open')">Close</button>
        </div>

        <div style="padding: 12px 16px; border-bottom: 1px solid var(--border-color); display: flex; gap: 8px;">
            <select id="debugNodeFilter" onchange="debugTick()" class="form-control" style="flex:1; font-size:12px;">
                <option value="__all__">All Checkpoint Nodes</option>
            </select>
            <select id="debugSortMode" onchange="debugTick()" class="form-control" style="flex:1; font-size:12px;">
                <option value="proximity">Sort: Closest First</option>
                <option value="node_id">Sort: Node ID</option>
            </select>
        </div>

        <div style="flex: 1; overflow-y: auto; padding: 16px; display: flex; flex-direction: column; gap: 18px;">
            <!-- Drones Section -->
            <div>
                <div class="debug-sec-header" id="debugDronesHeader">Drones</div>
                <div id="debugDronesList" style="display: flex; flex-direction: column; gap: 6px;"></div>
            </div>

            <!-- Nodes Section -->
            <div>
                <div class="debug-sec-header">Checkpoint Nodes</div>
                <div id="debugNodesList" style="display: flex; flex-direction: column; gap: 10px;"></div>
            </div>

            <!-- Raw Log Section -->
            <div>
                <div class="debug-sec-header">Raw Request Log</div>
                <div id="debugLogList" style="font-family: var(--font-mono); font-size: 11px; display: flex; flex-direction: column; gap: 8px;"></div>
            </div>
        </div>
    </div>

    <script>
        let debugTickInFlight = false;

        function formatAgeSec(sec) {
            if (sec < 1) return Math.round(sec * 1000) + 'ms ago';
            return sec.toFixed(1) + 's ago';
        }

        async function debugTick() {
            if (debugTickInFlight) return;
            debugTickInFlight = true;

            try {
                const res = await fetch('/api/debug');
                const data = await res.json();

                // 1. Drones Section
                const drones = data.drones || [];
                const onlineDrones = drones.filter(d => d.online).length;
                document.getElementById('debugDronesHeader').textContent = `Drones · ${onlineDrones}/${drones.length} Online`;

                const dronesListEl = document.getElementById('debugDronesList');
                if (drones.length === 0) {
                    dronesListEl.innerHTML = `<div style="color: var(--text-muted); font-size: 12px;">No drone beacons heard yet.</div>`;
                } else {
                    dronesListEl.innerHTML = drones.map(d => `
                        <div style="display: flex; justify-content: space-between; align-items: center; font-size: 12px; background: var(--bg-input); padding: 6px 10px; border-radius: 4px;">
                            <div>
                                <span class="debug-dot ${d.online ? 'online' : 'offline'}"></span>
                                <strong style="color: ${d.color}">Drone ${d.id}</strong>
                            </div>
                            <span style="color: var(--text-secondary); font-size: 11px;">
                                ${d.online ? `@ ${d.closest_node} &middot; ${d.rssi} dBm` : 'offline'}
                            </span>
                        </div>
                    `).join('');
                }

                // 2. Node Filter Options Update
                const filterEl = document.getElementById('debugNodeFilter');
                const selectedFilter = filterEl.value || '__all__';
                const knownNodeIds = (data.nodes || []).map(n => n.node_id);
                const currentOptIds = Array.from(filterEl.options).map(o => o.value);

                if (currentOptIds.length !== knownNodeIds.length + 1) {
                    filterEl.innerHTML = '<option value="__all__">All Checkpoint Nodes</option>' +
                        knownNodeIds.map(id => `<option value="${id}">${id}</option>`).join('');
                    filterEl.value = currentOptIds.includes(selectedFilter) ? selectedFilter : '__all__';
                }

                const activeFilter = filterEl.value || '__all__';
                const sortMode = document.getElementById('debugSortMode').value || 'proximity';

                // 3. Visible Nodes & Sorting
                let visibleNodes = activeFilter === '__all__'
                    ? (data.nodes || []).slice()
                    : (data.nodes || []).filter(n => n.node_id === activeFilter);

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

                const nodesListEl = document.getElementById('debugNodesList');
                if (visibleNodes.length === 0) {
                    nodesListEl.innerHTML = `<div style="color: var(--text-muted); font-size: 12px;">No nodes connected.</div>`;
                } else {
                    nodesListEl.innerHTML = visibleNodes.map(n => {
                        const heard = n.drones_heard || [];
                        const dronesInfo = heard.length > 0 ? heard.map(h =>
                            `Drone ${h.id} &middot; ${h.rssi} dBm${h.is_closest ? ' <span class="closest-tag">CLOSEST</span>' : ''}`
                        ).join('<br>') : '';

                        const fwBadge = n.fw_version
                            ? `<span class="fw-tag">fw ${n.fw_version}</span>`
                            : `<span class="fw-tag" style="color: var(--accent-red)">fw ?</span>`;

                        const dropInfo = n.disconnect_count ? `
                            <div style="color: var(--accent-amber); font-size: 11px; margin-top: 2px;">
                                Dropped ${n.disconnect_count}x ${n.last_disc_label ? '&middot; last: ' + n.last_disc_label : ''}
                            </div>
                        ` : '';

                        return `
                            <div style="background: var(--bg-input); padding: 8px 10px; border-radius: 4px; font-size: 12px;">
                                <div style="display: flex; justify-content: space-between; align-items: center;">
                                    <div>
                                        <span class="debug-dot ${n.online ? 'online' : 'offline'}"></span>
                                        <strong>${n.node_id}</strong>
                                        ${fwBadge}
                                    </div>
                                    <span style="color: var(--text-secondary); font-size: 11px;">${formatAgeSec(n.age)} &middot; ${n.ip}</span>
                                </div>
                                ${dronesInfo ? `<div style="color: var(--accent-green); font-size: 11px; margin-top: 4px;">${dronesInfo}</div>` : ''}
                                ${dropInfo}
                            </div>
                        `;
                    }).join('');
                }

                // 4. Raw Log Stream
                const visibleLogs = activeFilter === '__all__'
                    ? (data.log || [])
                    : (data.log || []).filter(l => l.node_id === activeFilter);

                const logListEl = document.getElementById('debugLogList');
                if (visibleLogs.length === 0) {
                    logListEl.innerHTML = `<div style="color: var(--text-muted); font-size: 12px;">No request logs recorded.</div>`;
                } else {
                    logListEl.innerHTML = visibleLogs.slice(0, 40).map(l => {
                        let detailStr = '';
                        if (l.kind === 'checkpoint') {
                            detailStr = `drone ${l.detail.drone_id} &middot; ${l.detail.rssi} dBm`;
                        } else if (l.kind === 'heartbeat') {
                            detailStr = l.detail.wifi_rssi !== undefined && l.detail.wifi_rssi !== null
                                ? `wifi ${l.detail.wifi_rssi} dBm` : '';
                            if (l.detail.drone_rssi !== undefined && l.detail.drone_rssi !== null) {
                                detailStr += ` &middot; drone ${l.detail.drone_rssi} dBm`;
                            }
                        }
                        return `
                            <div style="border-bottom: 1px solid var(--border-color); padding-bottom: 4px;">
                                <span style="color: var(--text-muted);">${l.time_str}</span>
                                <span class="log-kind-tag ${l.kind}">${l.kind}</span><br>
                                <strong style="color: var(--text-primary);">${l.node_id}</strong>
                                <span style="color: var(--text-secondary);">${detailStr}</span>
                            </div>
                        `;
                    }).join('');
                }
            } catch (e) {
            } finally {
                debugTickInFlight = false;
            }
        }

        setInterval(debugTick, 250);
        debugTick();
    </script>
"""

# -----------------------------------------------------------------------------
# Main Race Control Page
# -----------------------------------------------------------------------------

RACE_CONTROL_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>kwad — Race Control</title>
    <style>
    """ + BASE_STYLE + """
        .race-header {
            display: flex;
            align-items: center;
            justify-content: space-between;
            gap: 20px;
            flex-wrap: wrap;
        }

        .race-clock-display {
            font-family: var(--font-mono);
            font-size: 42px;
            font-weight: 700;
            color: var(--text-primary);
            letter-spacing: -1px;
            line-height: 1;
        }

        .controls-group {
            display: flex;
            gap: 10px;
            align-items: center;
        }

        .config-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
            gap: 16px;
            margin-top: 16px;
            padding-top: 16px;
            border-top: 1px solid var(--border-color);
        }

        .gate-sequence-pills {
            display: flex;
            align-items: center;
            gap: 8px;
            flex-wrap: wrap;
            margin-top: 8px;
        }

        .gate-pill {
            display: inline-flex;
            align-items: center;
            gap: 6px;
            background: var(--bg-input);
            border: 1px solid var(--border-color);
            padding: 4px 10px;
            border-radius: var(--radius-sm);
            font-family: var(--font-mono);
            font-size: 12px;
        }

        .gate-arrow {
            color: var(--text-muted);
            font-size: 12px;
        }

        .log-stream {
            max-height: 220px;
            overflow-y: auto;
            font-family: var(--font-mono);
            font-size: 12px;
        }

        .log-row {
            padding: 6px 0;
            border-bottom: 1px solid var(--border-color);
            display: flex;
            gap: 12px;
            align-items: baseline;
        }

        .log-time { color: var(--text-muted); min-width: 80px; }
        .log-valid { color: var(--accent-green); }
        .log-invalid { color: var(--accent-red); font-weight: 600; }

        .tab-bar {
            display: flex;
            gap: 12px;
            border-bottom: 1px solid var(--border-color);
            margin-bottom: 16px;
        }

        .tab-btn {
            background: none;
            border: none;
            color: var(--text-secondary);
            font-size: 13px;
            font-weight: 600;
            padding: 8px 12px;
            cursor: pointer;
            border-bottom: 2px solid transparent;
        }

        .tab-btn.active {
            color: var(--accent-green);
            border-bottom-color: var(--accent-green);
        }
    </style>
</head>
<body>
    <div class="container">
        {{ topbar_html|safe }}

        <!-- Live Node Network Grid -->
        <div class="nodes-grid" id="nodesBannerGrid">
            <div style="color: var(--text-muted); font-size: 12px; grid-column: 1 / -1;">
                Scanning for active checkpoint nodes...
            </div>
        </div>

        <!-- Race Control Banner -->
        <div class="card">
            <div class="race-header">
                <div>
                    <div style="display: flex; align-items: center; gap: 12px; margin-bottom: 6px;">
                        <span id="raceStatusBadge" class="badge badge-stopped">
                            <span class="badge-dot"></span><span id="raceStatusText">STOPPED</span>
                        </span>
                        <span style="font-size: 13px; color: var(--text-secondary);" id="raceModeLabel">Practice / Free Fly</span>
                    </div>
                    <div class="race-clock-display" id="raceClock">00:00.0</div>
                </div>

                <div class="controls-group">
                    <button class="btn btn-success" id="startBtn" onclick="controlRace('start')">
                        Start Race
                    </button>
                    <button class="btn btn-danger" id="stopBtn" onclick="controlRace('stop')">
                        Stop
                    </button>
                    <button class="btn" id="resetBtn" onclick="controlRace('reset')">
                        Reset Data
                    </button>
                </div>
            </div>

            <!-- Race Rules & Gate Config -->
            <div class="config-grid">
                <div class="form-group">
                    <label class="form-label">Target Laps</label>
                    <input type="number" id="targetLapsInput" class="form-control" min="1" max="100" value="3">
                </div>

                <div class="form-group">
                    <label class="form-label">Gate Order Enforcement</label>
                    <select id="enforceGatesSelect" class="form-control">
                        <option value="true">ENFORCED (Drones must hit gates in order)</option>
                        <option value="false">DISABLED (Any gate pass counts)</option>
                    </select>
                </div>

                <div class="form-group" style="grid-column: 1 / -1;">
                    <label class="form-label">Gate Sequence Order (Comma separated Node IDs)</label>
                    <div style="display: flex; gap: 10px;">
                        <input type="text" id="gateOrderInput" class="form-control" style="flex: 1;" placeholder="checkpoint-1, checkpoint-2, checkpoint-3">
                        <button class="btn btn-sm" onclick="saveRaceConfig()">Update Config</button>
                    </div>
                    <div class="gate-sequence-pills" id="gateSequencePreview"></div>
                </div>
            </div>
        </div>

        <!-- Standings / Leaderboard -->
        <div class="card">
            <div class="tab-bar">
                <button class="tab-btn active" onclick="switchMainTab('standings', this)">Live Standings</button>
                <button class="tab-btn" onclick="switchMainTab('rawEvents', this)">Raw Checkpoint Log (<span id="totalPassesCount">0</span>)</button>
            </div>

            <div id="tabStandingsView">
                <div class="table-responsive">
                    <table>
                        <thead>
                            <tr>
                                <th>Pos</th>
                                <th>Drone</th>
                                <th>Status</th>
                                <th>Laps</th>
                                <th>Next Required Gate</th>
                                <th>Last Lap</th>
                                <th>Best Lap</th>
                                <th>Total Time</th>
                            </tr>
                        </thead>
                        <tbody id="standingsBody">
                            <tr><td colspan="8" style="color: var(--text-muted); text-align: center; padding: 24px;">No active drones detected. Power on a drone or fly through a gate.</td></tr>
                        </tbody>
                    </table>
                </div>
            </div>

            <div id="tabRawEventsView" style="display: none;">
                <div class="table-responsive">
                    <table>
                        <thead>
                            <tr>
                                <th>#</th>
                                <th>Node ID</th>
                                <th>Drone ID</th>
                                <th>RSSI</th>
                                <th>Node Timestamp</th>
                                <th>Received Time</th>
                            </tr>
                        </thead>
                        <tbody id="rawEventsBody">
                            <tr><td colspan="6" style="color: var(--text-muted); text-align: center; padding: 24px;">No pass events recorded yet.</td></tr>
                        </tbody>
                    </table>
                </div>
            </div>
        </div>

        <!-- Live Gate Pass Stream -->
        <div class="card">
            <div class="card-title">Live Gate Feed</div>
            <div class="card-subtitle">Real-time gate pass verification, practice hits, and skipped gate alerts</div>

            <div class="log-stream" id="logStream">
                <div style="color: var(--text-muted); padding: 12px 0;">Waiting for gate passes...</div>
            </div>
        </div>
    </div>

    """ + DEBUG_DRAWER_HTML + """

    <script>
        function switchMainTab(tabName, btnEl) {
            document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
            btnEl.classList.add('active');

            if (tabName === 'standings') {
                document.getElementById('tabStandingsView').style.display = 'block';
                document.getElementById('tabRawEventsView').style.display = 'none';
            } else {
                document.getElementById('tabStandingsView').style.display = 'none';
                document.getElementById('tabRawEventsView').style.display = 'block';
            }
        }

        async function fetchRaceStatus() {
            try {
                const res = await fetch('/api/race/status');
                const data = await res.json();
                renderRaceUI(data);
            } catch (e) {
                console.error("Failed to fetch race status", e);
            }
        }

        function renderRaceUI(data) {
            const state = data.race;
            
            // Status Badge & Mode Label
            const badge = document.getElementById('raceStatusBadge');
            const statusText = document.getElementById('raceStatusText');
            const modeLabel = document.getElementById('raceModeLabel');
            statusText.textContent = state.status;
            
            if (state.status === 'RUNNING') {
                badge.className = 'badge badge-running';
                modeLabel.textContent = "Timed Race Active";
            } else if (state.status === 'FINISHED') {
                badge.className = 'badge badge-finished';
                modeLabel.textContent = "Race Finished";
            } else {
                badge.className = 'badge badge-stopped';
                modeLabel.textContent = "Practice / Free Fly";
            }

            // Race Clock
            if (state.status === 'RUNNING' && state.start_time) {
                const elapsed = (Date.now() / 1000) - state.start_time;
                document.getElementById('raceClock').textContent = formatSeconds(elapsed);
            } else if (state.status === 'FINISHED' && state.start_time && state.end_time) {
                document.getElementById('raceClock').textContent = formatSeconds(state.end_time - state.start_time);
            } else if (state.status === 'STOPPED') {
                document.getElementById('raceClock').textContent = "00:00.0";
            }

            // Config inputs
            if (document.activeElement !== document.getElementById('targetLapsInput')) {
                document.getElementById('targetLapsInput').value = state.target_laps;
            }
            if (document.activeElement !== document.getElementById('enforceGatesSelect')) {
                document.getElementById('enforceGatesSelect').value = state.enforce_gate_order ? 'true' : 'false';
            }
            if (document.activeElement !== document.getElementById('gateOrderInput')) {
                document.getElementById('gateOrderInput').value = state.gate_order.join(', ');
            }

            // Gate sequence preview
            const previewEl = document.getElementById('gateSequencePreview');
            previewEl.innerHTML = state.gate_order.map((g, i) => `
                <span class="gate-pill">Gate ${i+1}: ${g}</span>
                ${i < state.gate_order.length - 1 ? '<span class="gate-arrow">&rarr;</span>' : ''}
            `).join('');

            // Connected Checkpoints Grid Banner
            const nodesGridEl = document.getElementById('nodesBannerGrid');
            const nodes = data.nodes || [];
            if (nodes.length === 0) {
                nodesGridEl.innerHTML = `<div style="color: var(--text-muted); font-size: 12px; grid-column: 1 / -1;">No checkpoint nodes connected yet. Power on an ESP32 node.</div>`;
            } else {
                nodesGridEl.innerHTML = nodes.map(n => `
                    <div class="node-card">
                        <div class="node-card-header">
                            <span class="node-card-id">${n.node_id}</span>
                            <span class="debug-dot ${n.online ? 'online' : 'offline'}"></span>
                        </div>
                        <div class="node-card-sub">
                            ${n.online ? 'ONLINE · ' + n.ip : 'OFFLINE'}
                        </div>
                        ${n.wifi_rssi ? `<div style="font-size: 10px; color: var(--text-muted);">WiFi: ${n.wifi_rssi} dBm</div>` : ''}
                    </div>
                `).join('');
            }

            // Standings table
            const standingsBody = document.getElementById('standingsBody');
            const standings = data.standings || [];

            if (standings.length === 0) {
                standingsBody.innerHTML = `<tr><td colspan="8" style="color: var(--text-muted); text-align: center; padding: 24px;">No active drones detected yet. Fly through a gate or power on a drone.</td></tr>`;
            } else {
                standingsBody.innerHTML = standings.map((s, idx) => {
                    const lastLap = s.laps.length > 0 ? s.laps[s.laps.length - 1].lap_time.toFixed(2) + 's' : '—';
                    const bestLap = s.best_lap ? s.best_lap.toFixed(2) + 's' : '—';
                    const totalTime = s.total_time ? s.total_time.toFixed(2) + 's' : '—';
                    const nextGate = state.gate_order[s.next_gate_index] || '—';

                    return `
                        <tr>
                            <td class="font-mono" style="font-weight:700;">#${idx + 1}</td>
                            <td>
                                <strong style="color: ${s.color};">Drone ${s.drone_id}</strong>
                            </td>
                            <td>
                                <span class="badge ${s.status === 'FINISHED' ? 'badge-finished' : (s.status === 'RACING' ? 'badge-running' : 'badge-stopped')}">
                                    ${s.status}
                                </span>
                            </td>
                            <td class="font-mono"><strong>${s.completed_laps}</strong> / ${state.target_laps}</td>
                            <td class="font-mono" style="color: var(--accent-blue);">${s.status === 'FINISHED' ? '—' : nextGate}</td>
                            <td class="font-mono">${lastLap}</td>
                            <td class="font-mono" style="color: var(--accent-green);">${bestLap}</td>
                            <td class="font-mono">${totalTime}</td>
                        </tr>
                    `;
                }).join('');
            }

            // Total passes count
            document.getElementById('totalPassesCount').textContent = data.total_passes || 0;

            // Raw Events Table
            const rawBody = document.getElementById('rawEventsBody');
            const rawEvents = data.events || [];
            if (rawEvents.length === 0) {
                rawBody.innerHTML = `<tr><td colspan="6" style="color: var(--text-muted); text-align: center; padding: 24px;">No pass events recorded yet.</td></tr>`;
            } else {
                rawBody.innerHTML = rawEvents.slice(0, 30).map((e, idx) => `
                    <tr>
                        <td class="font-mono" style="color: var(--text-muted);">${String(idx + 1).padStart(2, '0')}</td>
                        <td><strong style="color: var(--text-primary);">${e.node_id}</strong></td>
                        <td style="color: var(--accent-green);">Drone ${e.drone_id}</td>
                        <td class="font-mono">${e.rssi} dBm</td>
                        <td class="font-mono">${e.timestamp}</td>
                        <td class="font-mono" style="color: var(--text-secondary);">${e.received_at}</td>
                    </tr>
                `).join('');
            }

            // Live Feed Log Stream
            const logEl = document.getElementById('logStream');
            const logs = data.logs || [];
            if (logs.length === 0) {
                logEl.innerHTML = `<div style="color: var(--text-muted); padding: 12px 0;">Waiting for gate passes...</div>`;
            } else {
                logEl.innerHTML = logs.map(l => `
                    <div class="log-row">
                        <span class="log-time">${l.time}</span>
                        <span class="${l.valid ? 'log-valid' : 'log-invalid'}">${l.message}</span>
                    </div>
                `).join('');
            }
        }

        function formatSeconds(sec) {
            if (!sec || sec < 0) return "00:00.0";
            const m = Math.floor(sec / 60);
            const s = (sec % 60).toFixed(1);
            return `${String(m).padStart(2, '0')}:${String(s).padStart(4, '0')}`;
        }

        async function controlRace(action) {
            await fetch(`/api/race/${action}`, { method: 'POST' });
            fetchRaceStatus();
        }

        async function saveRaceConfig() {
            const targetLaps = parseInt(document.getElementById('targetLapsInput').value, 10);
            const enforce = document.getElementById('enforceGatesSelect').value === 'true';
            const rawOrder = document.getElementById('gateOrderInput').value;
            const gateOrder = rawOrder.split(',').map(s => s.trim()).filter(Boolean);

            await fetch('/api/race/config', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    target_laps: targetLaps,
                    enforce_gate_order: enforce,
                    gate_order: gateOrder
                })
            });
            fetchRaceStatus();
        }

        setInterval(fetchRaceStatus, 500);
        fetchRaceStatus();
    </script>
</body>
</html>
"""

# -----------------------------------------------------------------------------
# Radar & Settings Pages
# -----------------------------------------------------------------------------

RADAR_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>kwad — Radar</title>
    <style>
    """ + BASE_STYLE + """
        .radar-box {
            position: relative;
            width: 100%;
            aspect-ratio: 1 / 1;
            background: var(--bg-input);
            border: 1px solid var(--border-color);
            border-radius: var(--radius-md);
            overflow: hidden;
        }

        .node-marker {
            position: absolute;
            width: 16px;
            height: 16px;
            transform: translate(-50%, -50%);
            border: 2px solid var(--accent-blue);
            background: var(--bg-card);
            border-radius: 4px;
        }

        .node-label {
            position: absolute;
            transform: translate(-50%, 12px);
            top: 100%;
            font-size: 11px;
            font-family: var(--font-mono);
            color: var(--text-secondary);
            white-space: nowrap;
        }

        .drone-marker {
            position: absolute;
            width: 14px;
            height: 14px;
            border-radius: 50%;
            transform: translate(-50%, -50%);
            transition: left 0.4s ease, top 0.4s ease;
        }

        .drone-label {
            position: absolute;
            left: 18px;
            top: -4px;
            font-size: 11px;
            font-family: var(--font-mono);
            font-weight: 700;
            white-space: nowrap;
        }
    </style>
</head>
<body>
    <div class="container">
        {{ topbar_html|safe }}
        
        <div class="card">
            <div class="card-title">Track Proximity Radar</div>
            <div class="card-subtitle">Visual positioning based on latest gate proximity reports</div>
            <div class="radar-box" id="radarBox"></div>
        </div>
    </div>

    """ + DEBUG_DRAWER_HTML + """

    <script>
        async function updateRadar() {
            try {
                const res = await fetch('/api/radar');
                const data = await res.json();
                const box = document.getElementById('radarBox');
                box.innerHTML = '';

                data.nodes.forEach(n => {
                    const el = document.createElement('div');
                    el.className = 'node-marker';
                    el.style.left = n.x + '%';
                    el.style.top = n.y + '%';

                    const lbl = document.createElement('div');
                    lbl.className = 'node-label';
                    lbl.textContent = n.id;
                    el.appendChild(lbl);

                    box.appendChild(el);
                });

                data.drones.forEach(d => {
                    const el = document.createElement('div');
                    el.className = 'drone-marker';
                    el.style.left = d.x + '%';
                    el.style.top = d.y + '%';
                    el.style.backgroundColor = d.color;

                    const lbl = document.createElement('div');
                    lbl.className = 'drone-label';
                    lbl.style.color = d.color;
                    lbl.textContent = 'DRONE ' + d.id;
                    el.appendChild(lbl);

                    box.appendChild(el);
                });
            } catch (e) {}
        }
        setInterval(updateRadar, 400);
        updateRadar();
    </script>
</body>
</html>
"""

SETTINGS_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>kwad — Gate Settings</title>
    <style>
    """ + BASE_STYLE + """
        .settings-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
            gap: 16px;
            margin-top: 16px;
        }
    </style>
</head>
<body>
    <div class="container">
        {{ topbar_html|safe }}

        <div class="card">
            <div class="card-title">Checkpoint Hardware Tuning</div>
            <div class="card-subtitle">Configure gate-timing and RSSI thresholds per node</div>

            <div style="display: flex; gap: 12px; margin-bottom: 20px;">
                <select id="nodeSelect" class="form-control" style="flex:1;" onchange="loadNodeSettings()"></select>
                <button class="btn btn-danger btn-sm" type="button" onclick="resetNodeSettings()">Reset Defaults</button>
            </div>

            <form id="settingsForm">
                <div class="settings-grid" id="fieldsGrid"></div>
                <div style="margin-top: 20px; display: flex; align-items: center; gap: 12px;">
                    <button class="btn btn-success" type="submit">Save Hardware Settings</button>
                    <span id="saveStatus" style="font-size: 13px;"></span>
                </div>
            </form>
        </div>
    </div>

    """ + DEBUG_DRAWER_HTML + """

    <script>
        const FIELDS = [
            { key: 'enter_rssi', label: 'Enter RSSI (dBm)' },
            { key: 'exit_rssi', label: 'Exit RSSI (dBm)' },
            { key: 'required_weak_samples', label: 'Required Weak Samples' },
            { key: 'pass_timeout_ms', label: 'Pass Timeout (ms)' },
            { key: 'event_cooldown_ms', label: 'Event Cooldown (ms)' },
            { key: 'heartbeat_interval_ms', label: 'Heartbeat Interval (ms)' }
        ];

        async function initSettings() {
            const res = await fetch('/api/settings');
            const data = await res.json();
            const select = document.getElementById('nodeSelect');
            const nodes = Object.keys(data.nodes).sort();
            select.innerHTML = nodes.map(n => `<option value="${n}">${n}</option>`).join('');
            loadNodeSettings();
        }

        async function loadNodeSettings() {
            const nodeId = document.getElementById('nodeSelect').value;
            if (!nodeId) return;

            const res = await fetch('/api/settings/' + encodeURIComponent(nodeId));
            const settings = await res.json();

            const grid = document.getElementById('fieldsGrid');
            grid.innerHTML = FIELDS.map(f => `
                <div class="form-group">
                    <label class="form-label">${f.label}</label>
                    <input type="number" id="f_${f.key}" class="form-control" value="${settings[f.key]}">
                </div>
            `).join('');
        }

        document.getElementById('settingsForm').addEventListener('submit', async (e) => {
            e.preventDefault();
            const nodeId = document.getElementById('nodeSelect').value;
            const payload = {};
            FIELDS.forEach(f => {
                payload[f.key] = parseInt(document.getElementById('f_' + f.key).value, 10);
            });

            const res = await fetch('/api/settings/' + encodeURIComponent(nodeId), {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(payload)
            });

            const status = document.getElementById('saveStatus');
            if (res.ok) {
                status.textContent = "Saved settings to " + nodeId;
                status.style.color = "var(--accent-green)";
            } else {
                status.textContent = "Save failed";
                status.style.color = "var(--accent-red)";
            }
        });

        async function resetNodeSettings() {
            const nodeId = document.getElementById('nodeSelect').value;
            await fetch('/api/settings/' + encodeURIComponent(nodeId), { method: 'DELETE' });
            loadNodeSettings();
        }

        initSettings();
    </script>
</body>
</html>
"""

# -----------------------------------------------------------------------------
# Flask Web Routes & API Endpoints
# -----------------------------------------------------------------------------

@app.route("/")
def index():
    return render_template_string(RACE_CONTROL_PAGE, topbar_html=topbar("Race Control"))

@app.route("/radar")
def radar_page():
    return render_template_string(RADAR_PAGE, topbar_html=topbar("Radar"))

@app.route("/settings")
def settings_page_route():
    return render_template_string(SETTINGS_PAGE, topbar_html=topbar("Gate Settings"))

# -----------------------------------------------------------------------------
# ESP32 Checkpoint & Heartbeat Telemetry Ingestion
# -----------------------------------------------------------------------------

@app.route("/checkpoint", methods=["POST"])
def checkpoint():
    data = request.get_json(force=True, silent=True)
    if not data or "node_id" not in data:
        return jsonify({"error": "expected JSON with node_id"}), 400

    node_id = data.get("node_id")
    raw_drone_id = data.get("drone_id")
    drone_id = str(raw_drone_id) if raw_drone_id is not None else "1"

    try:
        rssi = int(data.get("rssi"))
    except (TypeError, ValueError):
        rssi = None

    time_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]

    event = {
        "node_id": node_id,
        "drone_id": drone_id,
        "rssi": rssi if rssi is not None else "n/a",
        "timestamp": data.get("timestamp", "n/a"),
        "received_at": time_str,
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

    race_res = process_race_checkpoint(node_id, drone_id, rssi, time_str)

    print(f"[checkpoint] Node: {node_id} | Drone: {drone_id} | RSSI: {event['rssi']} | Event: {race_res}")
    return jsonify({"status": "ok", "event": event, "race": race_res}), 200

@app.route("/api/heartbeat", methods=["POST"])
def heartbeat():
    data = request.get_json(force=True, silent=True) or {}
    node_id = data.get("node_id")
    if not node_id:
        return jsonify({"error": "expected JSON with a node_id field"}), 400

    wifi_rssi = data.get("wifi_rssi")
    extra = {"wifi_rssi": wifi_rssi, "fw_version": data.get("fw_version")}

    if "disconnect_count" in data:
        extra["disconnect_count"] = data.get("disconnect_count")
        extra["last_disc_reason"] = data.get("last_disc_reason")

    if "drone_rssi" in data:
        raw_d_id = data.get("drone_id")
        d_id = str(raw_d_id) if raw_d_id is not None else "1"
        extra["drone_id"] = d_id
        extra["drone_rssi"] = data.get("drone_rssi")
        extra["drone_age_ms"] = data.get("drone_age_ms")
        remember_drone(d_id)

    drone_array = data.get("drones")
    if isinstance(drone_array, list):
        extra["drones"] = drone_array
        for entry in drone_array:
            if isinstance(entry, dict):
                r_id = entry.get("id")
                if r_id is not None:
                    remember_drone(str(r_id))
                    record_drone_sighting(node_id, str(r_id), entry.get("rssi"),
                                          entry.get("age_ms"), from_array=True)
    elif "drone_rssi" in data:
        raw_d_id = data.get("drone_id")
        d_id = str(raw_d_id) if raw_d_id is not None else "1"
        record_drone_sighting(node_id, d_id, data.get("drone_rssi"),
                              data.get("drone_age_ms"), from_array=False)

    record_contact(node_id, "heartbeat", **extra)

    print(f"[heartbeat] Node: {node_id} | from {request.remote_addr} | wifi {wifi_rssi} dBm | fw {data.get('fw_version')}")
    return jsonify({"status": "ok", **get_effective_settings(node_id)}), 200

@app.route("/api/leaderboard")
def api_leaderboard():
    with events_lock:
        recent = list(reversed(events))
    return jsonify({"events": recent, "count": len(events)})

# -----------------------------------------------------------------------------
# Race Control REST APIs
# -----------------------------------------------------------------------------

@app.route("/api/race/status", methods=["GET"])
def api_race_status():
    now = time.time()
    with race_lock:
        # Populate connected nodes list for main view
        nodes_list = []
        with node_registry_lock:
            for n_id, n_state in sorted(node_registry.items()):
                age = now - n_state["last_seen"]
                nodes_list.append({
                    "node_id": n_id,
                    "online": age <= NODE_ONLINE_TIMEOUT_S,
                    "ip": n_state.get("ip", "unknown"),
                    "wifi_rssi": n_state.get("wifi_rssi")
                })

        # Ensure all known/seen drones exist in standings
        with known_drone_lock:
            for k_drone in known_drone_ids:
                init_drone_for_race(k_drone)

        standings = list(drone_race_data.values())

        def standings_sort_key(d):
            is_finished = d["status"] == "FINISHED"
            fin_time = d["total_time"] if d["total_time"] is not None else 999999
            laps = d["completed_laps"]
            best = d["best_lap"] if d["best_lap"] is not None else 999999
            return (0 if is_finished else 1, fin_time if is_finished else -laps, best)

        sorted_standings = sorted(standings, key=standings_sort_key)

        for s in sorted_standings:
            s["color"] = get_drone_color(s["drone_id"])

        logs = list(race_log)

        with events_lock:
            recent_events = list(reversed(events[:50]))
            total_passes = len(events)

        return jsonify({
            "race": race_state,
            "standings": sorted_standings,
            "logs": logs,
            "nodes": nodes_list,
            "events": recent_events,
            "total_passes": total_passes
        })

@app.route("/api/race/start", methods=["POST"])
def api_race_start():
    with race_lock:
        race_state["status"] = "RUNNING"
        race_state["start_time"] = time.time()
        race_state["end_time"] = None
        
        for d_id, d_data in drone_race_data.items():
            d_data["status"] = "RACING"
            d_data["completed_laps"] = 0
            d_data["next_gate_index"] = 0
            d_data["last_gate"] = None
            d_data["last_pass_time"] = None
            d_data["lap_start_time"] = race_state["start_time"]
            d_data["laps"] = []
            d_data["best_lap"] = None
            d_data["total_time"] = None
            d_data["invalid_passes"] = 0

        race_log.appendleft({
            "time": datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "ts": time.time(),
            "type": "RACE_CONTROL",
            "message": "TIMED RACE STARTED!",
            "valid": True
        })

    return jsonify({"status": "ok", "race": race_state})

@app.route("/api/race/stop", methods=["POST"])
def api_race_stop():
    with race_lock:
        race_state["status"] = "STOPPED"
        race_state["end_time"] = time.time()

        race_log.appendleft({
            "time": datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "ts": time.time(),
            "type": "RACE_CONTROL",
            "message": "RACE STOPPED (Practice Mode Active)",
            "valid": False
        })

    return jsonify({"status": "ok", "race": race_state})

@app.route("/api/race/reset", methods=["POST"])
def api_race_reset():
    with race_lock:
        race_state["status"] = "STOPPED"
        race_state["start_time"] = None
        race_state["end_time"] = None
        drone_race_data.clear()
        race_log.clear()

    return jsonify({"status": "ok", "race": race_state})

@app.route("/api/race/config", methods=["POST"])
def api_race_config():
    data = request.get_json(force=True, silent=True) or {}
    with race_lock:
        if "target_laps" in data:
            try:
                race_state["target_laps"] = max(1, int(data["target_laps"]))
            except (TypeError, ValueError):
                pass
        if "enforce_gate_order" in data:
            race_state["enforce_gate_order"] = bool(data["enforce_gate_order"])
        if "gate_order" in data and isinstance(data["gate_order"], list):
            cleaned_gates = [str(g).strip() for g in data["gate_order"] if str(g).strip()]
            if cleaned_gates:
                race_state["gate_order"] = cleaned_gates

    return jsonify({"status": "ok", "race": race_state})

# -----------------------------------------------------------------------------
# Radar & Telemetry Debug APIs
# -----------------------------------------------------------------------------

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
            drones.append({
                "id": drone_id,
                "x": round(x, 1),
                "y": round(y, 1),
                "color": get_drone_color(drone_id),
                "last_node": state["node_id"],
                "rssi": state["rssi"],
                "age": age,
            })

    return jsonify({"nodes": nodes, "drones": drones})

@app.route("/api/debug")
def api_debug():
    now = time.time()

    with drone_sighting_lock:
        sightings = list(drone_sighting_registry.items())

    heard_by_node = defaultdict(list)
    best_by_drone = {}
    for (sighting_node, drone_id), s in sightings:
        sighting_age = s["age_ms"] + (now - s["last_seen"]) * 1000
        limit = DRONE_LIVE_TIMEOUT_MS if s["from_array"] else DRONE_LEGACY_TIMEOUT_MS
        if sighting_age > limit:
            continue

        heard_by_node[sighting_node].append({
            "id": drone_id,
            "rssi": s["rssi"],
            "age_ms": round(sighting_age),
        })

        current = best_by_drone.get(drone_id)
        if current is None or s["rssi"] > current["rssi"]:
            best_by_drone[drone_id] = {
                "closest_node": sighting_node,
                "rssi": s["rssi"],
                "age_ms": round(sighting_age),
            }

    nodes = []
    with node_registry_lock:
        for node_id, state in sorted(node_registry.items()):
            age = now - state["last_seen"]

            drone_age_ms = state.get("drone_age_ms")
            live_drone_age_ms = None
            if drone_age_ms is not None:
                live_drone_age_ms = drone_age_ms + age * 1000

            drones_heard = sorted(heard_by_node.get(node_id, []),
                                  key=lambda d: d["rssi"], reverse=True)
            for entry in drones_heard:
                best = best_by_drone.get(entry["id"])
                entry["is_closest"] = bool(best and best["closest_node"] == node_id)

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
                "drones_heard": drones_heard,
            })

    live_nodes = [n for n in nodes if n["drone_age_ms"] is not None and n["drone_age_ms"] <= DRONE_LIVE_TIMEOUT_MS]
    closest_node = max(live_nodes, key=lambda n: n["drone_rssi"], default=None)

    with raw_log_lock:
        merged = [entry for entries in raw_log_by_node.values() for entry in entries]

    merged.sort(key=lambda e: e["ts"], reverse=True)
    log = merged[:200]

    drone_status = {
        "online": len(live_nodes) > 0,
        "closest_node": closest_node["node_id"] if closest_node else None,
        "closest_rssi": closest_node["drone_rssi"] if closest_node else None,
    }

    with known_drone_lock:
        all_drone_ids = set(known_drone_ids)
    all_drone_ids |= set(best_by_drone.keys())

    def drone_sort_key(value):
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

def get_local_ip():
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Doesn't actually send packets; just picks the outbound interface.
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()

if __name__ == "__main__":
    debug_mode = os.environ.get("KWAD_DEBUG", "1") != "0"

    local_ip = get_local_ip()
    print("=" * 60)
    print(f" kwad race control v{APP_VERSION}")
    print(f" Serving on http://{local_ip}:5000  (and http://127.0.0.1:5000)")
    print(f" Nodes must POST to this IP. Firmware currently targets the")
    print(f" IP set as PI_IP in the .ino files - make sure it matches {local_ip}.")
    print("=" * 60)

    app.run(host="0.0.0.0", port=5000, debug=debug_mode, threaded=True)
