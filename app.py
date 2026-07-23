from flask import Flask, request, jsonify, render_template_string
from datetime import datetime
import threading
 
app = Flask(__name__)
 
# In-memory event log - list of dicts: {node_id, timestamp, received_at}
events = []
events_lock = threading.Lock()
 
LEADERBOARD_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>KWAD // Live Track</title>
    <meta http-equiv="refresh" content="2">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta charset="UTF-8">
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
    <link href="https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@500;700&family=JetBrains+Mono:wght@400;500;700&display=swap" rel="stylesheet">
    <style>
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
    </style>
</head>
<body>
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
                    <div class="flap-row">
                        {% for digit in '%03d'|format(count) %}
                        <span class="flap-digit">{{ digit }}</span>
                        {% endfor %}
                    </div>
                </div>
                <div class="stat">
                    <div class="stat-label">Last node</div>
                    <div class="stat-value">{{ events[0].node_id if events else '\u2014' }}</div>
                </div>
                <div class="stat">
                    <div class="stat-label">Refresh interval</div>
                    <div class="stat-value">2s</div>
                </div>
            </div>
        </div>

        <div class="panel">
            <div class="table-scroll">
                <table>
                    <thead>
                        <tr><th>#</th><th>Node</th><th>Sent</th><th>Received &middot; Pi</th></tr>
                    </thead>
                    <tbody>
                    {% for e in events %}
                        <tr>
                            <td class="idx">{{ '%02d'|format(loop.index) }}</td>
                            <td><span class="node-pill">{{ e.node_id }}</span></td>
                            <td>{{ e.timestamp }}</td>
                            <td>{{ e.received_at }}</td>
                        </tr>
                    {% endfor %}
                    </tbody>
                </table>
            </div>
            {% if not events %}
            <div class="empty">Waiting for checkpoint signal &hellip;</div>
            {% endif %}
        </div>
    </div>
</body>
</html>
"""
 
@app.route("/")
def leaderboard():
    with events_lock:
        # show most recent first
        recent = list(reversed(events))
    return render_template_string(LEADERBOARD_PAGE, events=recent, count=len(events))
 
@app.route("/checkpoint", methods=["POST"])
def checkpoint():
    data = request.get_json(force=True, silent=True)
    if not data or "node_id" not in data:
        return jsonify({"error": "expected JSON with at least a node_id field"}), 400
 
    event = {
        "node_id": data.get("node_id"),
        "timestamp": data.get("timestamp", "n/a"),
        "received_at": datetime.now().strftime("%H:%M:%S.%f")[:-3],
    }
 
    with events_lock:
        events.append(event)
 
    print(f"[checkpoint] {event}")
    return jsonify({"status": "ok", "event": event}), 200
 
@app.route("/health")
def health():
    return jsonify({"status": "alive"}), 200
 
if __name__ == "__main__":
    # 0.0.0.0 so ESP32s on the same WiFi can reach it, not just localhost
    app.run(host="0.0.0.0", port=5000, debug=True)
