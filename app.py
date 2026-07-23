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
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;700&display=swap" rel="stylesheet">
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: 'JetBrains Mono', monospace;
            background: #0a0c0a;
            background-image:
                linear-gradient(rgba(0,255,140,0.03) 1px, transparent 1px),
                linear-gradient(90deg, rgba(0,255,140,0.03) 1px, transparent 1px);
            background-size: 24px 24px;
            color: #d8ffe8;
            padding: 40px 24px;
            min-height: 100vh;
        }
        .wrap { max-width: 920px; margin: 0 auto; }
 
        .hud {
            position: relative;
            border: 1px solid rgba(0,255,140,0.25);
            background: rgba(15,25,20,0.55);
            backdrop-filter: blur(6px);
            padding: 24px 28px;
            margin-bottom: 24px;
        }
        .hud::before, .hud::after,
        .corners .c1, .corners .c2 {
            content: ''; position: absolute; width: 14px; height: 14px;
            border: 2px solid #00ff8c;
        }
        .hud::before { top: -1px; left: -1px; border-right: none; border-bottom: none; }
        .hud::after  { bottom: -1px; right: -1px; border-left: none; border-top: none; }
 
        .title { font-size: 13px; letter-spacing: 4px; color: #7dffb8; text-transform: uppercase; }
        .title span { color: #00ff8c; }
        h1 { font-size: 28px; font-weight: 700; margin-top: 6px; letter-spacing: 1px; }
 
        .stats { display: flex; gap: 20px; margin-top: 18px; flex-wrap: wrap; }
        .stat { border: 1px solid rgba(0,255,140,0.2); padding: 10px 16px; min-width: 130px; background: rgba(0,255,140,0.03); }
        .stat-label { font-size: 10px; letter-spacing: 2px; color: #6ba884; text-transform: uppercase; }
        .stat-value { font-size: 22px; font-weight: 700; color: #00ff8c; margin-top: 2px; }
 
        .live-dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%; background: #00ff8c; margin-right: 8px; box-shadow: 0 0 8px #00ff8c; animation: pulse 1.4s infinite; }
        @keyframes pulse { 0%,100% { opacity: 1; } 50% { opacity: 0.35; } }
 
        table { width: 100%; border-collapse: collapse; }
        thead th {
            text-align: left; font-size: 11px; letter-spacing: 2px; text-transform: uppercase;
            color: #6ba884; padding: 10px 14px; border-bottom: 1px solid rgba(0,255,140,0.25);
        }
        tbody td { padding: 12px 14px; font-size: 14px; border-bottom: 1px solid rgba(0,255,140,0.08); }
        tbody tr:hover { background: rgba(0,255,140,0.05); }
        tbody tr:first-child td { color: #00ff8c; font-weight: 700; }
        .idx { color: #4a7c63; width: 40px; }
        .node-pill {
            display: inline-block; padding: 3px 10px; border: 1px solid rgba(0,255,140,0.3);
            font-size: 12px; letter-spacing: 1px;
        }
        .empty { padding: 40px 14px; text-align: center; color: #4a7c63; letter-spacing: 1px; }
    </style>
</head>
<body>
    <div class="wrap">
        <div class="hud">
            <div class="title">FPV RACE TRACKER // <span>{{ '{:03d}'.format(count) }}</span> EVENTS LOGGED</div>
            <h1><span class="live-dot"></span>LIVE LEADERBOARD</h1>
            <div class="stats">
                <div class="stat">
                    <div class="stat-label">Total passes</div>
                    <div class="stat-value">{{ count }}</div>
                </div>
                <div class="stat">
                    <div class="stat-label">Last node</div>
                    <div class="stat-value">{{ events[0].node_id if events else '--' }}</div>
                </div>
                <div class="stat">
                    <div class="stat-label">Refresh</div>
                    <div class="stat-value">2s</div>
                </div>
            </div>
        </div>
 
        <div class="hud">
            <table>
                <thead>
                    <tr><th>#</th><th>Node</th><th>Sent Timestamp</th><th>Received (Pi)</th></tr>
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
            {% if not events %}
            <div class="empty">WAITING FOR CHECKPOINT SIGNAL...</div>
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
