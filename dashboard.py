from flask import Flask, request, jsonify, render_template_string
from datetime import datetime
import json
import threading
import time
try:
    import serial
except ImportError:
    serial = None

app = Flask(__name__)

# Serial/PySerial mode
USE_PYSERIAL = True
SERIAL_PORT = "COM7"
SERIAL_BAUD = 115200

latest_data = {
    "test_mode": False,
    "uptime_ms": 0,
    "lidar_cm": [0, 0, 0, 0],
    "motor_integrity": [False, False, False, False],
    "camera": {
        "width": 0,
        "height": 0,
        "size_bytes": 0,
        "jpeg_base64": "",
    },
    "timestamp": None,
}

PAGE = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>UAV Telemetry Dashboard</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 20px; background: #111; color: #eee; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
    .card { background: #1d1d1d; border-radius: 10px; padding: 14px; }
    .title { font-weight: bold; margin-bottom: 10px; color: #6cf; }
    .row { margin: 6px 0; }
    img { max-width: 100%; border-radius: 8px; border: 1px solid #333; }
    .muted { color: #aaa; font-size: 0.9rem; }
  </style>
</head>
<body>
  <h2>UAV Telemetry Dashboard</h2>
  <div class="muted">Visible to any device in local network using this server IP and port.</div>
  <div class="muted" id="ts"></div>
  <div class="grid">
    <div class="card">
      <div class="title">System Status</div>
      <div class="row">Test mode: <span id="test_mode">false</span></div>
      <div class="row">Uptime (ms): <span id="uptime_ms">0</span></div>
      <div class="row">Motor 1 integrity: <span id="m1">false</span></div>
      <div class="row">Motor 2 integrity: <span id="m2">false</span></div>
      <div class="row">Motor 3 integrity: <span id="m3">false</span></div>
      <div class="row">Motor 4 integrity: <span id="m4">false</span></div>
    </div>
    <div class="card">
      <div class="title">LiDAR Distances (cm)</div>
      <div class="row">Lidar 1: <span id="l1">0</span></div>
      <div class="row">Lidar 2: <span id="l2">0</span></div>
      <div class="row">Lidar 3: <span id="l3">0</span></div>
      <div class="row">Lidar 4: <span id="l4">0</span></div>
    </div>
    <div class="card" style="grid-column: span 2;">
      <div class="title">Camera (latest JPEG frame)</div>
      <div class="row">Resolution: <span id="cw">0</span>x<span id="ch">0</span></div>
      <div class="row">Size: <span id="cs">0</span> bytes</div>
      <img id="cam" />
    </div>
  </div>

  <script>
    async function refresh() {
      const r = await fetch('/latest');
      const data = await r.json();
      document.getElementById('test_mode').textContent = String(data.test_mode ?? false);
      document.getElementById('uptime_ms').textContent = data.uptime_ms ?? 0;
      const mi = data.motor_integrity || [false, false, false, false];
      document.getElementById('m1').textContent = String(mi[0] ?? false);
      document.getElementById('m2').textContent = String(mi[1] ?? false);
      document.getElementById('m3').textContent = String(mi[2] ?? false);
      document.getElementById('m4').textContent = String(mi[3] ?? false);
      const l = data.lidar_cm || [0,0,0,0];
      document.getElementById('l1').textContent = l[0] ?? 0;
      document.getElementById('l2').textContent = l[1] ?? 0;
      document.getElementById('l3').textContent = l[2] ?? 0;
      document.getElementById('l4').textContent = l[3] ?? 0;

      const cam = data.camera || {};
      document.getElementById('cw').textContent = cam.width ?? 0;
      document.getElementById('ch').textContent = cam.height ?? 0;
      document.getElementById('cs').textContent = cam.size_bytes ?? 0;
      if (cam.jpeg_base64) {
        document.getElementById('cam').src = 'data:image/jpeg;base64,' + cam.jpeg_base64;
      }
      document.getElementById('ts').textContent = 'Last update: ' + (data.timestamp || 'none');
    }
    setInterval(refresh, 250);
    refresh();
  </script>
</body>
</html>
"""

@app.route("/", methods=["GET"])
def dashboard():
    return render_template_string(PAGE)

@app.route("/latest", methods=["GET"])
def latest():
    return jsonify(latest_data)

@app.route("/test", methods=["GET"])
def teste():
    return "testado"
def update_latest_data(data):
    latest_data["test_mode"] = bool(data.get("test_mode", False))
    latest_data["uptime_ms"] = int(data.get("uptime_ms", 0))
    latest_data["lidar_cm"] = data.get("lidar_cm", [0, 0, 0, 0])
    latest_data["motor_integrity"] = data.get("motor_integrity", [False, False, False, False])
    latest_data["camera"] = data.get("camera", latest_data["camera"])
    latest_data["timestamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

@app.route('/data', methods=['POST'])
def receive_data():
    data = request.get_json(silent=True) or {}
    update_latest_data(data)
    print("Received telemetry at", latest_data["timestamp"])
    return jsonify({"status": "ok", "received": True})

def serial_reader_loop():
    if serial is None:
        print("PySerial not installed. Run: pip install pyserial")
        return

    while True:
        try:
            with serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1) as ser:
                print(f"PySerial connected on {SERIAL_PORT} @ {SERIAL_BAUD}")
                while True:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if not line or not line.startswith("{"):
                        continue
                    try:
                        data = json.loads(line)
                        update_latest_data(data)
                    except json.JSONDecodeError:
                        continue
        except Exception as exc:
            print(f"Serial reader error: {exc}. Retrying in 2s...")
            time.sleep(2)

if __name__ == "__main__":
    if USE_PYSERIAL:
        t = threading.Thread(target=serial_reader_loop, daemon=True)
        t.start()
    app.run(host='0.0.0.0', port=5000)