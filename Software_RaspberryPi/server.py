# ==========================================
# Wind Speed & Direction Flask Server
# ==========================================
from flask import Flask, render_template, jsonify
import serial, json, time, csv, os, threading, socket
from datetime import datetime

app = Flask(__name__)

# ---------- Serial Port Configuration ----------
# ESP32 connected via /dev/ttyUSB0 at 115200 bps
SERIAL_PORT = os.environ.get("WIND_SERIAL", "/dev/ttyUSB0")
BAUD = 115200
ser = None

def _open_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
        print(f"✅ Serial opened on {SERIAL_PORT} @ {BAUD}")
    except Exception as e:
        ser = None
        print(f"❌ Serial open failed: {e}")

_open_serial()

# ---------- Global Data Variables ----------
last_data = {"spd": 0.0, "dir": 0.0}
samples = []
data_lock = threading.Lock()

# ---------- Utility: Get Local IP ----------
def get_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "127.0.0.1"

# ---------- Read Sensor Data (Single Fetch) ----------
def read_one():
    global last_data, ser
    if ser is None or not ser.is_open:
        _open_serial()
        return

    try:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            return

        # Accept only complete JSON strings
        if not (line.startswith("{") and line.endswith("}")):
            return

        d = json.loads(line)
        if "spd" in d and "dir" in d:
            with data_lock:
                last_data = {"spd": float(d["spd"]), "dir": float(d["dir"])}
                samples.append(last_data)

    except json.JSONDecodeError:
        pass
    except Exception as e:
        print("❌ read_one error:", e)

# ---------- Calculate & Save 1-Minute Average ----------
def save_average():
    global samples
    with data_lock:
        if not samples:
            return
        avg_spd = sum(d["spd"] for d in samples) / len(samples)
        avg_dir = sum(d["dir"] for d in samples) / len(samples)
        samples = []

    if not os.path.exists("data"):
        os.makedirs("data")
    
    filename = f"data/wind_{datetime.now().strftime('%Y-%m-%d')}.csv"
    write_header = not os.path.exists(filename)

    with open(filename, "a", newline="") as f:
        w = csv.writer(f)
        if write_header:
            w.writerow(["timestamp", "avg_speed(m/s)", "avg_direction(deg)"])
        w.writerow([datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    round(avg_spd, 2), round(avg_dir, 1)])
    print(f"📦 Saved 1-min avg: {avg_spd:.2f} m/s, {avg_dir:.1f}°")

# ---------- Background Sensor Polling Thread ----------
def sensor_loop():
    last_save = time.time()
    while True:
        read_one()
        if time.time() - last_save > 60:
            save_average()
            last_save = time.time()
        time.sleep(0.02)  # 50Hz lightweight polling

# ---------- Flask Routes ----------
@app.route("/")
def index():
    return render_template("index.html")  # Requires templates/index.html

@app.route("/data")
def data():
    with data_lock:
        d = dict(last_data)
    return jsonify(d)

# ---------- Main Entry Point ----------
if __name__ == "__main__":
    ip = get_ip()
    print(f"🌐 Server running at http://{ip}:5000")

    # Start background thread for sensor data acquisition
    threading.Thread(target=sensor_loop, daemon=True).start()

    # Start Flask web server (Accessible externally)
    app.run(host="0.0.0.0", port=5000, debug=False)