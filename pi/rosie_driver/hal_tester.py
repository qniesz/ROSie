"""hal_tester.py - Real-time HAL sensor hold-button tester with live map overlay.

Serves a web page on port 8765 (Pi LAN only).
- Hold buttons activate HAL state instantly via direct HTTP (no HA/MQTT round-trip).
- Base map (dock marker only) from MQTT rosie/map_image_base.
- Robot position overlaid in real-time via SSE, using AMCL pose from rosie/pose.
- Raw MAG sensor values polled every 500 ms.

Browse to http://<pi-ip>:8765 from any device on the LAN.
"""

import base64
import json
import logging
import math
import queue
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import paho.mqtt.client as _mqtt_lib
except ImportError:
    _mqtt_lib = None

from . import no_go_guard

logger = logging.getLogger(__name__)

_PORT = 8765

# ---------------------------------------------------------------------------
# Module-level state (updated by MQTT subscriber)
# ---------------------------------------------------------------------------
_latest_map_png: bytes = b""   # decoded PNG bytes from rosie/map_image_base
_latest_meta: dict = {}         # rosie/map_meta JSON
_map_lock = threading.Lock()

# SSE: all active event-stream response queues
_sse_clients: list = []
_sse_lock = threading.Lock()

_sensor_callback = lambda: {"mag_left": None, "mag_right": None}


def set_sensor_callback(fn):
    """Register a callable returning {'mag_left': int, 'mag_right': int}."""
    global _sensor_callback
    _sensor_callback = fn


def _push_pose_event(x: float, y: float, theta: float) -> None:
    data = json.dumps({"x": x, "y": y, "theta": theta})
    msg = f"data: {data}\n\n"
    with _sse_lock:
        dead = []
        for q in _sse_clients:
            try:
                q.put_nowait(msg)
            except Exception:
                dead.append(q)
        for q in dead:
            _sse_clients.remove(q)


# ---------------------------------------------------------------------------
# HTML page (bytes literal — avoids any encoding issues)
# ---------------------------------------------------------------------------
_HTML = b"""<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ROSie HAL Tester</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    font-family: sans-serif; background: #111; color: #eee;
    display: flex; flex-direction: column; align-items: center;
    padding: 16px; gap: 14px;
  }
  h1 { font-size: 1.25rem; letter-spacing: 0.05em; }
  .row { display: flex; gap: 14px; }
  .col { display: flex; flex-direction: column; align-items: center; gap: 8px; }
  .lbl { font-size: 0.75rem; opacity: 0.5; text-transform: uppercase; letter-spacing: 0.07em; }
  button {
    width: 130px; height: 72px; border: 2px solid transparent;
    border-radius: 10px; font-size: 0.9rem; font-weight: bold;
    cursor: pointer; touch-action: none; user-select: none;
    transition: filter 0.04s, border-color 0.04s;
  }
  button.held { filter: brightness(1.5); border-color: #fff; }
  .s1 { background: #1a5c38; color: #fff; }
  .s2 { background: #6b1a1a; color: #fff; }
  .sensor-row { display: flex; gap: 32px; }
  .sensor { display: flex; flex-direction: column; align-items: center; gap: 2px; }
  .sensor-val { font-size: 1.6rem; font-weight: bold; font-variant-numeric: tabular-nums; min-width: 4ch; text-align: center; }
  #status { font-size: 0.75rem; opacity: 0.4; min-height: 1em; }
  #map-wrap { position: relative; display: inline-block; }
  #map-img  { display: block; max-width: min(90vw, 500px); border-radius: 8px; background: #222; }
  #map-canvas { position: absolute; top: 0; left: 0; pointer-events: none; }
  #map-status { font-size: 0.7rem; opacity: 0.35; }
</style>
</head>
<body>
<h1>ROSie HAL Tester</h1>

<div class="row">
  <div class="col">
    <div class="lbl">Left &mdash; GPIO18</div>
    <button class="s1"
      onpointerdown="hold(event,'left',1,this)"
      onpointerup="release('left',this)"
      onpointerleave="release('left',this)"
      onpointercancel="release('left',this)">State 1<br><small>Approaching</small></button>
    <button class="s2"
      onpointerdown="hold(event,'left',2,this)"
      onpointerup="release('left',this)"
      onpointerleave="release('left',this)"
      onpointercancel="release('left',this)">State 2<br><small>On Boundary</small></button>
  </div>
  <div class="col">
    <div class="lbl">Right &mdash; GPIO19</div>
    <button class="s1"
      onpointerdown="hold(event,'right',1,this)"
      onpointerup="release('right',this)"
      onpointerleave="release('right',this)"
      onpointercancel="release('right',this)">State 1<br><small>Approaching</small></button>
    <button class="s2"
      onpointerdown="hold(event,'right',2,this)"
      onpointerup="release('right',this)"
      onpointerleave="release('right',this)"
      onpointercancel="release('right',this)">State 2<br><small>On Boundary</small></button>
  </div>
  <div class="col">
    <div class="lbl">Left Raw</div>
    <div class="sensor-val" id="mag-left">--</div>
    <div class="lbl">Right Raw</div>
    <div class="sensor-val" id="mag-right">--</div>
  </div>
</div>

<div id="status">Ready</div>

<div id="map-wrap">
  <img id="map-img" src="/map.png" alt="Map loading..." />
  <canvas id="map-canvas"></canvas>
</div>
<div id="map-status">Connecting...</div>

<script>
// ---- HAL hold buttons ----
function post(path, body) {
  fetch(path, {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify(body), keepalive: true}).catch(()=>{});
}
function hold(ev, side, state, btn) {
  btn.setPointerCapture(ev.pointerId);
  btn.classList.add('held');
  document.getElementById('status').textContent = side + ' \u2192 State ' + state + ' (held)';
  post('/hal', {side, state});
}
function release(side, btn) {
  if (!btn.classList.contains('held')) return;
  btn.classList.remove('held');
  document.getElementById('status').textContent = side + ' \u2192 released';
  post('/hal', {side, state: 0});
}

// ---- MAG sensor polling ----
async function pollSensors() {
  try {
    const r = await fetch('/sensors');
    if (r.ok) {
      const d = await r.json();
      document.getElementById('mag-left').textContent  = d.mag_left  != null ? d.mag_left  : '--';
      document.getElementById('mag-right').textContent = d.mag_right != null ? d.mag_right : '--';
    }
  } catch(_) {}
  setTimeout(pollSensors, 500);
}
pollSensors();

// ---- Map + real-time robot overlay ----
// Base map is dock-only (no robot baked in); robot drawn live via SSE AMCL pose.
const img    = document.getElementById('map-img');
const canvas = document.getElementById('map-canvas');
const ctx    = canvas.getContext('2d');
let meta = null;
let robotPose = null;  // {x, y, theta} in map frame (AMCL)

// JS port of server/clean_map.py map_to_pixel()
function mapToPixel(mx, my, m) {
  const ox = m.origin[0], oy = m.origin[1];
  let px = (mx - ox) / m.resolution;
  let py = (m.map_h - 1) - (my - oy) / m.resolution;
  if (Math.abs(m.angle) > 0.1) {
    const rad = m.angle * Math.PI / 180;
    const ca = Math.cos(rad), sa = Math.sin(rad);
    const cx = m.pre_rot_w / 2, cy = m.pre_rot_h / 2;
    const dx = px - cx, dy = py - cy;
    px = ca * dx - sa * dy + m.post_rot_w / 2;
    py = sa * dx + ca * dy + m.post_rot_h / 2;
  }
  px = px * m.scale - m.crop_c + m.border;
  py = py * m.scale - m.crop_r + m.border;
  return [px, py];
}

function drawOverlay() {
  if (!meta || !robotPose) return;
  const scaleX = canvas.width  / img.naturalWidth;
  const scaleY = canvas.height / img.naturalHeight;

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const [rx, ry] = mapToPixel(robotPose.x, robotPose.y, meta);
  const sx = rx * scaleX, sy = ry * scaleY;
  const r = Math.max(8, 12 * scaleX);

  // Heading arrow — adjust theta for map rotation
  const adjTheta = robotPose.theta + (meta.angle || 0) * Math.PI / 180;
  const hx = Math.cos(adjTheta), hy = -Math.sin(adjTheta);
  ctx.beginPath();
  ctx.moveTo(sx, sy);
  ctx.lineTo(sx + hx * r * 1.8, sy + hy * r * 1.8);
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = Math.max(2, 3 * scaleX);
  ctx.stroke();

  // Robot circle
  ctx.beginPath();
  ctx.arc(sx, sy, r, 0, Math.PI * 2);
  ctx.fillStyle = 'rgba(0,180,255,0.85)';
  ctx.fill();
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = Math.max(1.5, 2 * scaleX);
  ctx.stroke();
}

function resizeCanvas() {
  canvas.width  = img.offsetWidth;
  canvas.height = img.offsetHeight;
  drawOverlay();
}

img.onload = () => {
  resizeCanvas();
  document.getElementById('map-status').textContent = 'Map loaded';
};
window.addEventListener('resize', resizeCanvas);

// Fetch map meta from server
async function loadMeta() {
  try {
    const r = await fetch('/meta.json');
    if (r.ok) {
      meta = await r.json();
      drawOverlay();
    }
  } catch(_) {}
}
loadMeta();

// Poll base map for dock position updates (infrequent — dock rarely moves)
let lastMapEtag = '';
async function refreshMap() {
  try {
    const r = await fetch('/map.png', {cache: 'no-store'});
    if (r.ok) {
      const etag = r.headers.get('X-Map-Version') || '';
      if (etag !== lastMapEtag) {
        lastMapEtag = etag;
        const blob = await r.blob();
        const url = URL.createObjectURL(blob);
        const old = img.src;
        img.src = url;
        if (old.startsWith('blob:')) URL.revokeObjectURL(old);
        await loadMeta();
      }
    }
  } catch(_) {}
  setTimeout(refreshMap, 5000);
}
refreshMap();

// SSE for real-time AMCL pose (5 Hz from ROS2 bridge via Pi MQTT subscriber)
function connectPoseStream() {
  const es = new EventSource('/events');
  es.onmessage = ev => {
    try {
      robotPose = JSON.parse(ev.data);
      drawOverlay();
    } catch(_) {}
  };
  es.onerror = () => {
    es.close();
    setTimeout(connectPoseStream, 2000);
  };
}
connectPoseStream();
</script>
</body>
</html>
"""


# ---------------------------------------------------------------------------
# MQTT subscriber (subscribes to map_image from the server)
# ---------------------------------------------------------------------------
def _start_mqtt_subscriber(host: str, port: int, user: str, password: str,
                            prefix: str) -> None:
    if _mqtt_lib is None:
        logger.warning("[hal_tester] paho-mqtt not available — map won't load")
        return

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(f"{prefix}/map_image_base", 0)
            client.subscribe(f"{prefix}/map_meta", 0)
            client.subscribe(f"{prefix}/pose", 0)
            logger.info("[hal_tester] MQTT subscriber connected, watching map_image_base + pose")

    def on_message(client, userdata, msg):
        global _latest_map_png, _latest_meta
        try:
            if msg.topic == f"{prefix}/map_image_base":
                raw = msg.payload
                if raw:
                    with _map_lock:
                        _latest_map_png = base64.b64decode(raw)
            elif msg.topic == f"{prefix}/map_meta":
                with _map_lock:
                    _latest_meta = json.loads(msg.payload)
            elif msg.topic == f"{prefix}/pose":
                data = json.loads(msg.payload)
                _push_pose_event(
                    float(data["x"]), float(data["y"]), float(data.get("theta", 0))
                )
        except Exception as exc:
            logger.debug("[hal_tester] MQTT message error: %s", exc)

    client = _mqtt_lib.Client(client_id="rosie-hal-tester-map", protocol=_mqtt_lib.MQTTv311)
    if user:
        client.username_pw_set(user, password)
    client.on_connect = on_connect
    client.on_message = on_message

    def _run():
        while True:
            try:
                client.connect(host, port, keepalive=60)
                client.loop_forever()
            except Exception as exc:
                logger.debug("[hal_tester] MQTT map subscriber error: %s", exc)
            threading.Event().wait(5)

    t = threading.Thread(target=_run, name="hal-tester-mqtt", daemon=True)
    t.start()


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------
class _Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _json(self, data, status=200):
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(_HTML)))
            self.end_headers()
            self.wfile.write(_HTML)

        elif self.path == "/sensors":
            self._json(_sensor_callback())

        elif self.path == "/map.png":
            with _map_lock:
                data = _latest_map_png
            if data:
                import hashlib
                version = hashlib.md5(data).hexdigest()[:8]
                self.send_response(200)
                self.send_header("Content-Type", "image/png")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("X-Map-Version", version)
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)
            else:
                # Return 1x1 transparent PNG as placeholder
                placeholder = base64.b64decode(
                    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNk+M9QDwADhgGAWjR9awAAAABJRU5ErkJggg==")
                self.send_response(200)
                self.send_header("Content-Type", "image/png")
                self.send_header("Content-Length", str(len(placeholder)))
                self.send_header("X-Map-Version", "none")
                self.end_headers()
                self.wfile.write(placeholder)

        elif self.path == "/meta.json":
            with _map_lock:
                meta = dict(_latest_meta)
            self._json(meta)

        elif self.path == "/events":
            # Server-Sent Events — streams AMCL pose updates to the browser at 5 Hz
            q: queue.Queue = queue.Queue(maxsize=30)
            with _sse_lock:
                _sse_clients.append(q)
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                while True:
                    try:
                        msg = q.get(timeout=15)
                        self.wfile.write(msg.encode())
                        self.wfile.flush()
                    except queue.Empty:
                        self.wfile.write(b": ping\n\n")
                        self.wfile.flush()
            except Exception:
                pass
            finally:
                with _sse_lock:
                    try:
                        _sse_clients.remove(q)
                    except ValueError:
                        pass

        else:
            self.send_response(404)
            self.end_headers()
        if self.path == "/hal":
            length = int(self.headers.get("Content-Length", 0))
            try:
                body = json.loads(self.rfile.read(length))
                side  = str(body["side"])
                state = int(body["state"])
                if side in ("left", "right") and state in (0, 1, 2):
                    no_go_guard.set_channel_state(side, state)
                    self._json({"ok": True})
                else:
                    self._json({"ok": False, "error": "invalid side/state"}, 400)
            except Exception as exc:
                self._json({"ok": False, "error": str(exc)}, 400)
        else:
            self.send_response(404)
            self.end_headers()


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------
def start(mqtt_host: str = "localhost", mqtt_port: int = 1883,
          mqtt_user: str = "", mqtt_pass: str = "",
          mqtt_prefix: str = "rosie") -> None:
    """Start HTTP server and MQTT map subscriber. Returns immediately."""
    _start_mqtt_subscriber(mqtt_host, mqtt_port, mqtt_user, mqtt_pass, mqtt_prefix)
    server = ThreadingHTTPServer(("0.0.0.0", _PORT), _Handler)
    t = threading.Thread(target=server.serve_forever,
                         name="hal-tester-http", daemon=True)
    t.start()
    logger.info("[hal_tester] hold-button UI at http://<pi-ip>:%d", _PORT)
