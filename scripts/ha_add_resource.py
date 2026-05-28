"""
Register all ha/www/*.js files as Lovelace JS Module resources via HA WebSocket API.

Reads HA_URL and HA_TOKEN from the .env file in the repo root.
Safe to run multiple times — skips already-registered resources.
"""
import json
import os
import glob
import sys

# ---------------------------------------------------------------------------
# Load .env
# ---------------------------------------------------------------------------
_repo_root = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
_env_path = os.path.join(_repo_root, ".env")

def _read_env(path):
    values = {}
    if not os.path.exists(path):
        return values
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            idx = line.index("=") if "=" in line else -1
            if idx <= 0:
                continue
            k = line[:idx].strip()
            v = line[idx + 1:].strip().strip('"').strip("'")
            values[k] = v
    return values

_env = _read_env(_env_path)
HA_URL   = _env.get("HA_URL", "").rstrip("/")
HA_TOKEN = _env.get("HA_TOKEN", "")

if not HA_URL or not HA_TOKEN:
    print(f"ERROR: HA_URL and HA_TOKEN must be set in {_env_path}", file=sys.stderr)
    sys.exit(1)

# ---------------------------------------------------------------------------
# Discover JS files to register
# ---------------------------------------------------------------------------
_www_dir = os.path.join(_repo_root, "ha", "www")
js_files = sorted(glob.glob(os.path.join(_www_dir, "*.js")))

if not js_files:
    print(f"No *.js files found in {_www_dir}")
    sys.exit(0)

resource_urls = [f"/local/{os.path.basename(f)}" for f in js_files]

# ---------------------------------------------------------------------------
# WebSocket registration
# ---------------------------------------------------------------------------
try:
    import websocket
except ImportError:
    print("ERROR: websocket-client not installed. Run: pip install websocket-client", file=sys.stderr)
    sys.exit(1)

ws_url = HA_URL.replace("http://", "ws://").replace("https://", "wss://") + "/api/websocket"

ws = websocket.create_connection(ws_url, timeout=10)

def recv():
    return json.loads(ws.recv())

def send(msg):
    ws.send(json.dumps(msg))

# Auth
msg = recv()
assert msg["type"] == "auth_required", f"Expected auth_required, got {msg}"
send({"type": "auth", "access_token": HA_TOKEN})
msg = recv()
if msg["type"] != "auth_ok":
    print(f"Auth failed: {msg}", file=sys.stderr)
    sys.exit(1)

# List existing resources
send({"id": 1, "type": "lovelace/resources"})
msg = recv()
existing = {r["url"] for r in msg.get("result", [])}

# Register missing resources
msg_id = 2
registered = 0
for url in resource_urls:
    if url in existing:
        print(f"  already registered: {url}")
    else:
        send({"id": msg_id, "type": "lovelace/resources/create", "res_type": "module", "url": url})
        result = recv()
        if result.get("success"):
            print(f"  registered: {url}")
            registered += 1
        else:
            print(f"  FAILED: {url} — {result}", file=sys.stderr)
        msg_id += 1

ws.close()

if registered:
    print(f"\nRegistered {registered} resource(s). Hard-refresh HA (Ctrl+Shift+R).")
else:
    print("\nAll resources already registered.")
