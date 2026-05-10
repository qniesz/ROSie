"""Comprehensive ROSie SLAM diagnostic analysis.

Reads a diag_capture session directory and produces:
  - Pose jump statistics with timestamps
  - Correlation of jumps with: odom drift between samples, scan rate,
    pipeline state changes, slam_toolbox events, MQTT broker events
  - Driver-side event timeline around each jump
"""
import json
import math
import re
import sys
from pathlib import Path
from collections import defaultdict

D = Path(sys.argv[1] if len(sys.argv) > 1 else "replay_out/logs/session_20260430_215307")
print(f"=== analyzing {D} ===\n")

# Robot physical limits
MAX_LIN = 0.30      # m/s, Neato D6
MAX_ANG = 1.6       # rad/s
LDS_DT = 0.20       # 5 Hz
JUMP_LIN_THRESH = MAX_LIN * LDS_DT * 1.25   # 75 mm
JUMP_ANG_THRESH = MAX_ANG * LDS_DT * 1.25   # ~23 deg


def load_jsonl_recv(path):
    """Lines: '<recv_unix> <json>' -> list of (recv, dict)."""
    out = []
    if not path.exists():
        return out
    for line in path.read_text(errors="replace").splitlines():
        if not line.strip():
            continue
        recv_str, _, payload = line.partition(" ")
        try:
            out.append((float(recv_str), json.loads(payload)))
        except Exception:
            pass
    return out


def fmt_t(t, t0=0):
    return f"+{t - t0:7.2f}s"


def pct(arr, p):
    a = sorted(arr)
    return a[min(len(a) - 1, int(len(a) * p / 100))]


def print_serial_summary(serial_samples):
    print(f"serial stats samples: {len(serial_samples)}")
    if not serial_samples:
        print()
        return

    totals = defaultdict(lambda: {
        "count": 0, "ok": 0, "fail": 0, "flush_before": 0,
        "avg_ms_weighted": 0.0, "max_ms": 0.0, "avg_lines_weighted": 0.0,
    })
    flush_count = 0
    flush_avg_weighted = 0.0
    flush_max = 0.0
    for _, sample in serial_samples:
        flush = sample.get("flush", {})
        fc = int(flush.get("count", 0) or 0)
        flush_count += fc
        flush_avg_weighted += float(flush.get("avg_ms", 0.0) or 0.0) * fc
        flush_max = max(flush_max, float(flush.get("max_ms", 0.0) or 0.0))
        for command, stats in sample.get("commands", {}).items():
            count = int(stats.get("count", 0) or 0)
            if count <= 0:
                continue
            total = totals[command]
            total["count"] += count
            total["ok"] += int(stats.get("ok", 0) or 0)
            total["fail"] += int(stats.get("fail", 0) or 0)
            total["flush_before"] += int(stats.get("flush_before", 0) or 0)
            total["avg_ms_weighted"] += float(stats.get("avg_ms", 0.0) or 0.0) * count
            total["avg_lines_weighted"] += float(stats.get("avg_lines", 0.0) or 0.0) * count
            total["max_ms"] = max(total["max_ms"], float(stats.get("max_ms", 0.0) or 0.0))

    print("=== serial command timing ===")
    if flush_count:
        print(
            f"  flushes: {flush_count}  "
            f"avg={flush_avg_weighted / flush_count:.1f}ms  max={flush_max:.1f}ms"
        )
    else:
        print("  flushes: 0")
    for command, total in sorted(totals.items(), key=lambda item: item[1]["count"], reverse=True):
        count = total["count"]
        print(
            f"  {command:18s} count={count:5d} ok={total['ok']:5d} fail={total['fail']:4d} "
            f"flush_before={total['flush_before']:5d} "
            f"avg={total['avg_ms_weighted'] / count:7.1f}ms max={total['max_ms']:7.1f}ms "
            f"avg_lines={total['avg_lines_weighted'] / count:6.1f}"
        )
    print()


# ---------- 1. Pose stream ----------
serial_stats = load_jsonl_recv(D / "serial.jsonl")
print_serial_summary(serial_stats)

poses = load_jsonl_recv(D / "pose.jsonl")
print(f"pose samples: {len(poses)}")
if not poses:
    sys.exit("no pose data")

t0 = poses[0][0]
t_end = poses[-1][0]
print(f"duration: {t_end - t0:.1f}s ({(t_end-t0)/60:.1f} min)")
print(f"avg rate: {len(poses) / (t_end - t0):.2f} Hz")
print()

# Compute per-step deltas
steps = []
for (r0, p0), (r1, p1) in zip(poses, poses[1:]):
    dt_msg = p1["t"] - p0["t"]
    dd = math.hypot(p1["x"] - p0["x"], p1["y"] - p0["y"])
    dh = (p1["theta"] - p0["theta"] + math.pi) % (2 * math.pi) - math.pi
    v = dd / dt_msg if dt_msg > 0 else 0
    w = dh / dt_msg if dt_msg > 0 else 0
    steps.append({
        "recv": r1, "t": p1["t"], "dt": dt_msg,
        "x": p1["x"], "y": p1["y"], "th": p1["theta"],
        "px": p0["x"], "py": p0["y"], "pth": p0["theta"],
        "dd": dd, "dh": dh, "v": v, "w": w,
    })


disps = [s["dd"] for s in steps]
hdgs = [abs(s["dh"]) for s in steps]
vels = [s["v"] for s in steps]
angs = [abs(s["w"]) for s in steps]
print("=== pose step distribution ===")
print(f"  disp:    p50={pct(disps,50)*1000:6.1f}mm  p90={pct(disps,90)*1000:6.1f}mm  p99={pct(disps,99)*1000:6.1f}mm  max={max(disps)*1000:6.1f}mm")
print(f"  heading: p50={math.degrees(pct(hdgs,50)):6.2f}°  p90={math.degrees(pct(hdgs,90)):6.2f}°  p99={math.degrees(pct(hdgs,99)):6.2f}°  max={math.degrees(max(hdgs)):6.2f}°")
print(f"  v:       p50={pct(vels,50):6.3f}  p90={pct(vels,90):6.3f}  max={max(vels):6.3f} m/s")
print(f"  |w|:     p50={pct(angs,50):6.3f}  p90={pct(angs,90):6.3f}  max={max(angs):6.3f} rad/s")
print()

# Identify jumps
jumps = [s for s in steps if s["dd"] > JUMP_LIN_THRESH or abs(s["dh"]) > JUMP_ANG_THRESH]
print(f"=== {len(jumps)} jumps of {len(steps)} steps ({100*len(jumps)/len(steps):.1f}%) ===")
print(f"   threshold: disp>{JUMP_LIN_THRESH*1000:.0f}mm OR |dheading|>{math.degrees(JUMP_ANG_THRESH):.1f}°")
print()


# ---------- 2. Odom stream for drift correlation ----------
odoms = load_jsonl_recv(D / "odom.jsonl")
print(f"odom samples: {len(odoms)}\n")


def odom_at(t, window=0.5):
    """Find odom samples bracketing time t (msg time, not recv)."""
    cand = [o for r, o in odoms if abs(o.get("stamp", o.get("t", 0)) - t) < window]
    return cand


# Compute odom trajectory diff between two slam pose times
def odom_delta(t_start, t_end):
    """Linear and angular distance the wheel-odom thinks it moved between t_start and t_end."""
    pre = [o for r, o in odoms if o.get("stamp", o.get("t", 0)) <= t_start]
    post = [o for r, o in odoms if o.get("stamp", o.get("t", 0)) <= t_end]
    if not pre or not post:
        return None, None
    o0, o1 = pre[-1], post[-1]
    if "x" in o0 and "x" in o1:
        dd = math.hypot(o1["x"] - o0["x"], o1["y"] - o0["y"])
        dh = (o1.get("theta", 0) - o0.get("theta", 0) + math.pi) % (2 * math.pi) - math.pi
        return dd, dh
    return None, None


# ---------- 3. Pipeline / state events ----------
def load_state_events(path):
    out = []
    for r, d in load_jsonl_recv(path):
        out.append((r, d))
    return out


pipe_events = load_state_events(D / "pipe.jsonl")
state_events = load_state_events(D / "state.jsonl")
print(f"pipeline events: {len(pipe_events)}")
print(f"driver state events: {len(state_events)}")

# Pipeline state transitions
prev_status = None
print("\n=== pipeline state timeline ===")
for r, d in pipe_events:
    s = d.get("status")
    if s != prev_status:
        detail = d.get("detail", "")
        print(f"  {fmt_t(r, t0)}  status={s:20s}  {detail}")
        prev_status = s
print()


# ---------- 4. Driver log events (no-go, recovery, errors) ----------
drv_events = []
drv_path = D / "driver.log"
if drv_path.exists():
    for line in drv_path.read_text(errors="replace").splitlines():
        # journalctl ISO timestamps: "2026-04-30T21:53:13-0400 ROSie python3[2609]: ..."
        m = re.match(r"^(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}[+-]\d{4})\s+\S+\s+\S+:\s*(.*)$", line)
        if not m:
            continue
        from datetime import datetime
        try:
            ts = datetime.strptime(m.group(1), "%Y-%m-%dT%H:%M:%S%z").timestamp()
        except Exception:
            continue
        msg = m.group(2)
        # Filter for interesting events
        if any(kw in msg for kw in [
            "no_go", "bump", "recover", "ERROR", "WARN", "stuck",
            "Pipeline:", "online slam", "slam_toolbox", "SLAM",
            "cleaning", "CLEAN", "DOCK", "dock", "Reach", "blocked",
            "outlier", "skipped", "lidar", "LDS",
        ]):
            drv_events.append((ts, msg))

print(f"interesting driver events: {len(drv_events)}")
print()

# ---------- 5. SLAM container log ----------
slam_events = []
slam_path = D / "slam.log"
if slam_path.exists():
    for line in slam_path.read_text(errors="replace").splitlines():
        # ros2 logs: "[INFO] [1777600127.123] [slam_toolbox]: ..."
        m = re.match(r"^\[(\w+)\]\s+\[(\d+\.\d+)\]\s+\[(\w+)\]:\s*(.*)$", line)
        if m:
            slam_events.append((float(m.group(2)), m.group(1), m.group(3), m.group(4)))
        else:
            # bare lines (Karto graph / Ceres warnings)
            slam_events.append((None, "RAW", "?", line))

# Loop closures and re-localization events
print("=== slam_toolbox notable events ===")
for ev in slam_events:
    ts, lvl, node, msg = ev
    if ts is None:
        if any(k in msg for k in ["loop", "Loop", "WARNING", "ERROR", "Registering"]):
            print(f"  (raw)             {msg[:120]}")
    elif any(k in msg for k in ["loop", "Loop", "elocaliz", "Initial pose", "blocked", "warn", "WARN", "failed", "Match"]):
        print(f"  {fmt_t(ts, t0)}  [{lvl}] {msg[:120]}")
print()

# ---------- 6. Detailed jump analysis ----------
print(f"=== top {min(15, len(jumps))} jumps with context ===\n")
# Sort by severity (combined disp + heading scaled to thresholds)
def severity(s):
    return s["dd"] / JUMP_LIN_THRESH + abs(s["dh"]) / JUMP_ANG_THRESH


sorted_jumps = sorted(jumps, key=severity, reverse=True)

for i, s in enumerate(sorted_jumps[:15]):
    t = s["t"]
    print(f"--- JUMP #{i+1} at {fmt_t(s['recv'], t0)} (msg t={t:.3f}) ---")
    print(f"  pose: ({s['px']:+.3f},{s['py']:+.3f},{math.degrees(s['pth']):+.1f}°) -> ({s['x']:+.3f},{s['y']:+.3f},{math.degrees(s['th']):+.1f}°)")
    print(f"  step: dt={s['dt']*1000:.0f}ms  disp={s['dd']*1000:.1f}mm  dheading={math.degrees(s['dh']):+.1f}°  v={s['v']:.2f}m/s w={math.degrees(s['w']):+.1f}°/s")
    odd, odh = odom_delta(s["t"] - s["dt"], s["t"])
    if odd is not None:
        diff_d = s["dd"] - odd
        diff_h = s["dh"] - odh
        print(f"  odom: disp={odd*1000:.1f}mm  dheading={math.degrees(odh):+.1f}°   "
              f"   slam-odom mismatch: {diff_d*1000:+.1f}mm  {math.degrees(diff_h):+.1f}°")
    # Driver events ±1.5s around jump
    near_drv = [(ts, m) for ts, m in drv_events if abs(ts - s["recv"]) < 1.5]
    if near_drv:
        print(f"  driver events ±1.5s:")
        for ts, m in near_drv[:5]:
            print(f"    {ts - s['recv']:+.2f}s  {m[:110]}")
    # Slam events ±1.5s around jump (using slam timestamp ~= driver/recv)
    near_slam = [(ts, lvl, m) for ts, lvl, n, m in slam_events if ts is not None and abs(ts - s["recv"]) < 1.5]
    if near_slam:
        print(f"  slam events ±1.5s:")
        for ts, lvl, m in near_slam[:3]:
            print(f"    {ts - s['recv']:+.2f}s  [{lvl}] {m[:100]}")
    # Pipeline state at this time
    cur_pipe = None
    for r, d in pipe_events:
        if r <= s["recv"]:
            cur_pipe = d
        else:
            break
    if cur_pipe:
        print(f"  pipeline state: {cur_pipe.get('status')}  detail={cur_pipe.get('detail','')!r}")
    print()


# ---------- 7. Time histogram of jumps ----------
print("=== jump rate over time (bins of 60s) ===")
duration = t_end - t0
nbins = max(1, int(duration / 60))
bins = [0] * nbins
for s in jumps:
    b = min(nbins - 1, int((s["recv"] - t0) / 60))
    bins[b] += 1
for i, n in enumerate(bins):
    bar = "#" * n
    print(f"  +{i*60:4d}s..+{(i+1)*60:4d}s  ({n:3d}) {bar}")
print()


# ---------- 8. Stationary residuals ----------
quiet = [s for s in steps if s["v"] < 0.005 and abs(s["w"]) < 0.05]
if quiet:
    qd = [s["dd"] for s in quiet]
    qh = [abs(s["dh"]) for s in quiet]
    print(f"=== {len(quiet)} stationary samples (v<5mm/s, w<3deg/s) ===")
    print(f"  residual disp:    p50={pct(qd,50)*1000:.2f}mm  p99={pct(qd,99)*1000:.2f}mm  max={max(qd)*1000:.2f}mm")
    print(f"  residual heading: p50={math.degrees(pct(qh,50)):.3f}°  p99={math.degrees(pct(qh,99)):.3f}°  max={math.degrees(max(qh)):.3f}°")
    print()


# ---------- 9. Summary ----------
print("=== summary ===")
print(f"  total pose updates: {len(steps)}")
print(f"  jumps (>{JUMP_LIN_THRESH*1000:.0f}mm or >{math.degrees(JUMP_ANG_THRESH):.0f}°): {len(jumps)} ({100*len(jumps)/len(steps):.1f}%)")
big_jumps = [s for s in jumps if s["dd"] > 0.15 or abs(s["dh"]) > math.radians(40)]
print(f"  severe jumps (>150mm or >40°): {len(big_jumps)}")
total_path_slam = sum(disps)
print(f"  total slam path length: {total_path_slam:.2f} m")
