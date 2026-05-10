"""Analyze jump magnitudes in slam pose stream.

Reads `mosquitto_sub -F '%U %p'` capture: each line is `<recv_unix> <json>`.
Computes per-step displacement, heading change, instantaneous velocity,
and flags samples that exceed plausible motion thresholds.
"""

import json
import math
import sys
from pathlib import Path

PATH = Path(sys.argv[1] if len(sys.argv) > 1 else "replay_out/logs/pose_sample.txt")

# Robot max physical motion per LDS frame (5 Hz = 0.2 s)
# Neato D6 max linear ~0.3 m/s, max angular ~1.5 rad/s
MAX_LIN_PER_STEP = 0.3 * 0.25  # 75 mm at 5 Hz with 25% margin
MAX_ANG_PER_STEP = 1.5 * 0.25  # ~22 deg at 5 Hz

samples = []
for line in PATH.read_text().splitlines():
    if not line.strip():
        continue
    recv_str, _, payload = line.partition(" ")
    try:
        recv = float(recv_str)
        d = json.loads(payload)
        samples.append((d["t"], d["x"], d["y"], d["theta"], recv))
    except Exception as e:
        print(f"skip: {e}", file=sys.stderr)

if len(samples) < 2:
    sys.exit("need >=2 samples")

print(f"loaded {len(samples)} samples over {samples[-1][0] - samples[0][0]:.1f}s")
print()

steps = []
for (t0, x0, y0, h0, _), (t1, x1, y1, h1, _) in zip(samples, samples[1:]):
    dt = t1 - t0
    dd = math.hypot(x1 - x0, y1 - y0)
    dh = (h1 - h0 + math.pi) % (2 * math.pi) - math.pi
    v = dd / dt if dt > 0 else 0
    w = dh / dt if dt > 0 else 0
    steps.append((t1, dt, dd, dh, v, w))

# Stats
disps = [s[2] for s in steps]
hdgs = [abs(s[3]) for s in steps]
vels = [s[4] for s in steps]
angs = [abs(s[5]) for s in steps]

def pct(arr, p):
    a = sorted(arr)
    return a[min(len(a) - 1, int(len(a) * p / 100))]

print("displacement per step (m):")
print(f"  median={pct(disps,50)*1000:6.1f} mm  p90={pct(disps,90)*1000:6.1f} mm  "
      f"p99={pct(disps,99)*1000:6.1f} mm  max={max(disps)*1000:6.1f} mm")
print("heading per step (deg):")
print(f"  median={math.degrees(pct(hdgs,50)):6.2f}  p90={math.degrees(pct(hdgs,90)):6.2f}  "
      f"p99={math.degrees(pct(hdgs,99)):6.2f}  max={math.degrees(max(hdgs)):6.2f}")
print("instantaneous lin velocity (m/s):")
print(f"  median={pct(vels,50):6.3f}  p90={pct(vels,90):6.3f}  max={max(vels):6.3f}")
print("instantaneous ang velocity (rad/s):")
print(f"  median={pct(angs,50):6.3f}  p90={pct(angs,90):6.3f}  max={max(angs):6.3f}")
print()

# Flag jumps
jumps = [s for s in steps if s[2] > MAX_LIN_PER_STEP or abs(s[3]) > MAX_ANG_PER_STEP]
print(f"=== suspicious steps (disp>{MAX_LIN_PER_STEP*1000:.0f}mm or |dheading|>{math.degrees(MAX_ANG_PER_STEP):.0f}deg): {len(jumps)} of {len(steps)} ===")
for t, dt, dd, dh, v, w in jumps[:20]:
    print(f"  t={t:.2f} dt={dt*1000:5.0f}ms  disp={dd*1000:6.1f}mm  dheading={math.degrees(dh):+6.1f}deg  v={v:5.2f}m/s  w={math.degrees(w):+7.1f}deg/s")
if len(jumps) > 20:
    print(f"  ... and {len(jumps)-20} more")

# Quiet (parked) detection
quiet = [s for s in steps if s[4] < 0.005 and abs(s[5]) < 0.05]
if quiet:
    qd = [s[2] for s in quiet]
    qh = [abs(s[3]) for s in quiet]
    print()
    print(f"=== {len(quiet)} 'should be still' steps (v<5mm/s, w<3deg/s) ===")
    print(f"  residual disp: median={pct(qd,50)*1000:.2f}mm max={max(qd)*1000:.2f}mm")
    print(f"  residual hdg : median={math.degrees(pct(qh,50)):.2f}deg max={math.degrees(max(qh)):.2f}deg")
