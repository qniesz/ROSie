#!/usr/bin/env python3
import json, math, sys
LOG = sys.argv[1] if len(sys.argv) > 1 else "/home/rosie/scan_logs/scan_20260512-192319.jsonl"
xs, ys = [], []
mn, mx = [1e9,1e9], [-1e9,-1e9]
last = None
total = 0.0
big_jumps = []
heading_jumps = []
dt_anom = []
n_scans_no_returns = 0
last_t = None
gap_anom = []
with open(LOG) as f:
    for ln in f:
        try:
            r = json.loads(ln)
        except Exception:
            continue
        o = r.get("odom")
        t = float(r.get("t", 0.0))
        scan = (r.get("scan") or {})
        ranges = scan.get("ranges") or []
        valid = sum(1 for v in ranges if v and v > 0.05 and v < 5.0 and math.isfinite(v))
        if valid < 30:
            n_scans_no_returns += 1
        if last_t is not None:
            gap = t - last_t
            if gap > 0.5:
                gap_anom.append((t, gap))
        last_t = t
        if not o:
            continue
        x, y, th = float(o["x"]), float(o["y"]), float(o["theta"])
        xs.append(x); ys.append(y)
        mn[0] = min(mn[0], x); mn[1] = min(mn[1], y)
        mx[0] = max(mx[0], x); mx[1] = max(mx[1], y)
        if last is not None:
            dx = x - last[0]; dy = y - last[1]; dth = th - last[2]
            while dth > math.pi: dth -= 2*math.pi
            while dth < -math.pi: dth += 2*math.pi
            step = math.hypot(dx, dy)
            total += step
            if step > 0.20:
                big_jumps.append((t, step, x, y))
            if abs(dth) > math.radians(30):
                heading_jumps.append((t, math.degrees(dth)))
        last = (x, y, th)

span = (mx[0]-mn[0], mx[1]-mn[1])
print(f"rows_with_odom={len(xs)} bbox=({mn[0]:.2f},{mn[1]:.2f})..({mx[0]:.2f},{mx[1]:.2f}) span={span[0]:.2f}m x {span[1]:.2f}m")
print(f"total_path={total:.1f}m  big_xy_jumps>0.20m={len(big_jumps)}  big_heading_jumps>30deg={len(heading_jumps)}")
print(f"empty_scans(<30 valid)={n_scans_no_returns}")
print(f"time_gaps>0.5s={len(gap_anom)}")
for j in big_jumps[:10]: print(f"  XY jump: t={j[0]:.1f} step={j[1]:.3f}m at ({j[2]:.2f},{j[3]:.2f})")
for j in heading_jumps[:10]: print(f"  HEAD jump: t={j[0]:.1f} dtheta={j[1]:.1f}deg")
for j in gap_anom[:10]: print(f"  GAP: t={j[0]:.1f} gap={j[1]:.2f}s")
