#!/usr/bin/env python3
"""Analyze bumper_events.jsonl to diagnose no-go line crossing / following quality.

Usage:
    python scripts/analyze_nogo_crossing.py logs/pi/bumper_events.jsonl

Outputs:
  - Timeline of chains (consecutive bumps < 4s apart)
  - Where on the map the long chains occur
  - Wheel differential during bump reaction (turn quality)
  - How often the robot crosses the line vs is pushed away
"""
import json
import math
import sys
import time
import collections

CHAIN_GAP = 4.0   # seconds — events closer than this = same chain


def load(path):
    events = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except Exception:
                pass
    return events


def build_chains(events):
    """Group events into chains (runs < CHAIN_GAP apart)."""
    if not events:
        return []
    chains = []
    cur = [events[0]]
    for e in events[1:]:
        if e["ts"] - cur[-1]["ts"] < CHAIN_GAP:
            cur.append(e)
        else:
            chains.append(cur)
            cur = [e]
    chains.append(cur)
    return chains


def wheel_differential(w):
    """Return (left_rpm, right_rpm, differential, description)."""
    if not w:
        return None
    l = w.get("LeftWheel_RPM", 0.0)
    r = w.get("RightWheel_RPM", 0.0)
    diff = r - l
    if abs(l) < 50 and abs(r) < 50:
        desc = "stopped"
    elif l > 0 and r > 0:
        if abs(diff) < 200:
            desc = "straight_fwd"
        elif diff > 0:
            desc = "curve_left"
        else:
            desc = "curve_right"
    elif l < 0 and r < 0:
        desc = "reversing"
    elif l > 0 > r:
        desc = "spin_right"
    elif r > 0 > l:
        desc = "spin_left"
    else:
        desc = "mixed"
    return l, r, diff, desc


def main():
    if len(sys.argv) < 2:
        print("Usage: analyze_nogo_crossing.py <bumper_events.jsonl>")
        sys.exit(1)

    events = load(sys.argv[1])
    if not events:
        print("No events found.")
        sys.exit(0)

    t0 = events[0]["ts"]
    t1 = events[-1]["ts"]
    print(f"=== Loaded {len(events)} events  "
          f"{time.strftime('%H:%M:%S', time.localtime(t0))} – "
          f"{time.strftime('%H:%M:%S', time.localtime(t1))}  "
          f"({(t1-t0)/60:.1f} min) ===")
    print()

    chains = build_chains(events)
    chain_lens = [len(c) for c in chains]
    print(f"Chains (consecutive bursts < {CHAIN_GAP}s): {len(chains)}")
    print(f"  single-shot  : {sum(1 for l in chain_lens if l == 1)}")
    print(f"  chain 2-4    : {sum(1 for l in chain_lens if 2 <= l <= 4)}")
    print(f"  chain 5-9    : {sum(1 for l in chain_lens if 5 <= l <= 9)}")
    print(f"  chain ≥10    : {sum(1 for l in chain_lens if l >= 10)}")
    print(f"  max chain len: {max(chain_lens)}")
    print()

    # Per-chain: where, how long, side, wheel action
    print("=== Longest chains (robot stuck at line) ===")
    print(f"{'len':>4}  {'side':12}  {'pos (m)':>18}  {'hdg':>6}  {'pre_wheels / turn':30}  {'time':8}")
    long_chains = sorted(chains, key=len, reverse=True)[:12]
    for c in long_chains:
        e = c[0]
        pre = e.get("pre_pose") or {}
        side = e["trigger"]["side"]
        x = pre.get("x", float("nan"))
        y = pre.get("y", float("nan"))
        hdg = math.degrees(pre.get("theta", 0.0))
        ts = time.strftime("%H:%M:%S", time.localtime(e["ts"]))
        pw = e.get("pre_wheels")
        if pw:
            wd = wheel_differential(pw)
            wheel_str = f"L={wd[0]:.0f} R={wd[1]:.0f} ({wd[3]})" if wd else "-"
        else:
            wheel_str = "no data"
        print(f"{len(c):4d}  {side:12}  ({x:6.3f}, {y:6.3f})  {hdg:6.1f}°  {wheel_str:30}  {ts}")
    print()

    # Turn quality: did the robot actually turn away from the line?
    print("=== Turn direction correctness per chain length ===")
    print("(shorter chains = escaped quickly; longer = got stuck)")
    buckets = {"1": [], "2-4": [], "5-9": [], "≥10": []}
    for c in chains:
        e = c[0]
        side = e["trigger"]["side"]
        pre = e.get("pre_pose") or {}
        samples = e.get("samples", [])
        last = samples[-1] if samples else {}
        if "theta" not in pre or "theta" not in last:
            continue
        dtheta = last["theta"] - pre["theta"]
        # Normalise to -π..π
        dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
        if side in ("front_right", "side_right"):
            correct = dtheta > 0
        else:
            correct = dtheta < 0
        key = "1" if len(c) == 1 else ("2-4" if len(c) <= 4 else ("5-9" if len(c) <= 9 else "≥10"))
        buckets[key].append(correct)

    for k in ["1", "2-4", "5-9", "≥10"]:
        vals = buckets[k]
        if vals:
            pct = 100 * sum(vals) / len(vals)
            print(f"  chain {k:5s}: {sum(vals):3d}/{len(vals):3d} correct turns ({pct:.0f}%)")
    print()

    # Wheel differential during reaction — only events with wheel data
    events_with_wheels = [e for e in events if e.get("pre_wheels")]
    print(f"=== Wheel state at bump time  ({len(events_with_wheels)} events with wheel data) ===")
    motions = collections.Counter()
    for e in events_with_wheels:
        wd = wheel_differential(e["pre_wheels"])
        if wd:
            motions[wd[3]] += 1
    for m, n in motions.most_common():
        print(f"  {m:20s}: {n}")
    print()

    # Sample-level wheel differential across reaction window
    all_samples = []
    for e in events_with_wheels:
        for s in e.get("samples", []):
            if "wheels" in s:
                all_samples.append((s["dt"], s["wheels"]))
    if all_samples:
        print(f"=== Wheel differential during reaction window ({len(all_samples)} samples) ===")
        by_dt = collections.defaultdict(list)
        for dt, w in all_samples:
            bucket = round(dt / 0.5) * 0.5   # 500ms buckets
            wd = wheel_differential(w)
            if wd:
                by_dt[bucket].append(wd[2])  # differential = R - L RPM
        print(f"  {'dt (s)':>8}  {'mean diff R-L RPM':>18}  {'n':>4}")
        for dt in sorted(by_dt):
            vals = by_dt[dt]
            mean = sum(vals) / len(vals)
            print(f"  {dt:8.1f}  {mean:18.0f}  {len(vals):4d}")
    else:
        print("No sample-level wheel data (events were captured before code update)")
        print("→ Wheel data only in the last 13 events of this run.")
    print()

    # Pose crossing check — did the robot end up on the same or wrong side?
    print("=== Net displacement after reaction (should move AWAY from line) ===")
    print(f"  {'side':12}  {'n':>4}  {'median_disp':>12}  {'median_dtheta':>14}")
    by_side = collections.defaultdict(list)
    for e in events:
        pre = e.get("pre_pose") or {}
        samples = e.get("samples", [])
        last = next((s for s in reversed(samples) if "x" in s), None)
        if not pre or not last or "x" not in pre:
            continue
        dx = last["x"] - pre["x"]
        dy = last["y"] - pre["y"]
        disp = math.hypot(dx, dy)
        dtheta = math.degrees(math.atan2(
            math.sin(last["theta"] - pre["theta"]),
            math.cos(last["theta"] - pre["theta"])
        ))
        by_side[e["trigger"]["side"]].append((disp, dtheta))

    for side in ["front_left", "front_right", "side_left", "side_right"]:
        vals = by_side.get(side, [])
        if not vals:
            continue
        disps = sorted(v[0] for v in vals)
        dthetas = sorted(v[1] for v in vals)
        med_d = disps[len(disps)//2]
        med_t = dthetas[len(dthetas)//2]
        print(f"  {side:12}  {len(vals):4d}  {med_d:12.3f}  {med_t:14.1f}°")
    print()

    print("=== DIAGNOSIS ===")
    stuck_pct = 100 * sum(1 for l in chain_lens if l >= 5) / len(chains)
    total_chains_over2 = sum(1 for l in chain_lens if l >= 2)
    print(f"  {stuck_pct:.0f}% of encounters needed ≥5 bumps to escape (stuck oscillating)")
    print(f"  {total_chains_over2} encounters ({100*total_chains_over2/len(chains):.0f}%) needed >1 bump")
    print()
    if stuck_pct > 30:
        print("  ⚠ HIGH chain rate — robot is oscillating against line rather than cleanly bouncing away.")
        print("    Likely causes:")
        print("    1. Cooldown too short → guard re-fires before firmware reaction completes")
        print("    2. Safety margin (HALF_LENGTH/HALF_WIDTH) too small → trips too late")
        print("    3. Side bumps not rotating robot far enough (side_right median dtheta ~20°)")
        print()
        print("  Suggested tuning:")
        print("    ROSIE_NOGO_HALF_LENGTH=0.22  (from 0.20) — trip earlier, more room to back away")
        print("    front_cooldown_secs: increase so firmware 50° turn completes")
        print("    side_cooldown_dtheta_deg: raise threshold before re-pulsing")


if __name__ == "__main__":
    main()
