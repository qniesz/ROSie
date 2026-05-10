"""Quick aggregate analysis of bumper_events.jsonl."""
import json
import math
import statistics
import sys
from collections import Counter, defaultdict
from pathlib import Path

path = Path(sys.argv[1] if len(sys.argv) > 1
            else "replay_out/logs/bumper_events.jsonl")

events = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
print(f"Total events: {len(events)}")
print()

# Group by trigger side
by_side = defaultdict(list)
for ev in events:
    by_side[ev["trigger"]["side"]].append(ev)

print("=== Events per side ===")
for side, evs in sorted(by_side.items()):
    print(f"  {side:14s} {len(evs):3d}")
print()

print("=== Virtual vs Physical ===")
virt = sum(1 for e in events if e["trigger"]["virtual"])
print(f"  virtual : {virt}")
print(f"  physical: {len(events) - virt}")
print()

print("=== UI state transitions during bumps ===")
transitions = Counter()
for ev in events:
    states = [c["ui_state"] for c in ev.get("ui_state_changes", [])]
    if len(states) > 1:
        for a, b in zip(states, states[1:]):
            transitions[f"{a} -> {b}"] += 1
if transitions:
    for t, n in transitions.most_common():
        print(f"  {n:3d}  {t}")
else:
    print("  (none — UI state never changed during any 3s window)")
print()

print("=== Aggregate per-side reaction (within 3s window) ===")
print(f"{'side':14s} {'n':>4s} "
      f"{'net_dx_m':>12s} {'net_dy_m':>12s} {'net_disp_m':>12s} "
      f"{'net_dtheta_deg':>16s} {'min_lin':>10s} {'max_ang':>10s}")
for side, evs in sorted(by_side.items()):
    dx = [e.get("net_dx", 0) for e in evs]
    dy = [e.get("net_dy", 0) for e in evs]
    disp = [math.hypot(a, b) for a, b in zip(dx, dy)]
    dth = [math.degrees(e.get("net_dtheta", 0)) for e in evs]
    min_lin = []
    max_ang = []
    for e in evs:
        lins = [s.get("lin", 0) for s in e.get("samples", []) if "lin" in s]
        angs = [abs(s.get("ang", 0)) for s in e.get("samples", []) if "ang" in s]
        if lins:
            min_lin.append(min(lins))
        if angs:
            max_ang.append(max(angs))
    def med(xs):
        return statistics.median(xs) if xs else 0.0
    print(f"{side:14s} {len(evs):4d} "
          f"{med(dx):>12.3f} {med(dy):>12.3f} {med(disp):>12.3f} "
          f"{med(dth):>16.1f} {med(min_lin):>10.3f} {med(max_ang):>10.3f}")
print()

print("=== Reverse evidence (min lin during window) ===")
all_min_lin = []
for ev in events:
    lins = [s.get("lin", 0) for s in ev.get("samples", []) if "lin" in s]
    if lins:
        all_min_lin.append(min(lins))
n_reversed = sum(1 for v in all_min_lin if v < -0.02)
print(f"  events showing reverse (min_lin < -0.02 m/s): {n_reversed}/{len(all_min_lin)}")
if all_min_lin:
    print(f"  median min_lin: {statistics.median(all_min_lin):.3f} m/s")
    print(f"  worst (most negative): {min(all_min_lin):.3f} m/s")
print()

print("=== Turn direction vs bump side (sanity check) ===")
print("Expected: bump on RIGHT side -> turn LEFT (positive dtheta)")
print("          bump on LEFT side  -> turn RIGHT (negative dtheta)")
for side, evs in sorted(by_side.items()):
    expected_dir = "+" if "right" in side else "-"
    correct = 0
    for e in evs:
        dth = e.get("net_dtheta", 0)
        if (expected_dir == "+" and dth > 0.1) or (expected_dir == "-" and dth < -0.1):
            correct += 1
    print(f"  {side:14s} expected {expected_dir}theta -> {correct}/{len(evs)} matched")
print()

print("=== Multi-trigger overlap rate ===")
extras = [len(e.get("additional_triggers", [])) for e in events]
print(f"  events with extra triggers in 3s window: "
      f"{sum(1 for x in extras if x > 0)}/{len(events)}")
print(f"  median extras per event: {statistics.median(extras):.1f}")
print(f"  max extras in single event: {max(extras) if extras else 0}")
print()

print("=== Time gaps between events (chain analysis) ===")
gaps = []
for a, b in zip(events, events[1:]):
    gaps.append(b["ts"] - a["ts"])
if gaps:
    print(f"  median gap: {statistics.median(gaps):.2f}s")
    print(f"  events fired <4s after prior: "
          f"{sum(1 for g in gaps if g < 4)}/{len(gaps)}  "
          f"(robot stuck retrying same line)")
