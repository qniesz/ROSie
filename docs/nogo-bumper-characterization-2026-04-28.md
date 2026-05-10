# Neato Bumper-Reaction Characterization

**Date:** 2026-04-28
**Sample:** 50 virtual bump events, 1 full cleaning cycle
**Source data:** `replay_out/logs/bumper_events.jsonl`
**Analyzer:** [scripts/analyze_bump_events.py](../scripts/analyze_bump_events.py)

All 50 triggers were **virtual** (no_go_guard → bumper_sensors.trigger_virtual). Physical poll loop is disabled on this Pi.

---

## 1. What the firmware does on a virtual bump

- **Reverses** at ~0.09 m/s peak for roughly 0.5–1.5 s.
  - Median min linear velocity during 3 s window: **-0.095 m/s**
  - Worst (most negative): **-0.112 m/s**
  - 40/50 events show a clear reverse (min_lin < -0.02 m/s)
- **Then rotates AWAY from the bump side** in the same continuous motion — no pause between reverse and turn.
- **`ui_state` NEVER changes.** Robot stays in `UIMGR_STATE_HOUSECLEANINGRUNNING` for the entire 3 s reaction window. There is no exposed RECOVERY/TURN/PAUSE substate.
  - **Implication:** we cannot use `ui_state` to detect "robot is reacting to a bump". Any cooldown logic must be timer- or pose-delta-based.
- Reaction completes in ~2.5–3 s — net pose is stable by the end of our 3 s window.

---

## 2. Per-side reaction (median over n events in 3 s window)

| Side          |  n | net_dx (m) | net_dy (m) | displacement (m) | net_dθ (deg) |
|---------------|---:|-----------:|-----------:|-----------------:|-------------:|
| front_left    |  4 |     -0.001 |     -0.008 |            0.040 |        -40.9 |
| front_right   | 12 |     -0.015 |     +0.005 |            0.067 |        +52.4 |
| side_left     |  2 |     +0.069 |     -0.016 |            0.113 |        +18.8 |
| side_right    | 32 |     -0.001 |     +0.051 |            0.122 |        +21.7 |

**Patterns:**
- **Front bumps → big turn (~40–55°), small displacement (4–7 cm).** Firmware reverses then pivots sharply away.
- **Side bumps → small turn (~20°), larger displacement (11–12 cm).** Robot continues forward more; just nudges its heading.
- **Left/right asymmetry:** left-side events are rare in this run (no-go geometry made right-side approaches dominant), so left medians are small-sample noise.

---

## 3. Side-detection sanity check (does our geometry pick the right side?)

Expected: bump on **RIGHT** → +dθ (turn left); bump on **LEFT** → -dθ (turn right).

| Side          | matched | total | rate |
|---------------|--------:|------:|-----:|
| front_right   |      12 |    12 | 100% |
| side_right    |      27 |    32 |  84% |
| front_left    |       3 |     4 |  75% |
| side_left     |       1 |     2 |  50% (n=2) |
| **Overall**   |   **43**|**50** | **86%** |

Geometry is mostly correct. The 14% misses correlate with multi-bump events (49/50 events had ≥1 additional trigger inside the 3 s window — when multiple lines fire at once, the "side" we report may not match the dominant firmware reaction).

---

## 4. Multi-trigger / chain behavior

- **49/50 events had additional bump triggers within their own 3 s window** (median 3 extra triggers).
  - Most of these are our own re-pulse cadence (~160 ms while still touching), **not** separate firmware reactions.
- **Inter-event gap analysis:** 28/49 gaps between consecutive *recorder events* were < 4 s.
  - This is the **"stuck oscillating against the line"** pattern: robot reverses, turns slightly, the line corner is *still* inside guard `reach`, we fire again immediately.
- The robot does eventually escape (cleaning completed and docked), but it can take many reverse→turn→re-bump cycles before it heads in a productive direction.

---

## 5. Hypotheses for future improvements (NOT implemented)

> Recorded for discussion. We are still in observation phase.

1. **Reaction-aware cooldown** — once we detect a turn ≥ 30° **or** displacement ≥ 10 cm after a pulse, suppress further pulses on the **same side** for 2–3 s. Would let the firmware's recovery complete instead of us fighting it. Currently the guard re-fires whenever the corner is still inside `reach`.
2. **Increase footprint margin** — bump `HALF_LENGTH`/`HALF_WIDTH` from 0.165 m → ~0.20 m so we trigger earlier, giving the firmware more room to back away before crossing the actual line.
3. **Switch guard pose source to SLAM-corrected pose.** All 50 events used `pose_source="odom"`. With geometry confirmed 86% correct, **odometry drift is the most likely remaining error source**. (This was Phase 1 of the earlier improvement draft — deferred during data collection.)
4. **Side-bump double-pulse** — side-bumps' 20° turn is barely enough to escape; that's why `side_right` has so many retries. Chain a 2nd pulse after first reaction completes, only for side contacts.
5. **Reconsider front-vs-side preference** — currently `NOGO_SIDE_PULSES_BEFORE_FRONT=3` prefers side bumps. But data shows side bumps cause **more displacement** (12 cm) than front (4–7 cm) — the opposite of what we'd want for a keep-out. Hypothesis: front-only is better for **keep-out**, side is better for **nudge-along-wall**.

---

## 6. What this run does NOT tell us

- **Physical bumper behavior** — physical poll loop is disabled, so we have no comparison data for actual obstacle hits. (Hypothesis: same firmware reaction, but possibly different magnitudes if the bumper switch latches differently.)
- **SLAM-pose accuracy** — we logged odom only. Cannot yet compare "where the guard *thought* the robot was" vs "where SLAM said it was".
- **Long-term escape patterns** — we only see 3 s windows. Don't know if the robot eventually finds a heading that clears the no-go zone after N retries, or if it just happens to drift away.
- **Variance** — only 1 cycle. Need 2–3 more cycles in different starting positions to confirm the per-side medians are stable.
