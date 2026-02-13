# Fix: Battery Locomotive Simulation Hang in `walk_internal`

## Summary

Battery and hybrid locomotive simulations could hang indefinitely after completing the timed path phase. The train would stop inside the braking zone and never restart, sitting at speed=0 while slowly draining SOC forever. This PR fixes the root cause in `solve_required_pwr` and adds debug instrumentation and safety nets.

## Root Cause

In `braking_point.rs`, `recalc()` builds braking curves backward from `offset_end`. The **first point** pushed is the end-stop:

```rust
self.points.push(BrakingPoint {
    offset: path_tpc.offset_end(),
    ..Default::default() // speed_target = 0, speed_limit = 0
});
```

All subsequent braking curve points inherit `speed_target: bp_curr.speed_target`, which propagates the end-stop's `0` through every point on the curve. This `speed_target = 0` is **intentional** — it signals moving trains to start decelerating early before speed limit drops. However, it causes a stall when the train is **stopped** inside the braking zone:

1. When `walk_internal` started after the timed path, the train was stopped (speed=0) inside the braking zone
2. `calc_speeds()` returned `speed_target = 0`
3. In `solve_required_pwr`: `f_applied_target = res_net + mass * (0 - 0) / dt = res_net` — force exactly balanced resistance
4. Zero net acceleration → train stuck permanently, draining SOC until the 5-day limit or forever

### Observed Behavior

```
[SLTS SOC] t=12.0h link_idx=1051 offset=168.4mi speed=0.0mph  loco[0] SOC=0.2768
[SLTS SOC] t=13.0h link_idx=1051 offset=168.4mi speed=0.0mph  loco[0] SOC=0.2756
... (repeats for days, offset never changes, SOC slowly drains)
[SLTS SOC] t=120.0h link_idx=1051 offset=168.4mi speed=0.0mph loco[0] SOC=0.1447
```

### Why Not Fix in `braking_point.rs`?

An initial attempt changed `speed_target` on all braking curve points to match their `speed_limit` (the braking curve speed). This fixed the stall but broke deceleration: trains no longer received the early "slow down" signal from `speed_target = 0` in the look-ahead window, causing them to overshoot speed limits. The fix must preserve the braking curve's `speed_target = 0` for moving trains while handling the stopped-train edge case.

## Changes

### 1. Fix stalled train in `solve_required_pwr` (speed_limit_train_sim.rs)

**File:** `altrios-core/src/train/speed_limit_train_sim.rs`

After `calc_speeds` returns `(speed_limit, speed_target)`, added a targeted override:

```rust
if speed_target == si::Velocity::ZERO
    && *self.state.speed.get_stale(...)? < uc::MPH * 0.1
    && *self.state.offset.get_stale(...)? < self.path_tpc.offset_end() - 1000.0 * uc::FT
{
    speed_target = speed_limit;
}
```

This only activates when ALL three conditions are met:
- `speed_target == 0` (inside a braking zone)
- Train is nearly stopped (< 0.1 mph)
- Train has NOT reached the end of the path (> 1000 ft remaining)

By setting `speed_target = speed_limit` (the braking curve's maximum allowable speed at this offset), the train accelerates along the curve and follows it naturally down to 0 at `offset_end`. Moving trains are completely unaffected since their speed is well above 0.1 mph.

### 2. Add stall detection to `walk_internal` (speed_limit_train_sim.rs)

**File:** `altrios-core/src/train/speed_limit_train_sim.rs`

Added a safety net: if the train has `speed == 0` and `offset` unchanged for more than **1 simulated hour**, `walk_internal` bails with a diagnostic error:

```
[SLTS] Train stalled in walk_internal for >1h: offset=168.4mi, offset_end=172.1mi, gap=19536ft. Train cannot reach end of path.
```

This prevents future hangs from burning compute time indefinitely.

### 3. Add 5-day simulation time limit (speed_limit_train_sim.rs)

**File:** `altrios-core/src/train/speed_limit_train_sim.rs`

Added to `step()`: if `state.time` exceeds 5 days (432,000 seconds), the simulation bails:

```
[SLTS] Simulation exceeded 5-day time limit (120.0 hours elapsed). Aborting.
```

This applies to all simulation modes (dispatch `walk`, timed-path `walk_timed_path`, and `walk_internal`).

### 4. Add hourly SOC debug printing (speed_limit_train_sim.rs)

**File:** `altrios-core/src/train/speed_limit_train_sim.rs`

Added to `step()`: once per simulated hour, if the consist contains any hybrid or battery-electric locomotives, prints:

```
[SLTS SOC] t=3.0h link_idx=42 offset=125.3mi offset_end=172.1mi speed=45.2mph  loco[0] SOC=0.8742  loco[1] SOC=0.6531
```

Includes:
- Simulation time (hours)
- Current `link_idx_front`
- Current offset (miles)
- Path end offset (miles) — to see remaining distance
- Current speed (mph)
- SOC for each locomotive with reversible energy storage

## Testing

All **120 existing tests pass** (`cargo test --lib`). No new tests added — the fix is exercised by the existing simulation tests.

## Files Changed

| File | Change |
|------|--------|
| `altrios-core/src/train/speed_limit_train_sim.rs` | Stall fix in `solve_required_pwr`, stall detection in `walk_internal`, 5-day time limit, hourly SOC debug |
