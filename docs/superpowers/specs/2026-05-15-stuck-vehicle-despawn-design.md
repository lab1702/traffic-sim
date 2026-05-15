# Stuck-Vehicle Despawn — Design

**Date:** 2026-05-15
**Status:** Approved (brainstorming phase)

## Goal

Add a defensive guard to the sim: any vehicle that has been effectively stationary for longer than expected — and is not legitimately waiting at a red light or yielding at an unsignalized/flash/off intersection — is logged with full state and despawned.

This is a follow-up item deferred from the v1 plan ([../plans/2026-05-15-traffic-sim.md](../plans/2026-05-15-traffic-sim.md), "Known follow-ups" §1). It catches unforeseen sim bugs (gridlock, routing dead-ends, IDM degeneracies) during long research runs without halting the whole simulation.

## Trigger condition

A vehicle is despawned when **all** of the following hold for >60 sim-seconds of accumulated tick time:

- `V < 0.1 m/s`
- `World.stopDistanceForRed(v)` returns `(_, false)`
- `World.stopDistanceForYield(v, byEdge)` returns `(_, false)`

The 60-second accumulator resets to 0 on any tick where one of those conditions fails. A vehicle that idles 30 seconds, moves briefly, then idles another 40 seconds is **not** despawned.

## Major decisions (from brainstorming)

| Axis | Decision |
|---|---|
| Trace event | Reuse existing `VehicleDespawn` — same wire format as normal end-of-route despawn. Reason is captured only in the sim log, not the trace. |
| Thresholds | Hardcoded constants in `internal/sim/world.go`. Matches existing style (`gapThresholdSec`, `VehicleLength`). Promote to config later if a use case appears. |
| Log level | `slog.Warn` with full vehicle state. Visible by default, doesn't fail the run. |
| State storage | One new `float64` field on `Vehicle` (mirrors `LaneChangeCooldown` pattern). |

## Placement in the tick loop

The check lives inside `World.Step()` in `internal/sim/world.go`, in the per-vehicle stepping pass (currently lines 347-388). It runs **after** `stepIDM` and **before** the existing `if v.Despawned { EmitTrace(VehicleDespawn) }` block, so a stuck-despawn flows through the same trace emission path as a normal route-completion despawn.

```
for i := range w.Vehicles {
    ...
    stepIDM(v, ...)
    // NEW: stuck-despawn check goes here.
    if v.Despawned {
        w.EmitTrace(VehicleDespawn)
    }
    ...
}
```

## State additions

### `internal/sim/vehicle.go`

Add one field to `Vehicle`:

```go
// StuckTime accumulates sim-seconds where V < stuckSpeedThresh and the
// vehicle is not legitimately waiting at a red light or yield. Reset to 0
// whenever any of those conditions fails. When it exceeds stuckTimeoutSec,
// the vehicle is logged and despawned.
StuckTime float64
```

### `internal/sim/world.go`

Two new constants near `gapThresholdSec`:

```go
const (
    stuckSpeedThresh = 0.1  // m/s — below this counts as "not moving"
    stuckTimeoutSec  = 60.0 // sim-seconds of accumulated stuck time → despawn
)
```

## Per-tick logic

After `stepIDM`, before the `if v.Despawned` emit block:

```go
if !v.Despawned && v.V < stuckSpeedThresh {
    _, isRed := w.stopDistanceForRed(v)
    _, mustYield := w.stopDistanceForYield(v, byEdge)
    if !isRed && !mustYield {
        v.StuckTime += w.dt
        if v.StuckTime > stuckTimeoutSec {
            slog.Warn("stuck vehicle despawned",
                "vehicle_id", v.ID,
                "edge", v.Edge,
                "lane", v.Lane,
                "s", v.S,
                "v", v.V,
                "route_idx", v.RouteIdx,
                "route_len", len(v.Route),
                "tick", w.Tick,
                "sim_time", w.SimTime,
                "stuck_duration", v.StuckTime,
            )
            v.Despawned = true
        }
    } else {
        v.StuckTime = 0
    }
} else {
    v.StuckTime = 0
}
```

Cost: one `<` compare per vehicle per tick; for the rare slow vehicle, two map lookups and a handful of comparisons. Negligible at city scale.

## Testing

Three new unit tests in `internal/sim/world_test.go`:

1. **`TestWorld_StuckVehicleDespawned`** — synthetic 1-edge network. Force a vehicle to V=0 (e.g. pin it behind an immovable virtual leader, or step it for 0 advancement). Advance the world >60 sim-seconds. Assert vehicle is removed from `w.Vehicles` after compact and a warning was logged.

2. **`TestWorld_StuckAtRedNotDespawned`** — vehicle approaches a red light, sits stopped for 90 sim-seconds. Assert vehicle is NOT despawned; `StuckTime` remains 0.

3. **`TestWorld_StuckAtYieldNotDespawned`** — unsignalized intersection with persistent priority traffic so the yielding vehicle remains stopped 90 sim-seconds. Assert not despawned.

For log capture, swap `slog.Default()` with a handler writing to a `bytes.Buffer` for the duration of the test, then restore.

## Determinism

The existing phase-8 determinism gate (`TestWorld_TraceDeterminism`) is the safety net: two runs from the same seed will arrive at byte-identical sim states tick-by-tick, so stuck-despawns fire at byte-identical ticks and the trace stays byte-identical. No new determinism test is needed; verify the existing one still passes after the change.

## Edge cases

- **Queueing behind a queue.** A vehicle 5 cars back from a red light has both `stopDistanceForRed` and `stopDistanceForYield` return `false` (those functions only trigger on the front of the queue). Such a vehicle accumulates `StuckTime` while waiting. This is acceptable: in healthy sim runs the queue dissipates well within 60 sim-seconds once the light turns green. If a queue genuinely stays gridlocked for >60 sim-seconds with no movement, that is exactly the class of bug this guard is intended to surface.

- **No leader, v0 > 0, vehicle not moving.** A genuine sim bug. Warn + despawn is correct.

- **Cleanup.** `World.compact()` already runs at the end of every tick and removes `Despawned` vehicles from the slice; no special teardown.

## Scope

Strictly limited to this feature:

- 1 new field on `Vehicle`
- 2 new constants in `world.go`
- ~20 lines added inside the existing per-vehicle loop in `Step()`
- 3 new tests in `world_test.go`

No config changes. No trace format changes. No renderer changes. No changes to other files.

## Out of scope

- Making thresholds configurable
- A distinct `VehicleStuckDespawn` trace event
- Reasoning about queue depth as part of the trigger
- Any change to the determinism test
