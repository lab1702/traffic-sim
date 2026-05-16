# Driver-Gap Heterogeneity + Impatience — Phase 3 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make critical-gap acceptance heterogeneous (per-driver `GapFactor` sampled at spawn) and time-varying (linear-decay impatience that shrinks the accepted gap with wait time, floored at `minAcceptedGap`).

**Architecture:** Two new `Vehicle` fields — `GapFactor` (sampled at spawn alongside the existing `SpeedFactor`) and `WaitTime` (incremented each tick when slow-and-yielding). A new `effectiveGap` helper centralizes the curve: `max(baseGap × GapFactor − decayRate × WaitTime, minAcceptedGap)`. Both Phase 1's `yieldGapCheck` and Phase 4's `leftTurnYieldsToOpposing` call it instead of comparing against the raw `gapThresholdSec` / `leftTurnGapSec` constants.

**Tech Stack:** Go 1.x; existing dependencies only.

**Spec:** `docs/superpowers/specs/2026-05-15-impatience-phase-3-design.md`

---

## File map

| File | Change |
|---|---|
| `internal/sim/vehicle.go` | Add `GapFactor float64` + `WaitTime float64` fields; zero `WaitTime` in the existing edge-transition block of `stepIDM`. |
| `internal/sim/world.go` | Add Phase 3 constants (`gapFactorStdDev`, `gapFactorMin`, `gapFactorMax`, `impatienceDecayRate`, `minAcceptedGap`); add `effectiveGap` helper; update `yieldGapCheck` and `leftTurnYieldsToOpposing` call sites; sample `GapFactor` in `trySpawn`; update `WaitTime` in `Step`. |
| `internal/sim/world_test.go` | 7 new behavioral tests; 4 fixture updates to existing tests. |

---

## Task 1: Add `GapFactor` and `WaitTime` fields on `Vehicle`

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/vehicle.go`

- [ ] **Step 1: Add the two fields**

In `internal/sim/vehicle.go`, in the `Vehicle` struct, add both fields just below the existing `SpeedFactor` field:

```go
	// GapFactor is a per-driver multiplier on critical-gap thresholds
	// (gapThresholdSec for straight crossings, leftTurnGapSec for left
	// turns). Sampled at spawn from Normal(1.0, gapFactorStdDev) and
	// clamped to [gapFactorMin, gapFactorMax]. A zero value is treated
	// as 1.0 to keep hand-constructed test vehicles working without
	// modification.
	GapFactor float64

	// WaitTime accumulates sim-seconds during which the vehicle is
	// effectively stopped (V < stuckSpeedThresh) AND yielding via
	// gap-acceptance (mustYield or mustYieldLT). Resets to 0 the moment
	// either condition stops being true. Drives the impatience curve
	// in effectiveGap; does NOT apply to red lights.
	WaitTime float64
```

- [ ] **Step 2: Zero `WaitTime` on edge transition**

In `internal/sim/vehicle.go`, find the existing edge-transition block in `stepIDM`. After Phase 1 it looks like:

```go
		v.Edge = v.Route[v.RouteIdx]
		edge = &net.Edges[v.Edge]

		// Clear any mandatory-stop arrival timestamp now that we've left
		// the prior approach.
		v.StoppedSinceSec = 0
```

Change the comment and add the WaitTime reset:

```go
		v.Edge = v.Route[v.RouteIdx]
		edge = &net.Edges[v.Edge]

		// Clear any mandatory-stop arrival timestamp and accumulated
		// impatience now that we've left the prior approach.
		v.StoppedSinceSec = 0
		v.WaitTime = 0
```

- [ ] **Step 3: Build**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go build ./...`
Expected: exit 0.

- [ ] **Step 4: Run sim tests**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/`
Expected: PASS. The new fields are unused everywhere else; the WaitTime reset is a no-op until later tasks populate it.

- [ ] **Step 5: Commit**

```
git add internal/sim/vehicle.go
git commit -m "feat(sim): add Vehicle.GapFactor and Vehicle.WaitTime fields"
```

---

## Task 2: Add Phase 3 constants

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world.go`

- [ ] **Step 1: Add the constants**

In `internal/sim/world.go`, find the const block where Phase 4's `leftTurnGapSec` lives. After Phase 4 the block looks like:

```go
const gapThresholdSec = 3.0

const leftTurnGapSec = 6.0

const (
	stuckSpeedThresh = 0.1
	stuckTimeoutSec = 60.0
	stopDwellSec = 0.5
	stopLineTolMeters = 8.0
)
```

(Existing comments preserved.)

Add a new standalone const group just after `leftTurnGapSec`:

```go
const gapThresholdSec = 3.0

const leftTurnGapSec = 6.0

const (
	// Per-driver gap preference — Normal(1.0, gapFactorStdDev) clamped
	// to [gapFactorMin, gapFactorMax]. Same Normal-then-clamp shape as
	// SpeedFactor but wider, since gap tolerance varies more across
	// drivers than cruising-speed preference.
	gapFactorStdDev = 0.1
	gapFactorMin    = 0.8
	gapFactorMax    = 1.2

	// impatienceDecayRate is the seconds of accepted-gap reduction per
	// second of wait time. At 0.1, a 30-second wait reduces the
	// accepted gap by 3 seconds. Reduction floors at minAcceptedGap.
	impatienceDecayRate = 0.1

	// minAcceptedGap is the lower bound on the accepted gap regardless
	// of wait time. Prevents impatience from producing physically
	// unsafe gaps. 1.5s is on the aggressive end of normal human gap
	// acceptance.
	minAcceptedGap = 1.5
)

const (
	stuckSpeedThresh = 0.1
	// ... existing constants ...
)
```

Do not modify the existing constants. Preserve their exact comments.

- [ ] **Step 2: Build**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go build ./...`
Expected: exit 0. Unused constants don't cause warnings.

- [ ] **Step 3: Commit**

```
git add internal/sim/world.go
git commit -m "feat(sim): add Phase 3 impatience constants"
```

---

## Task 3: Add `effectiveGap` helper

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world.go`

- [ ] **Step 1: Add the helper**

In `internal/sim/world.go`, add this function. Place it just before `yieldGapCheck` (which is around line 380 after Phase 4):

```go
// effectiveGap returns the gap (in seconds of oncoming-ETA) that v
// will accept for a maneuver whose base critical gap is baseGap.
// Applies the per-driver GapFactor multiplier and shrinks linearly
// with WaitTime, floored at minAcceptedGap.
//
// A zero GapFactor on a hand-built test vehicle is treated as 1.0 so
// existing fixtures don't have to be updated.
func effectiveGap(v *Vehicle, baseGap float64) float64 {
	factor := v.GapFactor
	if factor == 0 {
		factor = 1.0
	}
	g := baseGap*factor - impatienceDecayRate*v.WaitTime
	if g < minAcceptedGap {
		g = minAcceptedGap
	}
	return g
}
```

- [ ] **Step 2: Build**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go build ./...`
Expected: exit 0. Unused function compiles fine.

- [ ] **Step 3: Run sim tests**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/`
Expected: PASS. The function exists but no call site uses it yet.

- [ ] **Step 4: Commit**

```
git add internal/sim/world.go
git commit -m "feat(sim): add effectiveGap helper"
```

---

## Task 4: Wire `effectiveGap` into `yieldGapCheck` and `leftTurnYieldsToOpposing`

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world.go`

This task changes behavior. Two single-line changes.

- [ ] **Step 1: Update `yieldGapCheck`**

In `internal/sim/world.go`, find the existing gap check inside `yieldGapCheck`. After Phase 4 it looks like:

```go
		for _, oi := range others {
			ov := &w.Vehicles[oi]
			d := otherEdge.Length - ov.S
			ovV := ov.V
			if ovV < 0.5 {
				ovV = 0.5
			}
			if d/ovV < gapThresholdSec {
				return myDist, true
			}
		}
```

Change the comparison to use `effectiveGap`:

```go
		for _, oi := range others {
			ov := &w.Vehicles[oi]
			d := otherEdge.Length - ov.S
			ovV := ov.V
			if ovV < 0.5 {
				ovV = 0.5
			}
			if d/ovV < effectiveGap(v, gapThresholdSec) {
				return myDist, true
			}
		}
```

- [ ] **Step 2: Update `leftTurnYieldsToOpposing`**

Find the existing gap check inside `leftTurnYieldsToOpposing`. After Phase 4 it looks like:

```go
		d := oppEdge.Length - ov.S
		ovV := ov.V
		if ovV < 0.5 {
			ovV = 0.5
		}
		if d/ovV < leftTurnGapSec {
			return myDist, true
		}
```

Change the comparison to use `effectiveGap`:

```go
		d := oppEdge.Length - ov.S
		ovV := ov.V
		if ovV < 0.5 {
			ovV = 0.5
		}
		if d/ovV < effectiveGap(v, leftTurnGapSec) {
			return myDist, true
		}
```

- [ ] **Step 3: Build**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go build ./...`
Expected: exit 0.

- [ ] **Step 4: Run sim tests**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/`
Expected: behavior on existing tests is essentially unchanged because all hand-built test vehicles have `GapFactor=0` (→ treated as 1.0) and `WaitTime=0` (no impatience accumulated yet — `WaitTime` updates happen in Task 6). So `effectiveGap(v, baseGap) == baseGap` until later tasks.

If any existing test fails at this point, it's a sign the gap math is wrong — investigate before proceeding.

- [ ] **Step 5: Commit**

```
git add internal/sim/world.go
git commit -m "feat(sim): route gap checks through effectiveGap"
```

---

## Task 5: Sample `GapFactor` at spawn

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world.go`
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world_test.go`

TDD: add a heterogeneity test first to verify the sampling distribution, then wire the sampling.

- [ ] **Step 1: Write the failing test**

Append to `internal/sim/world_test.go`:

```go
// TestWorld_GapFactor_Heterogeneous: spawn 200 vehicles via trySpawn
// with a fixed-seed world. Verify GapFactor distribution: mean within
// [0.95, 1.05], std within [0.07, 0.13], all values within [0.8, 1.2].
// Determinism: same seed → bit-identical values.
func TestWorld_GapFactor_Heterogeneous(t *testing.T) {
	net := build2x2Grid()
	w := NewWorld(net, NewRandomOD(net, 7, 100), nil) // high rate so spawns succeed

	// Drive 200 spawns by repeatedly calling trySpawn with synthetic
	// requests. Use a fixed seed via the world's existing rng.
	factors := make([]float64, 0, 200)
	for i := 0; i < 1000 && len(factors) < 200; i++ {
		w.trySpawn(SpawnRequest{OriginNode: 0, DestNode: 3})
		if len(w.Vehicles) > len(factors) {
			factors = append(factors, w.Vehicles[len(factors)].GapFactor)
		}
	}
	if len(factors) < 200 {
		t.Fatalf("could not collect 200 spawned vehicles, got %d", len(factors))
	}

	// Bounds.
	for i, f := range factors {
		if f < 0.8 || f > 1.2 {
			t.Errorf("factor[%d] = %f, out of [0.8, 1.2]", i, f)
		}
	}

	// Mean.
	sum := 0.0
	for _, f := range factors {
		sum += f
	}
	mean := sum / float64(len(factors))
	if mean < 0.95 || mean > 1.05 {
		t.Errorf("mean = %f, want in [0.95, 1.05]", mean)
	}

	// Std.
	varSum := 0.0
	for _, f := range factors {
		varSum += (f - mean) * (f - mean)
	}
	std := math.Sqrt(varSum / float64(len(factors)))
	if std < 0.07 || std > 0.13 {
		t.Errorf("std = %f, want in [0.07, 0.13]", std)
	}
}
```

Note: this test needs `"math"` import. The file already imports several stdlib packages; verify `math` is among them. If not, add it.

- [ ] **Step 2: Run it — expect failure**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -run TestWorld_GapFactor_Heterogeneous -v -count=1`
Expected: FAIL. Without sampling, all vehicles get `GapFactor=0`. The bounds check fires on every vehicle (`0 < 0.8`).

- [ ] **Step 3: Sample `GapFactor` in `trySpawn`**

In `internal/sim/world.go`, find `trySpawn`. After Phase 1+ it samples `SpeedFactor`:

```go
	factor := 1.0 + w.rng.NormFloat64()*speedFactorStdDev
	if factor < speedFactorMin {
		factor = speedFactorMin
	} else if factor > speedFactorMax {
		factor = speedFactorMax
	}

	// Spawn at this driver's cruising speed (factor * edge limit) so they
	// don't immediately decelerate. IDM regulates from there.
	v := Vehicle{
		ID:          w.nextID,
		Route:       route,
		Edge:        route[0],
		Lane:        0,
		S:           0,
		V:           w.Net.Edges[route[0]].SpeedLimit * factor,
		SpeedFactor: factor,
	}
```

Add a parallel `gapFactor` sample right after the `SpeedFactor` sample, and set it on the Vehicle literal:

```go
	factor := 1.0 + w.rng.NormFloat64()*speedFactorStdDev
	if factor < speedFactorMin {
		factor = speedFactorMin
	} else if factor > speedFactorMax {
		factor = speedFactorMax
	}

	gapFactor := 1.0 + w.rng.NormFloat64()*gapFactorStdDev
	if gapFactor < gapFactorMin {
		gapFactor = gapFactorMin
	} else if gapFactor > gapFactorMax {
		gapFactor = gapFactorMax
	}

	// Spawn at this driver's cruising speed (factor * edge limit) so they
	// don't immediately decelerate. IDM regulates from there.
	v := Vehicle{
		ID:          w.nextID,
		Route:       route,
		Edge:        route[0],
		Lane:        0,
		S:           0,
		V:           w.Net.Edges[route[0]].SpeedLimit * factor,
		SpeedFactor: factor,
		GapFactor:   gapFactor,
	}
```

- [ ] **Step 4: Run the test**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -run TestWorld_GapFactor_Heterogeneous -v -count=1`
Expected: PASS.

- [ ] **Step 5: Run determinism + full sim suite**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -run TestWorld_TraceDeterminism -v -count=1`
Expected: PASS. The new `gapFactor` sample uses `w.rng.NormFloat64()` (same source as SpeedFactor); same seed → same sequence.

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/`
Expected: all PASS.

- [ ] **Step 6: Commit**

```
git add internal/sim/world.go internal/sim/world_test.go
git commit -m "feat(sim): sample per-driver GapFactor at spawn"
```

---

## Task 6: Update `WaitTime` in `Step`

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world.go`

- [ ] **Step 1: Add the update logic**

In `internal/sim/world.go`, find the per-vehicle stepping loop in `Step`. After Phase 4 the relevant section looks like this (around line 695-710 — the stuck-vehicle guard block, which we cache `isRed`, `mustYield`, `mustYieldLT` above):

```go
		v0 := w.computeDesiredSpeed(v)
		stepIDM(v, v0, lS, lV, has, w.Net, DefaultIDM(), w.dt)

		// Stuck-vehicle guard. Defensive against sim bugs that would
		// otherwise leave a vehicle wedged forever. Runs only when the
		// vehicle is below the speed threshold; reuses the yield-check
		// results from above to avoid re-calling stopDistance helpers.
		if !v.Despawned && v.V < stuckSpeedThresh {
			if !isRed && !mustYield && !mustYieldLT {
				v.StuckTime += w.dt
				if v.StuckTime > stuckTimeoutSec {
```

Add the WaitTime update **immediately after** `stepIDM` and **before** the stuck-guard block:

```go
		v0 := w.computeDesiredSpeed(v)
		stepIDM(v, v0, lS, lV, has, w.Net, DefaultIDM(), w.dt)

		// Update WaitTime: accumulates while the vehicle is effectively
		// stopped AND yielding via gap-acceptance. Resets the moment
		// either condition stops being true. Drives the impatience curve
		// in effectiveGap; does NOT apply to red lights.
		if v.V < stuckSpeedThresh && (mustYield || mustYieldLT) {
			v.WaitTime += w.dt
		} else {
			v.WaitTime = 0
		}

		// Stuck-vehicle guard. Defensive against sim bugs that would
		// otherwise leave a vehicle wedged forever. Runs only when the
		// vehicle is below the speed threshold; reuses the yield-check
		// results from above to avoid re-calling stopDistance helpers.
		if !v.Despawned && v.V < stuckSpeedThresh {
			...
		}
```

- [ ] **Step 2: Build**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go build ./...`
Expected: exit 0.

- [ ] **Step 3: Run sim tests**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/`
Expected: behavior changes for tests that yield long enough for impatience to bite. Some existing tests will likely FAIL at this step. That's expected — Task 7 fixes them.

- [ ] **Step 4: Commit (even if some tests fail)**

```
git add internal/sim/world.go
git commit -m "feat(sim): accumulate WaitTime during gap-acceptance yields"
```

(Tests will be fixed in Task 7; we commit the WaitTime-update change separately for clean history.)

---

## Task 7: Update existing tests that flake under impatience

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world_test.go`

Four existing tests pin a cross-traffic vehicle at ETA = 2-4s, expecting the yielder to wait indefinitely. With impatience, after ~5-25s of wait, the accepted gap shrinks below the pinned ETA and the yielder proceeds — breaking the test's intent.

Fix: push each pinned cross-traffic vehicle so its ETA stays below `minAcceptedGap = 1.5s` perpetually. The impatience floor then guarantees the yielder waits forever, restoring the test's "vehicle yields indefinitely" semantics.

- [ ] **Step 1: Identify which tests are failing**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -count=1 2>&1 | grep -E "FAIL|---"`

Expected failures (these are the four candidates from the spec): `TestWorld_StuckAtYieldNotDespawned`, `TestWorld_LeftTurn_StuckGuardBypassed`, `TestWorld_StopSign_GapAcceptance`, `TestWorld_LeftTurn_AllWayStop_YieldsToOpposing`. Two additional tests (`TestWorld_LeftTurn_PriorityRoad_YieldsToOpposing`, `TestWorld_LeftTurn_SignaledGreen_YieldsToOpposing`) are marginal and may also need pinning fixes.

- [ ] **Step 2: Fix `TestWorld_StuckAtYieldNotDespawned`**

Find this block in the test:

```go
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 99, V: 0.5}, // priority, ~1m out @ 0.5 m/s = 2s ETA
		{ID: 2, Route: []network.EdgeID{1, 2}, Edge: 1, S: 50, V: 10},  // yield, approaching
	}
```

And the inner pin loop:

```go
		// Find the priority vehicle by ID and re-pin its state.
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 1 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 99
				w.Vehicles[j].V = 0.5
			}
		}
```

Change the priority pin to push ETA below `minAcceptedGap=1.5s`. With d=1m, V=1.0 → ETA=1.0s:

```go
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 99, V: 1.0}, // priority, ~1m out @ 1.0 m/s = 1s ETA (below minAcceptedGap)
		{ID: 2, Route: []network.EdgeID{1, 2}, Edge: 1, S: 50, V: 10},  // yield, approaching
	}
```

And the pin loop:

```go
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 1 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 99
				w.Vehicles[j].V = 1.0
			}
		}
```

- [ ] **Step 3: Fix `TestWorld_StopSign_GapAcceptance`**

Find the priority vehicle:

```go
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 99, V: 0.5},
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 50, V: 8},
```

And the pin loop:

```go
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 1 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 99
				w.Vehicles[j].V = 0.5
			}
		}
```

Change to V=1.0 (ETA=1s, below floor):

```go
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 99, V: 1.0},
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 50, V: 8},
```

And:

```go
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 1 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 99
				w.Vehicles[j].V = 1.0
			}
		}
```

- [ ] **Step 4: Fix `TestWorld_LeftTurn_StuckGuardBypassed`**

Find Vehicle 2 (the opposing vehicle):

```go
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 98, V: 0.5},
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 98, V: 0.5},
	}
```

And the pin loop:

```go
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 2 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 98
				w.Vehicles[j].V = 0.5
			}
		}
```

With S=98, d=2m. For ETA < 1.5s need V > 2/1.5 = 1.34 m/s. Use V=1.5 → ETA=1.33s:

```go
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 98, V: 0.5}, // A (left turner)
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 98, V: 1.5}, // B (opposing, ETA=1.33s < minAcceptedGap)
	}
```

And:

```go
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 2 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 98
				w.Vehicles[j].V = 1.5
			}
		}
```

- [ ] **Step 5: Fix `TestWorld_LeftTurn_AllWayStop_YieldsToOpposing`**

Find Vehicle 2 (the opposing through):

```go
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 80, V: 5},
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 98, V: 0.5},
	}
```

And the pin loop:

```go
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 2 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 98
				w.Vehicles[j].V = 0.5
			}
		}
```

Push to V=1.5 (ETA=1.33s):

```go
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 80, V: 5},
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 98, V: 1.5},
	}
```

And:

```go
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 2 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 98
				w.Vehicles[j].V = 1.5
			}
		}
```

- [ ] **Step 6: Also tighten `TestWorld_LeftTurn_PriorityRoad_YieldsToOpposing` (defensive)**

This test is marginal (15s budget, gap drops to 4.5s, ETA=4s — should pass but margins are thin). Tighten the pin to be robust. Vehicle 2 (opposing):

```go
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 98, V: 0.5},
```

And the pin loop sets `V=0.5`. Change to V=1.5 (ETA=1.33s) so the test is bulletproof.

- [ ] **Step 7: Also tighten `TestWorld_LeftTurn_SignaledGreen_YieldsToOpposing` (defensive)**

Same defensive tightening. Change Vehicle 2's pin from V=0.5 to V=1.5.

- [ ] **Step 8: Run the full sim suite**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/`
Expected: all PASS.

If any test still fails, investigate before assuming the fix is wrong. The most likely cause is that the test relies on a different pinning pattern than the four spec'd ones — track the failure to a specific assertion and fix the pin accordingly.

- [ ] **Step 9: Commit**

```
git add internal/sim/world_test.go
git commit -m "test(sim): pin cross-traffic ETA below minAcceptedGap to survive impatience"
```

---

## Task 8: Test impatience shrinks straight-crossing gap

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world_test.go`

- [ ] **Step 1: Append the test**

```go
// TestWorld_Impatience_StraightCrossingShrinksGap: a yield-controlled
// vehicle facing perpetual cross-traffic with ETA=2.5s yields at t=0
// (base gap 3s > 2.5s ETA → yield). After ~5s wait, effectiveGap
// drops to 2.5s and vehicle accepts the gap. Test verifies the wait
// duration and the eventual departure.
func TestWorld_Impatience_StraightCrossingShrinksGap(t *testing.T) {
	// Same fixture as TestWorld_StuckAtYieldNotDespawned: 4-way with W
	// priority and S yield. Vehicle on S yields; vehicle on W pinned
	// at perpetual ETA=2.5s.
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -100, Y: 0}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},
	}
	mkEdge := func(id, from, to int) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	edges := []network.Edge{
		mkEdge(0, 0, 2),
		mkEdge(1, 1, 2),
		mkEdge(2, 2, 3),
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 2,
			Incoming:        []network.EdgeID{0, 1},
			IncomingControl: ctrls(network.ControlNone, network.ControlYield),
			Outgoing:        []network.EdgeID{2},
			HasSignal:       false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// Priority at S=98, V=0.8 → d=2m, ETA=2.5s (above floor 1.5s,
	// below base gap 3s).
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 98, V: 0.8},
		{ID: 2, Route: []network.EdgeID{1, 2}, Edge: 1, S: 99, V: 0.5}, // yield vehicle, near line
	}
	w.nextID = 3

	// Yield vehicle starts close to the line so it stops quickly. Pin
	// priority each tick.
	var crossedAt float64 = -1.0
	for i := 0; i < 600; i++ {
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 1 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 98
				w.Vehicles[j].V = 0.8
			}
		}
		w.Step()
		// Detect when the yield vehicle crosses into outbound.
		for j := range w.Vehicles {
			v := &w.Vehicles[j]
			if v.ID == 2 && !v.Despawned && v.Edge == 2 && crossedAt < 0 {
				crossedAt = w.SimTime
			}
		}
		if crossedAt > 0 {
			break
		}
	}

	if crossedAt < 0 {
		t.Fatal("yield vehicle never crossed; impatience never reduced gap below ETA")
	}
	// Predicted wait: gap needs to drop from 3.0 to 2.5 = 0.5s reduction.
	// At decay 0.1 s/s, that's 5s of wait. Plus a few seconds of approach
	// + stop. Expect crossing somewhere in [5, 15] sim-seconds.
	if crossedAt < 4 || crossedAt > 20 {
		t.Errorf("expected crossing in [4, 20] sim-seconds, got %.2f", crossedAt)
	}
}
```

- [ ] **Step 2: Run it**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -run TestWorld_Impatience_StraightCrossingShrinksGap -v -count=1`
Expected: PASS.

- [ ] **Step 3: Commit**

```
git add internal/sim/world_test.go
git commit -m "test(sim): impatience shrinks straight-crossing gap"
```

---

## Task 9: Test impatience shrinks left-turn gap

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world_test.go`

- [ ] **Step 1: Append the test**

```go
// TestWorld_Impatience_LeftTurnShrinksGap: priority-road left turner
// with perpetual opposing through at ETA=3.5s. Base 6s × 1.0 = 6s.
// To drop to 3.5s: 6 - 0.1*t = 3.5 → t = 25s. Vehicle waits ~25s
// then accepts.
func TestWorld_Impatience_LeftTurnShrinksGap(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},  // E destination (A's left)
		{ID: 4, Pos: network.Point{X: 0, Y: 200}},  // N destination (B's through)
	}
	mkEdge := func(id, from, to int) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	edges := []network.Edge{
		mkEdge(0, 0, 2),
		mkEdge(1, 1, 2),
		mkEdge(2, 2, 3), // C->E (A's left turn)
		mkEdge(3, 2, 4), // C->N (B's through)
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 2,
			Incoming:        []network.EdgeID{0, 1},
			IncomingControl: ctrls(network.ControlNone, network.ControlNone),
			Opposing:        []int8{1, 0},
			Outgoing:        []network.EdgeID{2, 3},
			HasSignal:       false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// A: left turner, near line. B: opposing through, pinned at ETA=3.5s
	// (d=2m, V=2/3.5 ≈ 0.571 m/s).
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 99, V: 0.5},
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 98, V: 0.5714},
	}
	w.nextID = 3

	var crossedAt float64 = -1.0
	for i := 0; i < 1200; i++ {
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 2 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 98
				w.Vehicles[j].V = 0.5714
			}
		}
		w.Step()
		for j := range w.Vehicles {
			v := &w.Vehicles[j]
			if v.ID == 1 && !v.Despawned && v.Edge == 2 && crossedAt < 0 {
				crossedAt = w.SimTime
			}
		}
		if crossedAt > 0 {
			break
		}
	}

	if crossedAt < 0 {
		t.Fatal("left turner never crossed; impatience never reduced gap below ETA")
	}
	// Predicted wait: gap drops from 6 to 3.5 = 2.5s reduction → 25s of
	// wait. Plus a few seconds of approach. Expect crossing in [20, 40]
	// sim-seconds.
	if crossedAt < 20 || crossedAt > 40 {
		t.Errorf("expected crossing in [20, 40] sim-seconds, got %.2f", crossedAt)
	}
}
```

- [ ] **Step 2: Run it**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -run TestWorld_Impatience_LeftTurnShrinksGap -v -count=1`
Expected: PASS.

- [ ] **Step 3: Commit**

```
git add internal/sim/world_test.go
git commit -m "test(sim): impatience shrinks left-turn gap"
```

---

## Task 10: Test floor prevents collision

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world_test.go`

- [ ] **Step 1: Append the test**

```go
// TestWorld_Impatience_FloorPreventsCollision: oncoming ETA = 1.0s
// (below minAcceptedGap=1.5s). Vehicle waits forever — the floor
// prevents impatience from producing collision-imminent crossings.
func TestWorld_Impatience_FloorPreventsCollision(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},
		{ID: 4, Pos: network.Point{X: 0, Y: 200}},
	}
	mkEdge := func(id, from, to int) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	edges := []network.Edge{
		mkEdge(0, 0, 2),
		mkEdge(1, 1, 2),
		mkEdge(2, 2, 3),
		mkEdge(3, 2, 4),
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 2,
			Incoming:        []network.EdgeID{0, 1},
			IncomingControl: ctrls(network.ControlNone, network.ControlNone),
			Opposing:        []int8{1, 0},
			Outgoing:        []network.EdgeID{2, 3},
			HasSignal:       false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// A: left turner. B: opposing, pinned at ETA=1.0s (d=2m, V=2 m/s).
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 99, V: 0.5},
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 98, V: 2.0},
	}
	w.nextID = 3

	// 200s sim = 4000 ticks.
	for i := 0; i < 4000; i++ {
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 2 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 98
				w.Vehicles[j].V = 2.0
			}
		}
		w.Step()
	}

	var a *Vehicle
	for j := range w.Vehicles {
		if w.Vehicles[j].ID == 1 {
			a = &w.Vehicles[j]
		}
	}
	if a == nil || a.Despawned {
		t.Fatal("left turner should not be despawned during legitimate yield")
	}
	if a.Edge != 0 {
		t.Errorf("left turner should still be on approach edge 0 (floor prevented unsafe crossing), got edge %d", a.Edge)
	}
	if a.StuckTime != 0 {
		t.Errorf("StuckTime must remain 0, got %.3f", a.StuckTime)
	}
	// WaitTime should be substantial (impatience accumulated) but vehicle
	// still hasn't crossed.
	if a.WaitTime < 100 {
		t.Errorf("WaitTime should reflect substantial wait, got %.3f", a.WaitTime)
	}
}
```

- [ ] **Step 2: Run it**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -run TestWorld_Impatience_FloorPreventsCollision -v -count=1`
Expected: PASS.

- [ ] **Step 3: Commit**

```
git add internal/sim/world_test.go
git commit -m "test(sim): impatience floor prevents unsafe crossings"
```

---

## Task 11: Test WaitTime resets on movement and edge transition

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world_test.go`

Two related tests — bundle into one task.

- [ ] **Step 1: Append both tests**

```go
// TestWorld_Impatience_ResetsOnMovement: vehicle yields, accumulates
// WaitTime, gap clears, vehicle accelerates past stuckSpeedThresh.
// Verify WaitTime resets to 0 immediately.
func TestWorld_Impatience_ResetsOnMovement(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -100, Y: 0}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},
	}
	mkEdge := func(id, from, to int) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	edges := []network.Edge{
		mkEdge(0, 0, 2),
		mkEdge(1, 1, 2),
		mkEdge(2, 2, 3),
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 2,
			Incoming:        []network.EdgeID{0, 1},
			IncomingControl: ctrls(network.ControlNone, network.ControlYield),
			Outgoing:        []network.EdgeID{2},
			HasSignal:       false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// Yield vehicle near line, priority near line at imminent ETA.
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 99, V: 0.5},
		{ID: 2, Route: []network.EdgeID{1, 2}, Edge: 1, S: 99, V: 0.5},
	}
	w.nextID = 3

	// Run until yield vehicle has accumulated some WaitTime.
	for i := 0; i < 200; i++ {
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 1 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 99
				w.Vehicles[j].V = 0.5
			}
		}
		w.Step()
		var v2 *Vehicle
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 2 {
				v2 = &w.Vehicles[j]
			}
		}
		if v2 != nil && v2.WaitTime > 3.0 {
			break
		}
	}

	var yieldV *Vehicle
	for j := range w.Vehicles {
		if w.Vehicles[j].ID == 2 {
			yieldV = &w.Vehicles[j]
		}
	}
	if yieldV == nil {
		t.Fatal("yield vehicle disappeared")
	}
	if yieldV.WaitTime < 3.0 {
		t.Fatalf("setup failed: yield vehicle didn't accumulate WaitTime, got %.3f", yieldV.WaitTime)
	}

	// Now remove the priority vehicle so the yielder can accelerate.
	for j := range w.Vehicles {
		if w.Vehicles[j].ID == 1 {
			w.Vehicles[j].Despawned = true
		}
	}

	// Run until the yield vehicle starts moving above stuckSpeedThresh.
	for i := 0; i < 50; i++ {
		w.Step()
		yieldV = nil
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 2 && !w.Vehicles[j].Despawned {
				yieldV = &w.Vehicles[j]
			}
		}
		if yieldV != nil && yieldV.V > stuckSpeedThresh*2 {
			break
		}
	}

	if yieldV == nil {
		t.Fatal("yield vehicle was unexpectedly removed")
	}
	if yieldV.V <= stuckSpeedThresh {
		t.Fatalf("yield vehicle should be moving, V=%.3f", yieldV.V)
	}
	if yieldV.WaitTime != 0 {
		t.Errorf("WaitTime should reset on movement, got %.3f", yieldV.WaitTime)
	}
}

// TestWorld_Impatience_ResetsOnEdgeTransition: vehicle yields at
// intersection A, accumulates WaitTime, eventually crosses. WaitTime
// must be 0 the moment the vehicle reaches the outbound edge.
func TestWorld_Impatience_ResetsOnEdgeTransition(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -100, Y: 0}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},
	}
	mkEdge := func(id, from, to int) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	edges := []network.Edge{
		mkEdge(0, 0, 2),
		mkEdge(1, 1, 2),
		mkEdge(2, 2, 3),
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 2,
			Incoming:        []network.EdgeID{0, 1},
			IncomingControl: ctrls(network.ControlNone, network.ControlYield),
			Outgoing:        []network.EdgeID{2},
			HasSignal:       false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 99, V: 0.5}, // priority at ETA=2s
		{ID: 2, Route: []network.EdgeID{1, 2}, Edge: 1, S: 99, V: 0.5}, // yield, near line
	}
	w.nextID = 3

	// Pin priority until yield vehicle has noticeable WaitTime, then
	// remove priority and let yielder cross.
	pinTicks := 0
	for i := 0; i < 600; i++ {
		if pinTicks < 200 {
			for j := range w.Vehicles {
				if w.Vehicles[j].ID == 1 && !w.Vehicles[j].Despawned {
					w.Vehicles[j].S = 99
					w.Vehicles[j].V = 0.5
				}
			}
			pinTicks++
		} else {
			// Remove priority vehicle so yielder can cross.
			for j := range w.Vehicles {
				if w.Vehicles[j].ID == 1 {
					w.Vehicles[j].Despawned = true
				}
			}
		}
		w.Step()

		// Check: once yield vehicle is on outbound edge, WaitTime must be 0.
		for j := range w.Vehicles {
			v := &w.Vehicles[j]
			if v.ID == 2 && !v.Despawned && v.Edge == 2 {
				if v.WaitTime != 0 {
					t.Fatalf("WaitTime should be 0 after edge transition, got %.3f", v.WaitTime)
				}
				return
			}
		}
	}

	t.Fatal("yield vehicle never crossed; cannot verify edge-transition reset")
}
```

- [ ] **Step 2: Run them**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -run "TestWorld_Impatience_ResetsOnMovement|TestWorld_Impatience_ResetsOnEdgeTransition" -v -count=1`
Expected: PASS.

- [ ] **Step 3: Commit**

```
git add internal/sim/world_test.go
git commit -m "test(sim): WaitTime resets on movement and edge transition"
```

---

## Task 12: Test impatience is not applied to red lights

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world_test.go`

- [ ] **Step 1: Append the test**

```go
// TestWorld_Impatience_NotAppliedToRedLight: vehicle stopped at a red
// light for 60s. WaitTime must remain 0 throughout — red lights are
// hard stops, not gap-acceptance yields.
func TestWorld_Impatience_NotAppliedToRedLight(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 10,
			Lanes: []network.Lane{{Index: 0}}},
	}
	xs := []network.Intersection{
		{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0}, HasSignal: true},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// Permanent all-red phase.
	w.SignalStates[0] = NewSignalState(SignalConfig{
		Phases: []SignalPhase{{GreenEdges: nil, GreenDur: 10000, YellowDur: 0}},
	})

	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0}, Edge: 0, S: 50, V: 10},
	}
	w.nextID = 2

	// Run 1200 ticks = 60 sim-seconds.
	for i := 0; i < 1200; i++ {
		w.Step()
	}

	if len(w.Vehicles) != 1 {
		t.Fatalf("vehicle should be stopped at red, got %d alive", len(w.Vehicles))
	}
	v := &w.Vehicles[0]
	if v.V > 0.1 {
		t.Errorf("vehicle should be stopped at red, V=%.2f", v.V)
	}
	if v.WaitTime != 0 {
		t.Errorf("WaitTime should remain 0 at red light (not a gap-acceptance yield), got %.3f", v.WaitTime)
	}
}
```

- [ ] **Step 2: Run it**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -run TestWorld_Impatience_NotAppliedToRedLight -v -count=1`
Expected: PASS.

- [ ] **Step 3: Commit**

```
git add internal/sim/world_test.go
git commit -m "test(sim): WaitTime does not accumulate at red lights"
```

---

## Task 13: Final verification

**Files:** none modified.

- [ ] **Step 1: Full repo suite**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./...`
Expected: all PASS.

- [ ] **Step 2: Determinism**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -run TestWorld_TraceDeterminism -v -count=1`
Expected: PASS. Verify the new `gapFactor` sample didn't introduce nondeterminism.

- [ ] **Step 3: Benchmarks**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -bench=BenchmarkTick -benchtime=2s -run=^$`
Expected: ns/op within ~15% of post-Phase-4 baseline (1k=0.54ms, 5k=1.86ms, 10k=3.31ms). The new code adds one `effectiveGap` call per gap-check iteration; impact should be minimal.

If significantly slower, profile with `-cpuprofile=cpu.out` to identify the regression.

- [ ] **Step 4: No commit (verification only)**

Phase 3 is complete. Per-driver `GapFactor` heterogeneity + linear-decay impatience now drive gap-acceptance decisions.

---

## Out of scope (deferred to later phases)

- Per-maneuver gap factors (separate `StraightGapFactor` and `LeftGapFactor`).
- Non-linear impatience curves (exponential, piecewise step).
- Honking / peer pressure on stalled cross-traffic.
- Red-light running.
- Driver fatigue (impatience persisting across intersections).
- Configurable per-intersection patience overrides.
