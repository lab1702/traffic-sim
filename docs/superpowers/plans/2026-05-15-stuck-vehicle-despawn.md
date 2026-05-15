# Stuck-Vehicle Despawn Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a defensive guard that despawns vehicles which have been stationary (V < 0.1 m/s) for >60 sim-seconds *and* are not legitimately waiting at a red light or yield, logging full state at WARN level.

**Architecture:** One new `StuckTime float64` field on `Vehicle` (mirrors `LaneChangeCooldown`). Two new constants in `internal/sim/world.go`. ~20 lines added inside the existing per-vehicle loop in `World.Step()`, immediately after `stepIDM` and before the existing despawn trace emission — so a stuck-despawn flows through the same `VehicleDespawn` trace event as a normal end-of-route despawn. Three new unit tests in `internal/sim/world_test.go`.

**Tech Stack:** Go 1.22+, `log/slog` (stdlib), existing `internal/sim` and `internal/trace` packages. No new dependencies.

**Spec:** [`../specs/2026-05-15-stuck-vehicle-despawn-design.md`](../specs/2026-05-15-stuck-vehicle-despawn-design.md)

---

## File Structure

| File | Change | Responsibility |
|---|---|---|
| `internal/sim/vehicle.go` | Modify | Add `StuckTime float64` field to `Vehicle` struct. |
| `internal/sim/world.go` | Modify | Add `stuckSpeedThresh` and `stuckTimeoutSec` constants. Add stuck-check block in `Step()` per-vehicle loop. |
| `internal/sim/world_test.go` | Modify | Add 3 new tests: stuck-vehicle-despawned, stuck-at-red-not-despawned, stuck-at-yield-not-despawned. |

No new files. No changes to `internal/trace/`, `internal/render/`, `internal/config/`, or any other package.

---

## Task 1: Add failing test for stuck-vehicle despawn

**Files:**
- Modify: `internal/sim/world_test.go` (append new test)

This task writes the test that drives the implementation. The test must fail until Task 2 lands the despawn logic.

The construction: a single 200m edge with no intersection at its end (so `stopDistanceForRed` and `stopDistanceForYield` both return `false`). Inject one vehicle on the edge. Before each `Step()`, force `v.V = 0`. After each `Step()`, IDM will accelerate from 0 to ~0.05 m/s (one tick of free-acceleration with `A=1.0, dt=0.05`) — still below the 0.1 m/s stuck threshold. So `StuckTime` accumulates `dt` per tick. After >1200 ticks (60 sim-seconds), the despawn condition triggers and the vehicle is removed.

For log capture, swap `slog.Default()` for the duration of the test with a handler that writes JSON to a `bytes.Buffer`, then restore.

- [ ] **Step 1: Write the failing test**

Append to `internal/sim/world_test.go`:

```go
// TestWorld_StuckVehicleDespawned: a vehicle below the stuck speed threshold
// for >60 sim-seconds on an edge with no red light and no yield must be
// logged at WARN level and despawned.
func TestWorld_StuckVehicleDespawned(t *testing.T) {
	// Single 200m edge, no intersection at the end. With no intersection,
	// stopDistanceForRed and stopDistanceForYield both return false, so the
	// stuck condition can trigger.
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 10,
			Lanes: []network.Lane{{Index: 0}}},
	}
	net := &network.Network{Nodes: nodes, Edges: edges}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0}, Edge: 0, S: 10, V: 0}}
	w.nextID = 2

	// Capture WARN logs via slog handler swap.
	var logBuf bytes.Buffer
	prev := slog.Default()
	slog.SetDefault(slog.New(slog.NewJSONHandler(&logBuf, &slog.HandlerOptions{Level: slog.LevelWarn})))
	defer slog.SetDefault(prev)

	// Pin V=0 before each tick. After stepIDM the vehicle's V will be
	// ~0.05 (one tick of free-acceleration from 0), still below the 0.1
	// threshold, so StuckTime accumulates dt per tick. >1200 ticks = >60
	// sim-seconds → despawn.
	for i := 0; i < 1500; i++ {
		if len(w.Vehicles) > 0 && !w.Vehicles[0].Despawned {
			w.Vehicles[0].V = 0
		}
		w.Step()
		if len(w.Vehicles) == 0 {
			break
		}
	}

	if len(w.Vehicles) != 0 {
		t.Fatalf("stuck vehicle should have been despawned, %d still alive", len(w.Vehicles))
	}
	if !strings.Contains(logBuf.String(), "stuck vehicle despawned") {
		t.Errorf("expected WARN log containing 'stuck vehicle despawned', got: %q", logBuf.String())
	}
}
```

Also add the required imports at the top of the file. The existing imports block is:

```go
import (
	"bytes"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/trace"
)
```

Extend it to:

```go
import (
	"bytes"
	"log/slog"
	"strings"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/trace"
)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./internal/sim/... -run TestWorld_StuckVehicleDespawned -count=1 -v`

Expected: FAIL with `stuck vehicle should have been despawned, 1 still alive` (the despawn logic does not yet exist).

- [ ] **Step 3: Commit the failing test**

```bash
git add internal/sim/world_test.go
git commit -m "test(sim): failing test for stuck-vehicle despawn

Drives the implementation in the next commit. Single-edge network with
no intersection so neither stopDistanceForRed nor stopDistanceForYield
can fire. V is pinned to 0 each tick; after 60+ sim-seconds the despawn
guard should remove the vehicle and emit a WARN log."
```

---

## Task 2: Implement the stuck-despawn feature

**Files:**
- Modify: `internal/sim/vehicle.go` (add `StuckTime` field)
- Modify: `internal/sim/world.go` (add constants + stuck-check block)

- [ ] **Step 1: Add `StuckTime` field to `Vehicle`**

In `internal/sim/vehicle.go`, change the `Vehicle` struct (currently lines 14-29) to add `StuckTime` right after `LaneChangeCooldown`. The full updated struct:

```go
type Vehicle struct {
	ID       VehicleID
	Route    []network.EdgeID
	RouteIdx int
	Edge     network.EdgeID
	Lane     uint8
	S        float64 // meters along edge, measured at front bumper
	V        float64 // m/s
	A        float64 // m/s^2 (last computed accel; useful for tracing)

	// LaneChangeCooldown counts down in seconds. Cannot change lanes
	// again until this reaches 0. Prevents oscillation.
	LaneChangeCooldown float64

	// StuckTime accumulates sim-seconds where V < stuckSpeedThresh and the
	// vehicle is not legitimately waiting at a red light or yield. Resets
	// to 0 whenever any of those conditions fails. When it exceeds
	// stuckTimeoutSec the vehicle is logged at WARN and despawned.
	StuckTime float64

	Despawned bool
}
```

- [ ] **Step 2: Add the two constants in `internal/sim/world.go`**

In `internal/sim/world.go`, find the existing `gapThresholdSec` constant (line 136):

```go
const gapThresholdSec = 3.0
```

Add two new constants immediately below it:

```go
const gapThresholdSec = 3.0

const (
	// stuckSpeedThresh is the speed (m/s) below which a vehicle is
	// considered "not moving" for the purposes of the stuck-despawn guard.
	stuckSpeedThresh = 0.1
	// stuckTimeoutSec is the accumulated sim-seconds of below-threshold
	// motion (with no legitimate red/yield reason) that triggers despawn.
	stuckTimeoutSec = 60.0
)
```

- [ ] **Step 3: Add the stuck-check block inside `Step()`**

In `internal/sim/world.go`, locate the per-vehicle loop block currently at lines 370-374:

```go
		v0 := w.computeDesiredSpeed(v)
		stepIDM(v, v0, lS, lV, has, w.Net, DefaultIDM(), w.dt)
		if v.Despawned {
			w.EmitTrace(w.Tick, w.SimTime, &trace.VehicleDespawn{VehicleID: uint32(v.ID)})
		}
```

Replace that block with:

```go
		v0 := w.computeDesiredSpeed(v)
		stepIDM(v, v0, lS, lV, has, w.Net, DefaultIDM(), w.dt)

		// Stuck-vehicle guard. Defensive against sim bugs that would
		// otherwise leave a vehicle wedged forever. Runs only when the
		// vehicle is below the speed threshold; the two stopDistance
		// helpers are cheap but skipped for the common moving case.
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

		if v.Despawned {
			w.EmitTrace(w.Tick, w.SimTime, &trace.VehicleDespawn{VehicleID: uint32(v.ID)})
		}
```

Two notes for the reader:

- The block lives *between* `stepIDM` and the existing `if v.Despawned { EmitTrace... }` check. That ordering is load-bearing: when the guard sets `v.Despawned = true`, the existing trace-emit block immediately fires `VehicleDespawn` for it — same as a normal end-of-route despawn. No new trace event kind.
- The `else { v.StuckTime = 0 }` branches matter: a vehicle that briefly idles must NOT accumulate stuck time forever. The 60-second budget is contiguous-stuck-time, not lifetime-stuck-time.

- [ ] **Step 4: Run the test from Task 1 to verify it passes**

Run: `go test ./internal/sim/... -run TestWorld_StuckVehicleDespawned -count=1 -v`

Expected: PASS.

- [ ] **Step 5: Run the full sim test suite to verify nothing else broke**

Run: `go test ./internal/sim/... -count=1`

Expected: PASS. All existing tests (including `TestWorld_TraceDeterminism`, `TestWorld_StopsAtRedLight`, etc.) should still pass. If `TestWorld_TraceDeterminism` fails, stop — the change introduced nondeterminism and must be diagnosed before continuing.

- [ ] **Step 6: Commit**

```bash
git add internal/sim/vehicle.go internal/sim/world.go
git commit -m "feat(sim): stuck-vehicle despawn guard

Vehicles below 0.1 m/s for >60 sim-seconds of accumulated tick time
that are not legitimately waiting at a red light or yield are logged
at WARN with full state and despawned. Defensive guard against sim
bugs surfacing in long research runs; reuses the existing
VehicleDespawn trace event so the trace format is unchanged."
```

---

## Task 3: Add test that legitimately stopped vehicles are NOT despawned (red light)

**Files:**
- Modify: `internal/sim/world_test.go` (append new test)

A vehicle correctly stopped at a red signal must keep its `StuckTime == 0` regardless of how long the red lasts. This test reuses the construction style of the existing `TestWorld_StopsAtRedLight` but runs long enough (>60 sim-seconds) that a missing guard branch would falsely despawn the vehicle.

- [ ] **Step 1: Write the test**

Append to `internal/sim/world_test.go`:

```go
// TestWorld_StuckAtRedNotDespawned: a vehicle stopped at a red light for
// longer than the stuck timeout must NOT be despawned, because
// stopDistanceForRed returning true is the "legitimately stopped" branch.
func TestWorld_StuckAtRedNotDespawned(t *testing.T) {
	// Same setup as TestWorld_StopsAtRedLight: single edge ending in a
	// signalized intersection forced all-red.
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
	// Force the signal all-red by giving it an empty-green phase that
	// outlasts the run.
	w.SignalStates[0] = NewSignalState(SignalConfig{
		Phases: []SignalPhase{{GreenEdges: nil, GreenDur: 1000, YellowDur: 0}},
	})

	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0}, Edge: 0, S: 50, V: 10}}
	w.nextID = 2

	// 1500 ticks = 75 sim-seconds, well past the 60-second stuck timeout.
	for i := 0; i < 1500; i++ {
		w.Step()
	}

	if len(w.Vehicles) != 1 {
		t.Fatalf("vehicle stopped at red should not be despawned, got %d alive", len(w.Vehicles))
	}
	v := &w.Vehicles[0]
	if v.StuckTime != 0 {
		t.Errorf("StuckTime should be 0 for a vehicle legitimately stopped at red, got %.3f", v.StuckTime)
	}
}
```

- [ ] **Step 2: Run the test to verify it passes**

Run: `go test ./internal/sim/... -run TestWorld_StuckAtRedNotDespawned -count=1 -v`

Expected: PASS. The implementation from Task 2 sets `StuckTime = 0` whenever `stopDistanceForRed` returns `true`, so the vehicle should stay parked at the stop line indefinitely.

- [ ] **Step 3: Commit**

```bash
git add internal/sim/world_test.go
git commit -m "test(sim): vehicle stopped at red is not despawned by stuck guard

Pairs with the despawn test: confirms the guard correctly distinguishes
'legitimately waiting at a red signal' from 'wedged by a sim bug'."
```

---

## Task 4: Add test that legitimately yielding vehicles are NOT despawned

**Files:**
- Modify: `internal/sim/world_test.go` (append new test)

Mirror of Task 3 for the unsignalized-yield branch. Build an unsignalized intersection with two incoming approaches; the lower-indexed approach has priority. Place a yielding vehicle close to the stop line on the yield approach, and a priority vehicle parked close to the intersection on the priority approach. With the priority vehicle's V pinned to 0.5 m/s and distance ~1 m, ETA = 2s < `gapThresholdSec` (3s), so the yielder must keep yielding for the entire run.

- [ ] **Step 1: Write the test**

Append to `internal/sim/world_test.go`:

```go
// TestWorld_StuckAtYieldNotDespawned: a vehicle correctly yielding at an
// unsignalized intersection must NOT be despawned, because
// stopDistanceForYield returning true is the "legitimately stopped" branch.
func TestWorld_StuckAtYieldNotDespawned(t *testing.T) {
	// Two incoming edges into an unsignalized intersection. Incoming[0]
	// (priority road) and Incoming[1] (yield road).
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -100, Y: 0}},  // W (priority origin)
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},  // S (yield origin)
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},     // center
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},   // E (downstream of priority)
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
		mkEdge(0, 0, 2), // priority approach: W -> C
		mkEdge(1, 1, 2), // yield approach:   S -> C
		mkEdge(2, 2, 3), // outbound:        C -> E (route exit for both)
	}
	xs := []network.Intersection{
		{
			ID:        0,
			NodeID:    2,
			Incoming:  []network.EdgeID{0, 1}, // 0 = priority, 1 = yield
			Outgoing:  []network.EdgeID{2},
			HasSignal: false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// Priority vehicle parked close to the stop line, moving slowly enough
	// that its ETA to the intersection is well inside gapThresholdSec (3s).
	// Yield vehicle approaching its own stop line.
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 99, V: 0.5}, // priority, ~1m out @ 0.5 m/s = 2s ETA
		{ID: 2, Route: []network.EdgeID{1, 2}, Edge: 1, S: 50, V: 10},  // yield, approaching
	}
	w.nextID = 3

	// Pin the priority vehicle's S and V each tick to keep the yield
	// continuously active. Without pinning, the priority vehicle would
	// clear the intersection within a tick or two.
	for i := 0; i < 1500; i++ {
		// Find the priority vehicle by ID and re-pin its state.
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 1 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 99
				w.Vehicles[j].V = 0.5
			}
		}
		w.Step()
	}

	// Find the yield vehicle by ID.
	var yielder *Vehicle
	for i := range w.Vehicles {
		if w.Vehicles[i].ID == 2 {
			yielder = &w.Vehicles[i]
		}
	}
	if yielder == nil {
		t.Fatal("yield vehicle (ID=2) was unexpectedly despawned")
	}
	if yielder.Edge != 1 {
		t.Errorf("yield vehicle should still be on approach edge 1, got edge %d", yielder.Edge)
	}
	if yielder.StuckTime != 0 {
		t.Errorf("StuckTime should be 0 for a vehicle legitimately yielding, got %.3f", yielder.StuckTime)
	}
}
```

- [ ] **Step 2: Run the test to verify it passes**

Run: `go test ./internal/sim/... -run TestWorld_StuckAtYieldNotDespawned -count=1 -v`

Expected: PASS. The implementation from Task 2 sets `StuckTime = 0` whenever `stopDistanceForYield` returns `true`, so the yielder should hold position without accumulating stuck time.

- [ ] **Step 3: Commit**

```bash
git add internal/sim/world_test.go
git commit -m "test(sim): vehicle yielding at unsignalized intersection is not despawned

Pairs with the red-light non-despawn test. The priority-vehicle V is
pinned each tick to keep ETA below the 3s gap threshold, holding the
yielder in a continuous yield state for the full 75 sim-second run."
```

---

## Task 5: Verify full test suite and determinism gate

**Files:** No source changes; verification only.

- [ ] **Step 1: Run the full sim package test suite**

Run: `go test ./internal/sim/... -count=1`

Expected: PASS — all tests, including the three new ones from this plan and the existing `TestWorld_TraceDeterminism`.

- [ ] **Step 2: Run the full repo test suite**

Run: `go test ./... -count=1`

Expected: PASS. Stuck-despawn doesn't change the trace format, so trace/replay/e2e tests should be unaffected.

- [ ] **Step 3: Sanity-check the determinism gate explicitly**

Run: `go test ./internal/sim/... -run TestWorld_TraceDeterminism -count=5 -v`

Expected: PASS five times in a row. If it fails even once, stop — the stuck-despawn change has introduced map-iteration order or other nondeterminism that must be fixed before merging.

- [ ] **Step 4: No commit needed for this task.**

The feature is complete after this verification step. No code changes were made in Task 5.
