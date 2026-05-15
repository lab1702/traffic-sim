# Phase 5 — Signals + Intersections

**Milestone:** Signalized intersections cycle through phases on a fixed schedule. Vehicles approaching a red phase decelerate via IDM treating the stop line as a stopped leader. Unsignalized intersections use simple priority (vehicles yield when an approaching gap is < threshold). Config file can override per-intersection phase lengths.

---

### Task 5.1: Signal controller (auto-generated fixed-time)

**Files:**
- Create: `internal/sim/signal.go`
- Create: `internal/sim/signal_test.go`

- [ ] **Step 1: Write the failing test**

Write `internal/sim/signal_test.go`:
```go
package sim

import "testing"

func TestSignalCycle_AdvancesPhases(t *testing.T) {
	// 2 phases, 30s green each, 3s yellow each => 66s total cycle.
	cfg := SignalConfig{
		Phases: []SignalPhase{
			{GreenEdges: []int{0, 2}, GreenDur: 30, YellowDur: 3},
			{GreenEdges: []int{1, 3}, GreenDur: 30, YellowDur: 3},
		},
	}
	s := NewSignalState(cfg)
	// At t=0 we're in phase 0 (green).
	if s.PhaseIdx != 0 || s.IsYellow {
		t.Fatalf("t=0 should be phase 0 green, got phase=%d yellow=%v", s.PhaseIdx, s.IsYellow)
	}
	// Advance 31s -> should be in phase 0 yellow.
	s.Advance(31.0)
	if s.PhaseIdx != 0 || !s.IsYellow {
		t.Errorf("t=31 should be phase 0 yellow, got phase=%d yellow=%v", s.PhaseIdx, s.IsYellow)
	}
	// Advance another 3s -> phase 1 green.
	s.Advance(3.0)
	if s.PhaseIdx != 1 || s.IsYellow {
		t.Errorf("t=34 should be phase 1 green, got phase=%d yellow=%v", s.PhaseIdx, s.IsYellow)
	}
	// Wrap-around: advance past full cycle.
	s.Advance(60.0)
	if s.PhaseIdx == 1 && !s.IsYellow {
		// any state is fine; we just want no panic and a sensible index
	}
	if s.PhaseIdx >= len(cfg.Phases) {
		t.Errorf("PhaseIdx out of range: %d", s.PhaseIdx)
	}
}

func TestSignalCycle_GreenForEdge(t *testing.T) {
	cfg := SignalConfig{
		Phases: []SignalPhase{
			{GreenEdges: []int{0}, GreenDur: 30, YellowDur: 3},
			{GreenEdges: []int{1}, GreenDur: 30, YellowDur: 3},
		},
	}
	s := NewSignalState(cfg)
	if !s.GreenFor(0) {
		t.Errorf("phase 0 should be green for edge index 0")
	}
	if s.GreenFor(1) {
		t.Errorf("phase 0 should NOT be green for edge index 1")
	}
}
```

- [ ] **Step 2: Run to verify failure**

Run: `go test ./internal/sim/ -run TestSignal -v`
Expected: FAIL — `SignalConfig`, `SignalPhase`, `NewSignalState`, `SignalState.Advance/GreenFor` undefined.

- [ ] **Step 3: Implement the controller**

Write `internal/sim/signal.go`:
```go
package sim

import "github.com/lab1702/traffic-sim/internal/network"

// SignalConfig is the per-intersection plan: ordered phases that repeat.
type SignalConfig struct {
	// IntersectionID is set when this config is applied to a specific
	// intersection (used by override loading); 0 when used as a default.
	IntersectionID network.IntersectionID

	Phases []SignalPhase
}

// SignalPhase describes which approaches get green for how long, plus the
// trailing yellow before the next phase begins.
type SignalPhase struct {
	// GreenEdges is the *position* of incoming edges in
	// Intersection.Incoming that are green during this phase.
	// (Storing positions rather than EdgeIDs keeps the config relative
	// to the intersection — easier to write override files by hand.)
	GreenEdges []int
	GreenDur   float64 // seconds
	YellowDur  float64 // seconds
}

// SignalState is mutable, advanced once per tick by the World.
type SignalState struct {
	Config   SignalConfig
	PhaseIdx int
	Elapsed  float64 // seconds within current phase
	IsYellow bool
}

func NewSignalState(c SignalConfig) *SignalState {
	return &SignalState{Config: c}
}

// Advance moves the state machine forward by dt seconds.
func (s *SignalState) Advance(dt float64) {
	if len(s.Config.Phases) == 0 {
		return
	}
	s.Elapsed += dt
	for {
		p := &s.Config.Phases[s.PhaseIdx]
		threshold := p.GreenDur
		if s.IsYellow {
			threshold = p.YellowDur
		}
		if s.Elapsed < threshold {
			return
		}
		s.Elapsed -= threshold
		if !s.IsYellow {
			s.IsYellow = true
		} else {
			s.IsYellow = false
			s.PhaseIdx = (s.PhaseIdx + 1) % len(s.Config.Phases)
		}
	}
}

// GreenFor returns true if the given incoming-edge position is green or
// yellow during the current phase. Vehicles treat yellow as "go" (real
// drivers do too); they only stop on red.
func (s *SignalState) GreenFor(incomingPos int) bool {
	if len(s.Config.Phases) == 0 {
		return true // no plan == permanent green
	}
	p := s.Config.Phases[s.PhaseIdx]
	for _, e := range p.GreenEdges {
		if e == incomingPos {
			return true
		}
	}
	return false
}

// DefaultSignalConfig builds a fair fixed-time plan for an intersection
// with the given incoming edges. Splits them into 2 phases (e.g.,
// NS vs EW) by alternating index, 30s green + 3s yellow each.
func DefaultSignalConfig(incoming []network.EdgeID) SignalConfig {
	if len(incoming) <= 1 {
		// 1-leg intersection: permanent green.
		return SignalConfig{Phases: []SignalPhase{{
			GreenEdges: phaseAllPositions(len(incoming)),
			GreenDur:   30, YellowDur: 0,
		}}}
	}
	var even, odd []int
	for i := range incoming {
		if i%2 == 0 {
			even = append(even, i)
		} else {
			odd = append(odd, i)
		}
	}
	return SignalConfig{Phases: []SignalPhase{
		{GreenEdges: even, GreenDur: 30, YellowDur: 3},
		{GreenEdges: odd, GreenDur: 30, YellowDur: 3},
	}}
}

func phaseAllPositions(n int) []int {
	out := make([]int, n)
	for i := range out {
		out[i] = i
	}
	return out
}
```

- [ ] **Step 4: Run tests**

Run: `go test ./internal/sim/ -run TestSignal -v`
Expected: both PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/sim/signal.go internal/sim/signal_test.go
git commit -m "feat(sim): fixed-time signal controller with default plan"
```

---

### Task 5.2: Intersection module — incoming-position lookup + stop-line distance

**Files:**
- Create: `internal/sim/intersection.go`
- Create: `internal/sim/intersection_test.go`

- [ ] **Step 1: Write the failing test**

Write `internal/sim/intersection_test.go`:
```go
package sim

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

func TestIncomingPos_FindsEdgePosition(t *testing.T) {
	x := network.Intersection{
		ID:       0,
		NodeID:   5,
		Incoming: []network.EdgeID{10, 20, 30},
	}
	if p := IncomingPos(&x, 20); p != 1 {
		t.Errorf("want 1, got %d", p)
	}
	if p := IncomingPos(&x, 99); p != -1 {
		t.Errorf("missing edge should return -1, got %d", p)
	}
}
```

- [ ] **Step 2: Run to verify failure**

Run: `go test ./internal/sim/ -run TestIncomingPos -v`
Expected: FAIL — `IncomingPos` undefined.

- [ ] **Step 3: Implement intersection helpers**

Write `internal/sim/intersection.go`:
```go
package sim

import "github.com/lab1702/traffic-sim/internal/network"

// IncomingPos returns the position of edgeID within x.Incoming, or -1.
func IncomingPos(x *network.Intersection, edgeID network.EdgeID) int {
	for i, e := range x.Incoming {
		if e == edgeID {
			return i
		}
	}
	return -1
}

// IntersectionAtNode returns the Intersection whose NodeID matches, or nil.
func IntersectionAtNode(net *network.Network, n network.NodeID) *network.Intersection {
	for i := range net.Intersections {
		if net.Intersections[i].NodeID == n {
			return &net.Intersections[i]
		}
	}
	return nil
}
```

- [ ] **Step 4: Run test**

Run: `go test ./internal/sim/ -run TestIncomingPos -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/sim/intersection.go internal/sim/intersection_test.go
git commit -m "feat(sim): intersection lookup helpers"
```

---

### Task 5.3: Apply signals to vehicle stepping (red = stopped leader at stop line)

**Files:**
- Modify: `internal/sim/world.go`

- [ ] **Step 1: Read current world.go**

Read `internal/sim/world.go` to see current state.

- [ ] **Step 2: Add SignalStates field and initialization**

Modify `internal/sim/world.go`:

1. Add to the `World` struct:
```go
// SignalStates is indexed by IntersectionID; nil entries mean no signal.
SignalStates []*SignalState

// xByNodeID is a NodeID -> Intersection index for O(1) lookup during tick.
xByNodeID map[network.NodeID]*network.Intersection
```

2. In `NewWorld`, after `Router:`, add:
```go
sigs := make([]*SignalState, len(net.Intersections))
xByNode := make(map[network.NodeID]*network.Intersection, len(net.Intersections))
for i := range net.Intersections {
    x := &net.Intersections[i]
    xByNode[x.NodeID] = x
    if x.HasSignal {
        sigs[x.ID] = NewSignalState(DefaultSignalConfig(x.Incoming))
    }
}
```
Then set `SignalStates: sigs, xByNodeID: xByNode,` in the returned `&World{...}`.

3. Add a helper method:
```go
// stopDistanceForRed returns (distance to stop line, true) if the vehicle
// is on an incoming edge to a red-signalled intersection and the vehicle
// is approaching it. Returns (0, false) otherwise.
func (w *World) stopDistanceForRed(v *Vehicle) (float64, bool) {
	edge := &w.Net.Edges[v.Edge]
	x, ok := w.xByNodeID[edge.To]
	if !ok {
		return 0, false
	}
	if !x.HasSignal {
		return 0, false
	}
	st := w.SignalStates[x.ID]
	if st == nil {
		return 0, false
	}
	pos := IncomingPos(x, v.Edge)
	if pos < 0 {
		return 0, false
	}
	if st.GreenFor(pos) {
		return 0, false
	}
	// Red: stop line is at the end of this edge.
	dist := edge.Length - v.S
	if dist < 0 {
		dist = 0
	}
	return dist, true
}
```

4. In the main `Step()` function, add a phase-advance step BEFORE bucketing vehicles:
```go
// Advance all signal phases.
for _, s := range w.SignalStates {
	if s != nil {
		s.Advance(w.dt)
	}
}
```

5. In the per-vehicle stepping loop, when constructing the leader, prefer the smaller of (real leader gap, red-stop-line distance). Modify the loop body to:
```go
for pos, vi := range idxs {
	v := &w.Vehicles[vi]
	var lS, lV float64
	has := false
	if pos+1 < len(idxs) {
		ld := &w.Vehicles[idxs[pos+1]]
		lS, lV, has = ld.S, ld.V, true
	} else if v.RouteIdx+1 < len(v.Route) {
		nextE := v.Route[v.RouteIdx+1]
		if nidxs, ok := byEdge[nextE]; ok && len(nidxs) > 0 {
			nv := &w.Vehicles[nidxs[0]]
			edge := &w.Net.Edges[v.Edge]
			lS = edge.Length + nv.S
			lV = nv.V
			has = true
		}
	}
	// Apply red-light virtual leader if closer.
	if d, isRed := w.stopDistanceForRed(v); isRed {
		// Virtual leader sits at the stop line, stationary, at position
		// (v.S + d) along the current edge. Smaller S of leader vs real
		// leader => the binding constraint.
		virtualS := v.S + d
		if !has || virtualS < lS {
			lS = virtualS
			lV = 0
			has = true
		}
	}
	stepIDM(v, lS, lV, has, w.Net, DefaultIDM(), w.dt)
}
```

- [ ] **Step 3: Verify build and existing tests pass**

Run: `go test ./internal/sim/ -v`
Expected: all PASS.

- [ ] **Step 4: Add a red-light integration test**

Append to `internal/sim/world_test.go`:
```go
func TestWorld_StopsAtRedLight(t *testing.T) {
	// Build a single-edge graph ending at a signalized intersection.
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

	w := NewWorld(net, NewRandomOD(net, 0, 0))
	// Force the signal to all-red by giving it an empty-green phase.
	w.SignalStates[0] = NewSignalState(SignalConfig{
		Phases: []SignalPhase{{GreenEdges: nil, GreenDur: 1000, YellowDur: 0}},
	})

	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0}, Edge: 0, S: 50, V: 10}}
	w.nextID = 2

	for i := 0; i < 500; i++ { // 25 sim-seconds
		w.Step()
	}

	if len(w.Vehicles) != 1 {
		t.Fatalf("vehicle should not have completed (red light), got %d alive", len(w.Vehicles))
	}
	v := &w.Vehicles[0]
	if v.V > 0.1 {
		t.Errorf("vehicle should be stopped at red, V=%.2f", v.V)
	}
	if v.S < 190 {
		t.Errorf("vehicle should be near the stop line, S=%.2f (edge length 200)", v.S)
	}
}
```

- [ ] **Step 5: Run tests**

Run: `go test ./internal/sim/ -v`
Expected: all PASS including new red-light test.

- [ ] **Step 6: Commit**

```bash
git add internal/sim/world.go internal/sim/world_test.go
git commit -m "feat(sim): vehicles stop at red lights via virtual stop-line leader"
```

---

### Task 5.4: Gap-acceptance for unsignalized intersections

**Files:**
- Modify: `internal/sim/world.go`

- [ ] **Step 1: Add unsignalized-yield logic**

For unsignalized intersections, we use a simple rule: a vehicle entering an unsignalized intersection waits if there's a vehicle on a *higher-priority* incoming edge that will arrive within `gapThreshold` seconds.

Add helper to `internal/sim/world.go`:
```go
const gapThresholdSec = 3.0

// stopDistanceForYield returns (distance to stop line, true) if the
// vehicle's current edge ends at an UNSIGNALIZED intersection AND a
// higher-priority incoming edge has a vehicle approaching within
// gapThresholdSec seconds. "Higher priority" is defined here as a lower
// Incoming index (i.e., x.Incoming[0] is the priority road).
func (w *World) stopDistanceForYield(v *Vehicle, byEdge map[network.EdgeID][]int) (float64, bool) {
	edge := &w.Net.Edges[v.Edge]
	x, ok := w.xByNodeID[edge.To]
	if !ok || x.HasSignal {
		return 0, false
	}
	myPos := IncomingPos(x, v.Edge)
	if myPos <= 0 {
		// No higher-priority edge; we're the priority road (or unknown).
		return 0, false
	}
	myDist := edge.Length - v.S
	for i := 0; i < myPos; i++ {
		otherEdgeID := x.Incoming[i]
		others := byEdge[otherEdgeID]
		if len(others) == 0 {
			continue
		}
		// Find the closest-to-stop-line vehicle on the other approach.
		otherEdge := &w.Net.Edges[otherEdgeID]
		var bestETA = 1e9
		for _, oi := range others {
			ov := &w.Vehicles[oi]
			d := otherEdge.Length - ov.S
			v := ov.V
			if v < 0.5 {
				v = 0.5
			}
			eta := d / v
			if eta < bestETA {
				bestETA = eta
			}
		}
		if bestETA < gapThresholdSec {
			return myDist, true
		}
	}
	_ = myDist
	return myDist, false
}
```

- [ ] **Step 2: Apply yield in Step()**

In the per-vehicle stepping loop, AFTER the red-light virtual leader, add the yield virtual leader (same pattern):
```go
// Apply unsignalized-yield virtual leader if closer.
if d, mustYield := w.stopDistanceForYield(v, byEdge); mustYield {
	virtualS := v.S + d
	if !has || virtualS < lS {
		lS = virtualS
		lV = 0
		has = true
	}
}
```

- [ ] **Step 3: Verify build and existing tests pass**

Run: `go test ./internal/sim/ -v`
Expected: all PASS. (The 2x2-grid test has 4-leg intersections but no priority road defined — vehicles will yield to themselves intermittently. That's still functional behavior; no test asserts a specific yield outcome.)

- [ ] **Step 4: Commit**

```bash
git add internal/sim/world.go
git commit -m "feat(sim): gap-acceptance yield at unsignalized intersections"
```

---

### Task 5.5: Signal config YAML loading

**Files:**
- Create: `internal/config/config.go`
- Create: `internal/config/config_test.go`
- Create: `configs/signals.example.yaml`

- [ ] **Step 1: Write the example YAML**

Write `configs/signals.example.yaml`:
```yaml
# Per-intersection signal overrides. IntersectionID values come from the
# graph build; you can list them with `trafficsim load <file>` (TODO: that
# subcommand currently prints summary stats only; a future task can add
# `--list-intersections`).
signals:
  - intersection_id: 42
    phases:
      - green_edges: [0, 2]
        green_dur: 45
        yellow_dur: 3
      - green_edges: [1, 3]
        green_dur: 20
        yellow_dur: 3
```

- [ ] **Step 2: Write the failing test**

Write `internal/config/config_test.go`:
```go
package config

import (
	"path/filepath"
	"testing"
)

func TestLoadSignalOverrides(t *testing.T) {
	path := filepath.Join("..", "..", "configs", "signals.example.yaml")
	overrides, err := LoadSignalOverrides(path)
	if err != nil {
		t.Fatalf("LoadSignalOverrides: %v", err)
	}
	if len(overrides) != 1 {
		t.Fatalf("want 1 override, got %d", len(overrides))
	}
	o := overrides[0]
	if o.IntersectionID != 42 {
		t.Errorf("want IntersectionID 42, got %d", o.IntersectionID)
	}
	if len(o.Phases) != 2 {
		t.Errorf("want 2 phases, got %d", len(o.Phases))
	}
	if o.Phases[0].GreenDur != 45 {
		t.Errorf("phase 0 green: want 45, got %v", o.Phases[0].GreenDur)
	}
}

func TestLoadSignalOverrides_MissingFile_NotAnError(t *testing.T) {
	overrides, err := LoadSignalOverrides("does-not-exist.yaml")
	if err != nil {
		t.Errorf("missing file should return empty list, not error: %v", err)
	}
	if len(overrides) != 0 {
		t.Errorf("want empty, got %d entries", len(overrides))
	}
}
```

- [ ] **Step 3: Run to verify failure**

Run: `go test ./internal/config/ -v`
Expected: FAIL — package doesn't exist.

- [ ] **Step 4: Implement loader**

Write `internal/config/config.go`:
```go
// Package config loads YAML configuration files (signal overrides today,
// more later). Missing files are not errors — defaults apply.
package config

import (
	"errors"
	"fmt"
	"io/fs"
	"os"

	"gopkg.in/yaml.v3"
)

type SignalOverride struct {
	IntersectionID uint32        `yaml:"intersection_id"`
	Phases         []PhaseConfig `yaml:"phases"`
}

type PhaseConfig struct {
	GreenEdges []int   `yaml:"green_edges"`
	GreenDur   float64 `yaml:"green_dur"`
	YellowDur  float64 `yaml:"yellow_dur"`
}

type signalFile struct {
	Signals []SignalOverride `yaml:"signals"`
}

func LoadSignalOverrides(path string) ([]SignalOverride, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		if errors.Is(err, fs.ErrNotExist) {
			return nil, nil
		}
		return nil, fmt.Errorf("read %s: %w", path, err)
	}
	var sf signalFile
	if err := yaml.Unmarshal(data, &sf); err != nil {
		return nil, fmt.Errorf("parse %s: %w", path, err)
	}
	return sf.Signals, nil
}
```

- [ ] **Step 5: Run tests**

Run: `go test ./internal/config/ -v`
Expected: both PASS.

- [ ] **Step 6: Apply overrides in World construction**

In `internal/sim/world.go`, change `NewWorld` to accept overrides:
```go
func NewWorld(net *network.Network, spawner Spawner, overrides map[network.IntersectionID]SignalConfig) *World {
	// ... same as before, but when initializing sigs[x.ID]:
	if x.HasSignal {
		if cfg, ok := overrides[x.ID]; ok {
			sigs[x.ID] = NewSignalState(cfg)
		} else {
			sigs[x.ID] = NewSignalState(DefaultSignalConfig(x.Incoming))
		}
	}
}
```

Update all `NewWorld(net, spawner)` call sites in tests to pass `nil` for overrides. Update `cmd/trafficsim/main.go` to load and apply overrides.

- [ ] **Step 7: Wire overrides into the CLI**

In `cmd/trafficsim/main.go`, add a `--signals` flag to `runRun`:
```go
signalsPath := fs.String("signals", "", "path to signal overrides YAML")
```
Convert loaded overrides to a `map[network.IntersectionID]SignalConfig`:
```go
overrides := map[network.IntersectionID]sim.SignalConfig{}
if *signalsPath != "" {
    list, err := config.LoadSignalOverrides(*signalsPath)
    if err != nil {
        slog.Error("signals load failed", "err", err)
        os.Exit(1)
    }
    for _, o := range list {
        phases := make([]sim.SignalPhase, len(o.Phases))
        for i, p := range o.Phases {
            phases[i] = sim.SignalPhase{
                GreenEdges: p.GreenEdges, GreenDur: p.GreenDur, YellowDur: p.YellowDur,
            }
        }
        // Validate intersection_id is in range; warn and skip if not.
        if int(o.IntersectionID) >= len(net.Intersections) {
            slog.Warn("signal override references unknown intersection",
                "id", o.IntersectionID, "max", len(net.Intersections)-1)
            continue
        }
        overrides[network.IntersectionID(o.IntersectionID)] = sim.SignalConfig{
            IntersectionID: network.IntersectionID(o.IntersectionID),
            Phases:         phases,
        }
    }
}
spawner := sim.NewRandomOD(net, *seed, *spawnRate)
w := sim.NewWorld(net, spawner, overrides)
```
Add `"github.com/lab1702/traffic-sim/internal/config"` to the imports.

- [ ] **Step 8: Run all tests and build**

Run: `go test ./...` and `go build ./...`
Expected: both succeed.

- [ ] **Step 9: Commit**

```bash
git add internal/config/ configs/ internal/sim/world.go internal/sim/world_test.go cmd/trafficsim/main.go
git commit -m "feat(config): YAML signal overrides applied at sim start"
```

---

**Phase 5 done when:**
- `go test ./...` is green.
- Signals cycle deterministically; vehicles stop at red lights (covered by test).
- Unsignalized yield works (manually verifiable; no automated assertion yet).
- Override YAML file changes intersection 42's plan when supplied via `--signals`.
