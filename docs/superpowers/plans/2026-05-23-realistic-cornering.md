# Realistic Radius-Based Cornering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the angle-based corner cap (which panic-brakes far up a straight road) with a radius + lateral-acceleration cornering model that eases vehicles smoothly into a geometry-appropriate corner speed reached at the corner.

**Architecture:** All changes are contained to `internal/sim/cornering.go` and its test. Estimate the upcoming turn's radius by fitting a circle through three points sampled ~15 m before / at / after the junction node, derive a comfortable corner speed `v=√(a_lat·R)`, and feed IDM a smooth kinematic desired-speed profile `√(v_safe² + 2·a_brake·d)`. No changes to `idm.go`, `world.go`, lane logic, snapshot, renderer, or trace.

**Tech Stack:** Go; standard `math`; `internal/network` geometry types. Tests via `go test ./internal/sim/`.

**Spec:** `docs/superpowers/specs/2026-05-23-realistic-cornering-design.md`

---

## File Structure

- `internal/sim/cornering.go` — modified. Adds geometry helpers (`circumradius`, `pointBackFromEnd`, `pointForwardFromStart`), the radius model (`turnRadius`, `cornerSpeed`), new tuning constants, and a rewritten `computeDesiredSpeed`. Removes `cornerSpeedCap`, `shouldApplyCornerCap`, `cornerBrakingDecel`, `cornerReactionBuf`.
- `internal/sim/cornering_test.go` — modified. Adds unit tests for the new helpers/model, updates `TestWorld_BrakesForSharpTurn`, adds a gentle-braking regression, removes `TestCornerSpeedCap_Anchors`. Keeps `TestWorld_DoesNotBrakeForStraight`.

Tasks 1 and 2 are purely additive (old functions remain, harmlessly unused — Go does not error on unused package-level functions). Task 3 wires the new model in and removes the old code.

---

## Task 1: Geometry helpers — `circumradius` and polyline walks

**Files:**
- Modify: `internal/sim/cornering.go` (append new functions)
- Test: `internal/sim/cornering_test.go` (append new tests)

- [ ] **Step 1: Write the failing tests**

Append to `internal/sim/cornering_test.go`:

```go
func TestCircumradius(t *testing.T) {
	// Collinear points -> +Inf (a straight road has no curvature constraint).
	got := circumradius(
		network.Point{X: 0, Y: 0},
		network.Point{X: 10, Y: 0},
		network.Point{X: 20, Y: 0})
	if !math.IsInf(got, 1) {
		t.Errorf("collinear: want +Inf, got %.3f", got)
	}
	// Right isosceles triangle, right angle at the apex, legs 15: for a right
	// triangle the circumradius is half the hypotenuse = 15*sqrt(2)/2 (~10.6).
	got = circumradius(
		network.Point{X: -15, Y: 0},
		network.Point{X: 0, Y: 0},
		network.Point{X: 0, Y: -15})
	want := 15 * math.Sqrt2 / 2
	if math.Abs(got-want) > 0.1 {
		t.Errorf("right-angle circumradius: want %.3f, got %.3f", want, got)
	}
}

func TestPolylineWalk(t *testing.T) {
	// Single long segment: interpolate within it.
	geom := []network.Point{{X: 0, Y: 0}, {X: 100, Y: 0}}
	if p := pointBackFromEnd(geom, 15); math.Abs(p.X-85) > 1e-9 || math.Abs(p.Y) > 1e-9 {
		t.Errorf("pointBackFromEnd 15m: got (%.3f,%.3f) want (85,0)", p.X, p.Y)
	}
	if p := pointForwardFromStart(geom, 15); math.Abs(p.X-15) > 1e-9 || math.Abs(p.Y) > 1e-9 {
		t.Errorf("pointForwardFromStart 15m: got (%.3f,%.3f) want (15,0)", p.X, p.Y)
	}
	// Shorter than dist: clamp to the far endpoint.
	short := []network.Point{{X: 0, Y: 0}, {X: 5, Y: 0}}
	if p := pointBackFromEnd(short, 15); math.Abs(p.X) > 1e-9 {
		t.Errorf("pointBackFromEnd short edge: got X=%.3f want 0 (clamp to start)", p.X)
	}
	if p := pointForwardFromStart(short, 15); math.Abs(p.X-5) > 1e-9 {
		t.Errorf("pointForwardFromStart short edge: got X=%.3f want 5 (clamp to end)", p.X)
	}
	// Multi-segment: walk across a vertex.
	multi := []network.Point{{X: 0, Y: 0}, {X: 10, Y: 0}, {X: 20, Y: 0}}
	if p := pointBackFromEnd(multi, 15); math.Abs(p.X-5) > 1e-9 {
		t.Errorf("pointBackFromEnd across vertex: got X=%.3f want 5", p.X)
	}
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `go test ./internal/sim/ -run 'TestCircumradius|TestPolylineWalk' -v`
Expected: compile error — `undefined: circumradius`, `undefined: pointBackFromEnd`, `undefined: pointForwardFromStart`.

- [ ] **Step 3: Implement the helpers**

Append to `internal/sim/cornering.go` (above or below the existing functions — order does not matter):

```go
// circumradius returns the radius of the circle through three points, or +Inf
// when they are (near-)collinear. A straight road therefore yields no
// curvature constraint. R = (|ab|·|bc|·|ca|) / (4·area); area is half the
// cross-product magnitude, so R = product / (2·crossMag).
func circumradius(p1, p2, p3 network.Point) float64 {
	a := math.Hypot(p1.X-p2.X, p1.Y-p2.Y)
	b := math.Hypot(p2.X-p3.X, p2.Y-p3.Y)
	c := math.Hypot(p3.X-p1.X, p3.Y-p1.Y)
	crossMag := math.Abs((p2.X-p1.X)*(p3.Y-p1.Y) - (p3.X-p1.X)*(p2.Y-p1.Y))
	if crossMag < 1e-9 {
		return math.Inf(1)
	}
	return (a * b * c) / (2 * crossMag)
}

// pointBackFromEnd returns the point reached walking `dist` metres back from the
// end of the polyline, interpolating within the segment where the distance runs
// out. Polylines shorter than `dist` clamp to the first point. Assumes
// len(geom) >= 2. Zero-length segments (duplicate points) are skipped.
func pointBackFromEnd(geom []network.Point, dist float64) network.Point {
	var acc float64
	for i := len(geom) - 1; i > 0; i-- {
		ax, ay := geom[i].X, geom[i].Y
		bx, by := geom[i-1].X, geom[i-1].Y
		seg := math.Hypot(bx-ax, by-ay)
		if seg == 0 {
			continue
		}
		if acc+seg >= dist {
			t := (dist - acc) / seg
			return network.Point{X: ax + (bx-ax)*t, Y: ay + (by-ay)*t}
		}
		acc += seg
	}
	return geom[0]
}

// pointForwardFromStart mirrors pointBackFromEnd, walking `dist` metres forward
// from the start of the polyline; clamps to the last point for short polylines.
func pointForwardFromStart(geom []network.Point, dist float64) network.Point {
	var acc float64
	for i := 0; i < len(geom)-1; i++ {
		ax, ay := geom[i].X, geom[i].Y
		bx, by := geom[i+1].X, geom[i+1].Y
		seg := math.Hypot(bx-ax, by-ay)
		if seg == 0 {
			continue
		}
		if acc+seg >= dist {
			t := (dist - acc) / seg
			return network.Point{X: ax + (bx-ax)*t, Y: ay + (by-ay)*t}
		}
		acc += seg
	}
	return geom[len(geom)-1]
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `go test ./internal/sim/ -run 'TestCircumradius|TestPolylineWalk' -v`
Expected: PASS (both tests).

- [ ] **Step 5: Commit**

```bash
git add internal/sim/cornering.go internal/sim/cornering_test.go
git commit -m "feat(sim): add circumradius and polyline-walk geometry helpers"
```

---

## Task 2: Radius model — `turnRadius` and `cornerSpeed`

**Files:**
- Modify: `internal/sim/cornering.go` (add constants + two functions)
- Test: `internal/sim/cornering_test.go` (append new tests)

- [ ] **Step 1: Write the failing tests**

Append to `internal/sim/cornering_test.go`:

```go
func TestCornerSpeed(t *testing.T) {
	if v := cornerSpeed(math.Inf(1)); !math.IsInf(v, 1) {
		t.Errorf("infinite radius: want +Inf, got %.3f", v)
	}
	if v := cornerSpeed(0.001); v != minCornerSpeed {
		t.Errorf("tiny radius: want floor %.2f, got %.3f", minCornerSpeed, v)
	}
	want := math.Sqrt(cornerLatAccel * 12)
	if v := cornerSpeed(12); math.Abs(v-want) > 1e-9 {
		t.Errorf("R=12: want %.3f, got %.3f", want, v)
	}
}

func TestTurnRadius(t *testing.T) {
	// Straight two-edge path -> +Inf.
	straight := &network.Network{Edges: []network.Edge{
		{ID: 0, Geometry: []network.Point{{X: 0, Y: 0}, {X: 100, Y: 0}}},
		{ID: 1, Geometry: []network.Point{{X: 100, Y: 0}, {X: 200, Y: 0}}},
	}}
	if r := turnRadius(straight, 0, 1); !math.IsInf(r, 1) {
		t.Errorf("straight path: want +Inf radius, got %.3f", r)
	}

	// 90° elbow with 15m sample arms -> right-triangle circumradius ~10.6m.
	elbow := &network.Network{Edges: []network.Edge{
		{ID: 0, Geometry: []network.Point{{X: 0, Y: 0}, {X: 100, Y: 0}}},
		{ID: 1, Geometry: []network.Point{{X: 100, Y: 0}, {X: 100, Y: -100}}},
	}}
	want := 15 * math.Sqrt2 / 2
	if r := turnRadius(elbow, 0, 1); math.Abs(r-want) > 0.5 {
		t.Errorf("90° elbow: want ~%.2f, got %.2f", want, r)
	}

	// Jagged-but-straight: a short angled end stub on edge 0, but the road is
	// straight over the 15m sample. The corner speed must exceed a 40km/h
	// (11.2 m/s) limit so no false slowdown happens (artifact regression).
	jagged := &network.Network{Edges: []network.Edge{
		{ID: 0, Geometry: []network.Point{{X: 0, Y: 0}, {X: 98, Y: 0}, {X: 100, Y: 1.5}}},
		{ID: 1, Geometry: []network.Point{{X: 100, Y: 1.5}, {X: 200, Y: 1.5}}},
	}}
	if vs := cornerSpeed(turnRadius(jagged, 0, 1)); vs < 11.2 {
		t.Errorf("jagged-but-straight: corner speed %.2f should exceed 40km/h (no false slowdown)", vs)
	}
}

func TestTurnRadius_SweepingVsTight(t *testing.T) {
	// Shallow bend: large radius -> high corner speed (above a 40km/h limit).
	shallow := &network.Network{Edges: []network.Edge{
		{ID: 0, Geometry: []network.Point{{X: 0, Y: 0}, {X: 100, Y: 0}}},
		{ID: 1, Geometry: []network.Point{{X: 100, Y: 0}, {X: 200, Y: -20}}},
	}}
	// Tight 90° corner: small radius -> low corner speed.
	tight := &network.Network{Edges: []network.Edge{
		{ID: 0, Geometry: []network.Point{{X: 0, Y: 0}, {X: 100, Y: 0}}},
		{ID: 1, Geometry: []network.Point{{X: 100, Y: 0}, {X: 100, Y: -100}}},
	}}
	sV := cornerSpeed(turnRadius(shallow, 0, 1))
	tV := cornerSpeed(turnRadius(tight, 0, 1))
	if sV <= tV {
		t.Errorf("sweeping (%.2f) should allow a higher speed than tight (%.2f)", sV, tV)
	}
	if sV < 11.2 {
		t.Errorf("shallow bend corner speed %.2f should exceed 40km/h (barely slows)", sV)
	}
	if tV >= 11.2 {
		t.Errorf("tight 90° corner speed %.2f should be below 40km/h (clearly slows)", tV)
	}
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `go test ./internal/sim/ -run 'TestCornerSpeed|TestTurnRadius' -v`
Expected: compile error — `undefined: cornerSpeed`, `undefined: turnRadius`, `undefined: minCornerSpeed`, `undefined: cornerLatAccel`.

- [ ] **Step 3: Implement the constants and functions**

Append to `internal/sim/cornering.go`:

```go
// Comfortable cornering parameters. A turn's safe speed comes from the lateral
// acceleration a driver tolerates while rounding a curve of the estimated
// radius: v = sqrt(cornerLatAccel * R).
const (
	cornerLatAccel   = 3.0  // m/s^2, comfortable lateral acceleration
	cornerSampleDist = 15.0 // m, radius sampling arm length on each side
	minCornerSpeed   = 2.5  // m/s (~9 km/h), floor so hairpins crawl, not stop
)

// turnRadius estimates the radius (m) of the turn from fromEdge onto toEdge by
// fitting a circle through a point cornerSampleDist back along fromEdge, the
// shared junction node, and a point cornerSampleDist forward along toEdge.
// Returns +Inf when either edge lacks geometry or the path is straight. The
// sample arms make the estimate robust to short, jagged OSM end-segments.
func turnRadius(net *network.Network, fromEdge, toEdge network.EdgeID) float64 {
	fg := net.Edges[fromEdge].Geometry
	tg := net.Edges[toEdge].Geometry
	if len(fg) < 2 || len(tg) < 2 {
		return math.Inf(1)
	}
	before := pointBackFromEnd(fg, cornerSampleDist)
	node := fg[len(fg)-1] // == tg[0], the shared junction
	after := pointForwardFromStart(tg, cornerSampleDist)
	return circumradius(before, node, after)
}

// cornerSpeed returns the comfortable speed (m/s) for a turn of radius R using
// the lateral-acceleration model, floored at minCornerSpeed. R == +Inf (a
// straight road) passes through as +Inf — no constraint.
func cornerSpeed(R float64) float64 {
	if math.IsInf(R, 1) {
		return math.Inf(1)
	}
	v := math.Sqrt(cornerLatAccel * R)
	if v < minCornerSpeed {
		return minCornerSpeed
	}
	return v
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `go test ./internal/sim/ -run 'TestCornerSpeed|TestTurnRadius' -v`
Expected: PASS (`TestCornerSpeed`, `TestTurnRadius`, `TestTurnRadius_SweepingVsTight`).

- [ ] **Step 5: Commit**

```bash
git add internal/sim/cornering.go internal/sim/cornering_test.go
git commit -m "feat(sim): add radius + lateral-accel corner-speed model"
```

---

## Task 3: Wire in the smooth profile and remove the old angle cap

**Files:**
- Modify: `internal/sim/cornering.go` (rewrite `computeDesiredSpeed`; delete old symbols; add `cornerBrakeDecel`)
- Test: `internal/sim/cornering_test.go` (remove `TestCornerSpeedCap_Anchors`; update `TestWorld_BrakesForSharpTurn`; add gentle-braking test)

- [ ] **Step 1: Update the behavior tests**

In `internal/sim/cornering_test.go`:

1. **Delete** `TestCornerSpeedCap_Anchors` entirely (it tests the removed angle table).
2. **Replace** `TestWorld_BrakesForSharpTurn` with the version below.
3. **Append** `TestWorld_CornerBrakingIsGentle`.
4. Leave `TestWorld_DoesNotBrakeForStraight` unchanged.

```go
// TestWorld_BrakesForSharpTurn: a car at the speed limit approaching a 90° turn
// should slow to about the radius-based corner speed by the time it crosses.
func TestWorld_BrakesForSharpTurn(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		{ID: 2, Pos: network.Point{X: 200, Y: -50}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 15,
			Lanes: []network.Lane{{Index: 0}}, Geometry: []network.Point{{X: 0, Y: 0}, {X: 200, Y: 0}}},
		{ID: 1, From: 1, To: 2, Length: 50, SpeedLimit: 15,
			Lanes: []network.Lane{{Index: 0}}, Geometry: []network.Point{{X: 200, Y: 0}, {X: 200, Y: -50}}},
	}
	net := &network.Network{Nodes: nodes, Edges: edges}

	expected := cornerSpeed(turnRadius(net, 0, 1))
	if expected >= 15 {
		t.Fatalf("test prerequisite: corner speed %.2f should be < edge speed 15", expected)
	}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, S: 0, V: 15}}

	minV := math.Inf(1)
	for i := 0; i < 500; i++ {
		w.Step()
		if len(w.Vehicles) == 0 {
			break
		}
		v := &w.Vehicles[0]
		onApproach := v.Edge == 0 && (200-v.S) < 30
		onExit := v.Edge == 1
		if (onApproach || onExit) && v.V < minV {
			minV = v.V
		}
	}
	if minV > expected+2.0 {
		t.Errorf("did not slow enough for the 90° corner: minV=%.2f, corner speed=%.2f", minV, expected)
	}
	if minV < 1.0 {
		t.Errorf("braked to a near stop: minV=%.2f, corner speed=%.2f", minV, expected)
	}
}

// TestWorld_CornerBrakingIsGentle: easing into a 90° turn must not hit the
// panic-brake clamp — the deceleration should stay well above -MaxBraking.
func TestWorld_CornerBrakingIsGentle(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		{ID: 2, Pos: network.Point{X: 200, Y: -100}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 15,
			Lanes: []network.Lane{{Index: 0}}, Geometry: []network.Point{{X: 0, Y: 0}, {X: 200, Y: 0}}},
		{ID: 1, From: 1, To: 2, Length: 100, SpeedLimit: 15,
			Lanes: []network.Lane{{Index: 0}}, Geometry: []network.Point{{X: 200, Y: 0}, {X: 200, Y: -100}}},
	}
	net := &network.Network{Nodes: nodes, Edges: edges}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, S: 0, V: 15}}

	minA := 0.0
	for i := 0; i < 600; i++ {
		w.Step()
		if len(w.Vehicles) == 0 {
			break
		}
		if a := w.Vehicles[0].A; a < minA {
			minA = a
		}
	}
	if minA < -4.0 {
		t.Errorf("corner braking too hard: peak decel %.2f m/s² (want > -4.0; MaxBraking is -%.1f)", minA, MaxBraking)
	}
}
```

- [ ] **Step 2: Run the updated tests to verify they fail**

Run: `go test ./internal/sim/ -run 'TestWorld_BrakesForSharpTurn|TestWorld_CornerBrakingIsGentle' -v`
Expected: `TestWorld_CornerBrakingIsGentle` FAILS (old model panic-brakes at -8.0, so `minA < -4.0`). `TestWorld_BrakesForSharpTurn` may still pass or fail depending on the old cap; that's fine — the gentle test is the proof the change is needed.

- [ ] **Step 3: Rewrite `computeDesiredSpeed` and delete the old angle cap**

In `internal/sim/cornering.go`:

1. **Delete** these four symbols and their doc comments: `cornerSpeedCap`, `cornerBrakingDecel`, `cornerReactionBuf`, `shouldApplyCornerCap`.
2. **Replace** the body of `computeDesiredSpeed` with the version below.
3. **Add** the `cornerBrakeDecel` constant (next to the other corner constants):

```go
// cornerBrakeDecel is the planning deceleration (m/s^2) for the smooth approach
// profile: the desired speed eases down so the vehicle reaches the corner speed
// at the corner braking at roughly this rate, instead of slamming the brakes.
const cornerBrakeDecel = 2.0
```

```go
// computeDesiredSpeed returns the v0 (desired speed) for IDM. It is the current
// edge's speed limit (scaled by the driver's preference and any Slowdown
// incident), optionally reduced for an upcoming turn. The turn reduction uses a
// radius-based comfortable speed and a smooth kinematic approach profile so the
// vehicle eases into the corner rather than braking hard far upstream.
func (w *World) computeDesiredSpeed(v *Vehicle) float64 {
	edge := &w.Net.Edges[v.Edge]
	// Per-driver speed preference. A zero factor means a hand-constructed
	// vehicle (test fixture) — treat as 1.0 so existing tests work.
	factor := v.SpeedFactor
	if factor == 0 {
		factor = 1.0
	}
	v0 := edge.SpeedLimit * factor
	if w.Incidents[v.Edge] == Slowdown {
		if cap := edge.SpeedLimit * incidentSlowdownFactor; cap < v0 {
			v0 = cap
		}
	}
	if v.RouteIdx+1 >= len(v.Route) {
		return v0 // no next edge: nothing to slow for
	}
	nextEdge := v.Route[v.RouteIdx+1]
	vSafe := cornerSpeed(turnRadius(w.Net, v.Edge, nextEdge))
	if math.IsInf(vSafe, 1) || vSafe >= v0 {
		return v0 // straight or gentle turn: no slowdown
	}
	// Kinematic approach: the max speed from which we can still decelerate at
	// cornerBrakeDecel to reach vSafe by the corner (distance d ahead). Far from
	// the corner this exceeds v0 (no effect); it eases to vSafe as d -> 0.
	d := edge.Length - v.S
	if d < 0 {
		d = 0
	}
	if v0corner := math.Sqrt(vSafe*vSafe + 2*cornerBrakeDecel*d); v0corner < v0 {
		return v0corner
	}
	return v0
}
```

- [ ] **Step 4: Run the cornering tests to verify they pass**

Run: `go test ./internal/sim/ -run 'TestCircumradius|TestPolylineWalk|TestCornerSpeed|TestTurnRadius|TestWorld_BrakesForSharpTurn|TestWorld_CornerBrakingIsGentle|TestWorld_DoesNotBrakeForStraight' -v`
Expected: all PASS. (If `TestWorld_CornerBrakingIsGentle`'s `minA` is borderline near -4.0, that indicates the profile is not engaging smoothly — re-check the `computeDesiredSpeed` rewrite; expected peak is roughly -2 to -2.5 m/s².)

- [ ] **Step 5: Run the full suite and vet**

Run: `go test ./... && go vet ./...`
Expected: all packages PASS (including `TestComputeDesiredSpeed_SlowdownCap` in `incident_test.go`, whose single-edge-route vehicle skips the corner logic), vet clean. Confirm there are no remaining references to the deleted symbols:

Run: `grep -rn "cornerSpeedCap\|shouldApplyCornerCap\|cornerBrakingDecel\|cornerReactionBuf" --include=*.go .`
Expected: no output.

- [ ] **Step 6: Commit**

```bash
git add internal/sim/cornering.go internal/sim/cornering_test.go
git commit -m "feat(sim): smooth radius-based cornering, replace angle cap"
```

---

## Manual verification (optional, after Task 3)

Build and watch traffic on a real extract — cars should slow only for genuine turns/tight bends, easing in rather than slamming the brakes on straight approaches:

```bash
go build -o trafficsim ./cmd/trafficsim
./trafficsim run --spawn-rate 20 /home/lab/MEGA/OpenStreetMap/jackson.osm
```

---

## Self-Review Notes

- **Spec coverage:** radius from 3-point circle (Task 2 `turnRadius`); lateral-accel speed + floor (Task 2 `cornerSpeed`); smooth kinematic profile (Task 3 `computeDesiredSpeed`); baseline measurement / artifact removal (`cornerSampleDist` arms, asserted by the jagged test in Task 2); panic-brake removal (gentle test in Task 3); junction-only scope (only `Route[RouteIdx+1]` consulted); removed old symbols (Task 3 Step 3 + grep in Step 5). All spec sections map to a task.
- **Naming consistency:** `circumradius`, `pointBackFromEnd`, `pointForwardFromStart`, `turnRadius`, `cornerSpeed`, `computeDesiredSpeed`; constants `cornerLatAccel`, `cornerSampleDist`, `minCornerSpeed`, `cornerBrakeDecel` — used identically across tasks.
- **No placeholders:** every code/test step shows complete code and exact commands.
