# GPS Reroute-When-Blocked Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A GPS vehicle committed to a road that becomes fully closed reroutes promptly — diverting at the upstream intersection without stopping — instead of queuing at the entry block until the stuck-timeout.

**Architecture:** Three small changes in `internal/sim/world.go`: a `nextEdgeFullClosed` helper; widen the `Step` reroute trigger to also fire when a GPS vehicle's next edge is fully closed; bypass the 20s reroute cooldown in `maybeReroute` for that case. The routing math (`maybeReroute` body, `edgeCost`, A*) is unchanged — `edgeCost` already returns 1e9 for a FullClose edge, so any alternative wins. The entry block remains the safety net when no alternative exists.

**Tech Stack:** Go; the existing sim `World`/`maybeReroute`/`edgeCost` rerouting.

**Spec:** `docs/superpowers/specs/2026-05-22-reroute-when-blocked-design.md`

**Branch:** work on the existing `feat/reroute-when-blocked` branch (already checked out; the spec commit is there). End every commit message body with:
```
Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
```

---

## File Structure

| File | Responsibility | Action |
|---|---|---|
| `internal/sim/world.go` | `nextEdgeFullClosed` helper; widened reroute trigger in `Step`; cooldown bypass in `maybeReroute` | Modify |
| `internal/sim/incident_test.go` | unit + behavioral tests | Modify |
| `README.md` | one-line note | Modify |

Tests reuse `buildRerouteGraph()` (in `internal/sim/world_test.go`, same package): nodes 0,1,2,3; edges e0(0→1,len100), e1(1→3,len150,**direct**), e2(1→2,len110), e3(2→3,len110). Destination node 3. From node 1 the direct tail is `[1]`; the detour tail is `[2,3]`.

Tasks are ordered so each leaves the package compiling and green.

---

## Task 1: `nextEdgeFullClosed` helper

**Files:**
- Modify: `internal/sim/world.go` (add the helper immediately before `func (w *World) maybeReroute`, currently at line 928)
- Test: `internal/sim/incident_test.go` (append)

- [ ] **Step 1: Write the failing test**

Append to `internal/sim/incident_test.go` (it already imports `testing` and `github.com/lab1702/traffic-sim/internal/network`; `buildRerouteGraph` is in the same package):

```go
func TestWorld_NextEdgeFullClosed(t *testing.T) {
	net := buildRerouteGraph()
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	v := &Vehicle{Route: []network.EdgeID{0, 1}, RouteIdx: 0} // next edge is 1

	if w.nextEdgeFullClosed(v) {
		t.Fatal("no incident: next edge should not be reported closed")
	}
	for _, sev := range []Severity{Slowdown, LaneClose} {
		w.Incidents[1] = sev
		if w.nextEdgeFullClosed(v) {
			t.Fatalf("severity %d on next edge must not count as full close", sev)
		}
	}
	w.Incidents[1] = FullClose
	if !w.nextEdgeFullClosed(v) {
		t.Fatal("FullClose on the next edge should be detected")
	}
	// On the last edge there is no next edge.
	last := &Vehicle{Route: []network.EdgeID{0, 1}, RouteIdx: 1}
	if w.nextEdgeFullClosed(last) {
		t.Fatal("last edge: there is no next edge to be closed")
	}
}
```

- [ ] **Step 2: Run to confirm it fails**

Run: `go test ./internal/sim/ -run TestWorld_NextEdgeFullClosed`
Expected: FAIL — `w.nextEdgeFullClosed undefined`.

- [ ] **Step 3: Add the helper**

In `internal/sim/world.go`, immediately before `func (w *World) maybeReroute(v *Vehicle) bool {`, add:

```go
// nextEdgeFullClosed reports whether the vehicle's next route edge exists and is
// fully closed. Used to trigger an immediate reroute and to bypass the reroute
// cooldown, so a GPS vehicle diverts around a closure ahead of it instead of
// queuing at the entry block. O(1); false whenever there are no incidents.
func (w *World) nextEdgeFullClosed(v *Vehicle) bool {
	if v.RouteIdx+1 >= len(v.Route) {
		return false
	}
	return w.Incidents[v.Route[v.RouteIdx+1]] == FullClose
}
```

- [ ] **Step 4: Run to verify it passes**

Run: `go test ./internal/sim/ -run TestWorld_NextEdgeFullClosed -v`
Expected: PASS.
Also run `go build ./...` (the helper is currently unused — Go allows unused methods, so the build is clean).

- [ ] **Step 5: Commit**

```bash
git add internal/sim/world.go internal/sim/incident_test.go
git commit -m "$(cat <<'EOF'
feat(sim): add nextEdgeFullClosed helper

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: Cooldown bypass in `maybeReroute`

**Files:**
- Modify: `internal/sim/world.go` (`maybeReroute` cooldown gate, currently line 935)
- Test: `internal/sim/incident_test.go` (append)

- [ ] **Step 1: Write the failing test**

Append to `internal/sim/incident_test.go` (it already imports `github.com/lab1702/traffic-sim/internal/trace`):

```go
func TestWorld_FullClose_GPSReroutesWhenBlocked(t *testing.T) {
	// A GPS car on e0 whose next edge e1 (the direct edge to dest node 3) is
	// fully closed must reroute around it (to the detour tail [2,3]) EVEN within
	// the cooldown window — proving the cooldown bypass.
	net := buildRerouteGraph()
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	var events int
	w.EmitTrace = func(_ uint64, _ float64, e trace.Event) {
		if _, ok := e.(*trace.VehicleReroute); ok {
			events++
		}
	}
	w.Incidents[1] = FullClose

	v := &Vehicle{
		ID: 1, Route: []network.EdgeID{0, 1}, RouteIdx: 0, Edge: 0, S: 50, V: 5,
		HasGPS: true, DestNode: 3, LastRerouteSec: w.SimTime, // within cooldown
	}
	if !w.maybeReroute(v) {
		t.Fatal("blocked GPS car should reroute despite the cooldown (bypass)")
	}
	if len(v.Route) != 3 || v.Route[1] != 2 || v.Route[2] != 3 {
		t.Fatalf("route after reroute = %v, want [0 2 3] around the closure", v.Route)
	}
	if events != 1 {
		t.Fatalf("want 1 VehicleReroute event, got %d", events)
	}
}
```

- [ ] **Step 2: Run to confirm it fails**

Run: `go test ./internal/sim/ -run TestWorld_FullClose_GPSReroutesWhenBlocked`
Expected: FAIL — `LastRerouteSec == w.SimTime` is within the 20s cooldown, so `maybeReroute` returns false (no bypass yet) and the route stays `[0 1]`.

- [ ] **Step 3: Bypass the cooldown when the next edge is fully closed**

In `internal/sim/world.go` `maybeReroute`, replace the cooldown gate:

```go
	if w.SimTime-v.LastRerouteSec < rerouteCooldownSec {
		return false
	}
	v.LastRerouteSec = w.SimTime
```

with:

```go
	// Bypass the cooldown when the next edge is fully closed: a committed vehicle
	// should divert around a closure promptly rather than waiting out the cooldown.
	if !w.nextEdgeFullClosed(v) && w.SimTime-v.LastRerouteSec < rerouteCooldownSec {
		return false
	}
	v.LastRerouteSec = w.SimTime
```

- [ ] **Step 4: Run to verify it passes**

Run: `go test ./internal/sim/ -run TestWorld_FullClose_GPSReroutesWhenBlocked -v`
Expected: PASS.

- [ ] **Step 5: Run the existing reroute suite (no regressions)**

Run: `go test ./internal/sim/ -run TestWorld_Reroute -count=1`
Expected: PASS — in particular `TestWorld_Reroute_CooldownRespected` (its vehicle's next edge is jammed via congestion, not an incident, so `nextEdgeFullClosed` is false and the cooldown still applies).

- [ ] **Step 6: Commit**

```bash
git add internal/sim/world.go internal/sim/incident_test.go
git commit -m "$(cat <<'EOF'
feat(sim): bypass reroute cooldown when the next edge is fully closed

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: Widen the reroute trigger in `Step`

**Files:**
- Modify: `internal/sim/world.go` (reroute trigger, currently line 824-830)
- Test: `internal/sim/incident_test.go` (append)

- [ ] **Step 1: Write the failing tests**

Append to `internal/sim/incident_test.go`:

```go
func TestWorld_FullClose_GPSDivertsViaStep(t *testing.T) {
	// A GPS car driving along e0 toward the (now closed) direct edge e1 should
	// divert to the detour [2,3] via the Step trigger — without crossing e1 and
	// without needing an edge transition to trigger the reroute.
	net := buildRerouteGraph()
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Incidents[1] = FullClose
	w.Vehicles = []Vehicle{{
		ID: 1, Route: []network.EdgeID{0, 1}, RouteIdx: 0, Edge: 0, S: 50, V: 5,
		HasGPS: true, DestNode: 3, LastRerouteSec: -1000, // cooldown not the gate here
	}}
	w.nextID = 2

	diverted := false
	for i := 0; i < 400; i++ {
		w.Step()
		if len(w.Vehicles) == 0 {
			diverted = true // reached dest 3 — only possible via the detour
			break
		}
		if w.Vehicles[0].Edge == 1 {
			t.Fatalf("GPS car entered the closed direct edge at tick %d", i)
		}
		if r := w.Vehicles[0].Route; len(r) == 3 && r[1] == 2 && r[2] == 3 {
			diverted = true
		}
	}
	if !diverted {
		t.Fatal("GPS car did not divert around the closure via the Step trigger")
	}
}

func TestWorld_FullClose_GPSNoAlternativeQueues(t *testing.T) {
	// Linear 0->1->2; edge 1 is the ONLY path to dest node 2. With no alternative,
	// the GPS car keeps its route and queues at the entry block — never entering
	// the closed edge (the safety net still holds).
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		{ID: 2, Pos: network.Point{X: 400, Y: 0}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 15, Lanes: []network.Lane{{Index: 0}}},
		{ID: 1, From: 1, To: 2, Length: 200, SpeedLimit: 15, Lanes: []network.Lane{{Index: 0}}},
	}
	net := &network.Network{Nodes: nodes, Edges: edges}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Incidents[1] = FullClose
	w.Vehicles = []Vehicle{{
		ID: 1, Route: []network.EdgeID{0, 1}, RouteIdx: 0, Edge: 0, S: 20, V: 15,
		HasGPS: true, DestNode: 2, LastRerouteSec: -1000,
	}}
	w.nextID = 2

	for i := 0; i < 400; i++ {
		w.Step()
		if len(w.Vehicles) == 0 {
			t.Fatal("car despawned; expected it queued at the closure")
		}
		if w.Vehicles[0].Edge != 0 {
			t.Fatalf("GPS car with no alternative entered closed edge %d at tick %d",
				w.Vehicles[0].Edge, i)
		}
	}
	if r := w.Vehicles[0].Route; len(r) != 2 || r[1] != 1 {
		t.Fatalf("route should be unchanged [0 1] (no alternative), got %v", r)
	}
}
```

- [ ] **Step 2: Run to confirm `GPSDivertsViaStep` fails**

Run: `go test ./internal/sim/ -run 'TestWorld_FullClose_GPSDivertsViaStep|TestWorld_FullClose_GPSNoAlternativeQueues'`
Expected: `GPSDivertsViaStep` FAILs — without the widened trigger, `maybeReroute` is never called while the car stays on e0 (no edge transition), so it never diverts and the test times out its loop with `diverted == false`. (`GPSNoAlternativeQueues` may already pass via the entry block; that's fine — it's a safety-net regression test.)

- [ ] **Step 3: Widen the trigger**

In `internal/sim/world.go` `Step`, replace the reroute trigger block:

```go
		// GPS rerouting fires on edge entry (a decision point), bounded by the
		// per-tick budget. maybeReroute self-gates on HasGPS and cooldown.
		if !v.Despawned && v.Edge != prevEdge && rerouteBudget > 0 {
			if w.maybeReroute(v) {
				rerouteBudget--
			}
		}
```

with:

```go
		// GPS rerouting fires on edge entry (a decision point) and, additionally,
		// when the next edge ahead is fully closed (so a committed vehicle diverts
		// around a closure rather than queuing at it). Bounded by the per-tick
		// budget. maybeReroute self-gates on HasGPS and cooldown.
		if !v.Despawned && rerouteBudget > 0 &&
			(v.Edge != prevEdge || (v.HasGPS && w.nextEdgeFullClosed(v))) {
			if w.maybeReroute(v) {
				rerouteBudget--
			}
		}
```

- [ ] **Step 4: Run the new tests to verify they pass**

Run: `go test ./internal/sim/ -run 'TestWorld_FullClose_GPSDivertsViaStep|TestWorld_FullClose_GPSNoAlternativeQueues' -v`
Expected: PASS (both).

- [ ] **Step 5: Run the full sim package**

Run: `go test ./internal/sim/ -count=1`
Expected: PASS, including `TestWorld_TraceDeterminism` and all `TestWorld_Reroute_*`.

- [ ] **Step 6: Commit**

```bash
git add internal/sim/world.go internal/sim/incident_test.go
git commit -m "$(cat <<'EOF'
feat(sim): reroute GPS vehicles toward a fully-closed next edge

Widen the Step reroute trigger so a committed GPS vehicle diverts around
a closure ahead of it without needing an edge transition. With no
alternative it keeps its route and queues at the entry block.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Docs + full verification

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Update the README incidents note**

In `README.md`, in the `### Incidents (interactive)` section, find this sentence:

```markdown
- **Fully closed** — every lane is blocked; through-traffic queues and
  GPS-equipped vehicles reroute around it.
```

Replace it with:

```markdown
- **Fully closed** — every lane is blocked. GPS-equipped vehicles reroute around
  it (diverting promptly, even mid-approach); vehicles with no alternative — and
  vehicles without GPS — queue at the entrance until it reopens.
```

- [ ] **Step 2: Verify the README edit**

Run: `grep -n "diverting promptly" README.md`
Expected: prints the updated line.

- [ ] **Step 3: Full verification**

Run: `go build ./...` — clean.
Run: `go vet ./...` — clean.
Run: `go test ./... -count=1` — all packages PASS (incl. `TestWorld_TraceDeterminism`).

- [ ] **Step 4: Commit**

```bash
git add README.md
git commit -m "$(cat <<'EOF'
docs(readme): note GPS vehicles reroute promptly around closures

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

- [ ] **Step 5: Rebuild the root binaries**

```bash
go build ./cmd/trafficsim/ && go build ./cmd/tracereplay/
```

Expected: both build with no output. (They are gitignored; do not commit them.)

---

## Self-Review Notes

- **Spec coverage:** `nextEdgeFullClosed` helper (Task 1); widened trigger (Task 3); cooldown bypass (Task 2); FullClose-only (helper checks `== FullClose`); entry-block safety net for no-alternative (Task 3 `GPSNoAlternativeQueues`); determinism preserved (Task 3/4 run `TestWorld_TraceDeterminism`); README note (Task 4). All spec sections map to a task.
- **Type/name consistency:** `nextEdgeFullClosed(v *Vehicle) bool` is defined in Task 1 and used identically in the Task 2 cooldown gate and the Task 3 trigger. Test graph facts (direct edge 1, detour `[2,3]`, dest node 3) match `buildRerouteGraph`.
- **Test isolation:** Task 2's test calls `maybeReroute` directly with `LastRerouteSec == SimTime` to isolate the *cooldown bypass*; Task 3's `GPSDivertsViaStep` uses `LastRerouteSec = -1000` and drives via `Step` to isolate the *trigger*. Each new test fails before its task's change and passes after.
- **No placeholders:** every code step shows complete code; every run step gives the command and expected result.
