# Turn-Aware Lane Choice Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make vehicles position themselves in a turn-compatible lane on the approach to an intersection, with a snap fallback at the crossing so every turn departs from a legal lane.

**Architecture:** Pre-compute per-lane allowed-outgoing-edges at netbuild time (OSM `turn:lanes=*` plus geometric fallback via `network.ClassifyTurn`), store in the existing `network.Lane.AllowedTurns` field. At runtime, extend `tryLaneChange` to bias toward compatible lanes within 300 m of the turn, and snap `v.Lane` in the route-advance block to a category-correct lane on the new edge.

**Tech Stack:** Go 1.21+, `log/slog`, `github.com/paulmach/osm`, existing `internal/network`, `internal/netbuild`, `internal/sim` packages.

**Spec:** [`../specs/2026-05-15-turn-aware-lane-choice-design.md`](../specs/2026-05-15-turn-aware-lane-choice-design.md)

---

## File Structure

**New files:**
- `internal/netbuild/lanes.go` — OSM `turn:lanes` parser, geometric fallback assignment, per-edge orchestration.
- `internal/netbuild/lanes_test.go` — unit tests for parser and assignment.
- `internal/sim/vehicle_test.go` — unit tests for the intersection-snap rule (current `vehicle.go` has no test file).

**Modified files:**
- `internal/netbuild/netbuild.go` — allocate one lane slice per edge (not one per segment), wire the lane-assignment pass into `Build()`, plumb `feat *osmload.Features` into the pass so it can read way tags.
- `internal/sim/lanechange.go` — add turn-bias branch; make speed-driven LC reject incompatible neighbor lanes when the trigger is active.
- `internal/sim/lanechange_test.go` — add tests covering bias, blocked bias, beyond-trigger, last-edge cases.
- `internal/sim/vehicle.go` — in the route-advance block, classify the just-taken turn and set `v.Lane` accordingly; emit a snap-fallback warning when bias didn't succeed.
- `internal/e2e/e2e_test.go` — assert the snap-fallback warning rate stays below a threshold.

---

## Task 1: Per-direction lane slices (pre-flight bug fix)

**Files:**
- Modify: `internal/netbuild/netbuild.go` (around lines 140-153)
- Test: `internal/netbuild/netbuild_test.go` (extend)

Today the same `lanes` slice is assigned to both the forward and reverse `Edge` for a two-way street. Once `AllowedTurns` becomes per-direction state, the two directions must own distinct slices.

- [ ] **Step 1: Write the failing test**

Add to `internal/netbuild/netbuild_test.go`:

```go
// TestBuild_TwoWayEdgesHaveDistinctLaneSlices verifies that mutating one
// direction's lanes does not affect the reverse direction's lanes. This
// matters once per-direction state (AllowedTurns) is stored on Lane.
func TestBuild_TwoWayEdgesHaveDistinctLaneSlices(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0),
		2: mkNode(2, 40.0, -74.001),
	}}
	feat.Ways = []*osm.Way{mkWay(10, "residential", false, 1, 2)}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Edges) != 2 {
		t.Fatalf("want 2 edges (fwd+rev), got %d", len(net.Edges))
	}
	// Mutate one direction's lanes; the other must not change.
	net.Edges[0].Lanes[0].AllowedTurns = []network.EdgeID{99}
	if len(net.Edges[1].Lanes[0].AllowedTurns) != 0 {
		t.Errorf("reverse edge lanes aliased to forward; got AllowedTurns=%v",
			net.Edges[1].Lanes[0].AllowedTurns)
	}
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./internal/netbuild/ -run TestBuild_TwoWayEdgesHaveDistinctLaneSlices -v`
Expected: FAIL — `reverse edge lanes aliased to forward; got AllowedTurns=[99]`

- [ ] **Step 3: Fix lane allocation in netbuild.go**

In `internal/netbuild/netbuild.go`, replace the segment-level `lanes := makeLanes(lanesPerDir)` with per-direction allocation. Current lines 140-153 look like:

```go
segChains = append(segChains, chain)
lanes := makeLanes(lanesPerDir)
edges = append(edges, network.Edge{
    ID: network.EdgeID(len(edges)), From: fromID, To: toID,
    Lanes: lanes, Length: length, SpeedLimit: speedFwd, Geometry: geom,
})
osmWayOfEdge = append(osmWayOfEdge, w.ID)
if !oneway {
    revGeom := reverseGeom(geom)
    edges = append(edges, network.Edge{
        ID: network.EdgeID(len(edges)), From: toID, To: fromID,
        Lanes: lanes, Length: length, SpeedLimit: speedBwd, Geometry: revGeom,
    })
    osmWayOfEdge = append(osmWayOfEdge, w.ID)
}
```

Change to:

```go
segChains = append(segChains, chain)
edges = append(edges, network.Edge{
    ID: network.EdgeID(len(edges)), From: fromID, To: toID,
    Lanes: makeLanes(lanesPerDir), Length: length, SpeedLimit: speedFwd, Geometry: geom,
})
osmWayOfEdge = append(osmWayOfEdge, w.ID)
if !oneway {
    revGeom := reverseGeom(geom)
    edges = append(edges, network.Edge{
        ID: network.EdgeID(len(edges)), From: toID, To: fromID,
        Lanes: makeLanes(lanesPerDir), Length: length, SpeedLimit: speedBwd, Geometry: revGeom,
    })
    osmWayOfEdge = append(osmWayOfEdge, w.ID)
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `go test ./internal/netbuild/ -run TestBuild_TwoWayEdgesHaveDistinctLaneSlices -v`
Expected: PASS.

Also run the full netbuild test suite: `go test ./internal/netbuild/`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/netbuild/netbuild.go internal/netbuild/netbuild_test.go
git commit -m "fix(netbuild): allocate one lane slice per edge, not per segment"
```

---

## Task 2: OSM `turn:lanes` token parser

**Files:**
- Create: `internal/netbuild/lanes.go`
- Create: `internal/netbuild/lanes_test.go`

Pure-function token classification: maps OSM token strings to `network.TurnCategory`. Operates on a single per-lane spec (which may contain multiple `;`-separated tokens).

- [ ] **Step 1: Write the failing tests**

Create `internal/netbuild/lanes_test.go`:

```go
package netbuild

import (
	"reflect"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

func TestParseTurnLaneToken(t *testing.T) {
	cases := []struct {
		in   string
		want []network.TurnCategory
	}{
		{"left", []network.TurnCategory{network.TurnLeft}},
		{"slight_left", []network.TurnCategory{network.TurnLeft}},
		{"sharp_left", []network.TurnCategory{network.TurnLeft}},
		{"merge_to_left", []network.TurnCategory{network.TurnLeft}},
		{"right", []network.TurnCategory{network.TurnRight}},
		{"slight_right", []network.TurnCategory{network.TurnRight}},
		{"sharp_right", []network.TurnCategory{network.TurnRight}},
		{"merge_to_right", []network.TurnCategory{network.TurnRight}},
		{"through", []network.TurnCategory{network.TurnStraight}},
		{"none", []network.TurnCategory{network.TurnStraight}},
		{"", []network.TurnCategory{network.TurnStraight}},
		{"through;right", []network.TurnCategory{network.TurnStraight, network.TurnRight}},
		{"left;through;right", []network.TurnCategory{network.TurnLeft, network.TurnStraight, network.TurnRight}},
		{"reverse", nil}, // dropped — U-turns not modeled
		{"floof", nil},   // dropped — unknown
	}
	for _, c := range cases {
		got := parseTurnLaneSpec(c.in)
		if !reflect.DeepEqual(got, c.want) {
			t.Errorf("parseTurnLaneSpec(%q) = %v, want %v", c.in, got, c.want)
		}
	}
}

func TestParseTurnLanesString(t *testing.T) {
	got := parseTurnLanesString("left|through|through;right")
	want := [][]network.TurnCategory{
		{network.TurnLeft},
		{network.TurnStraight},
		{network.TurnStraight, network.TurnRight},
	}
	if !reflect.DeepEqual(got, want) {
		t.Errorf("got %v, want %v", got, want)
	}
}

func TestParseTurnLanesString_Empty(t *testing.T) {
	if got := parseTurnLanesString(""); got != nil {
		t.Errorf("empty input should return nil, got %v", got)
	}
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `go test ./internal/netbuild/ -run TestParseTurnLane -v`
Expected: FAIL — `parseTurnLaneSpec`/`parseTurnLanesString` undefined.

- [ ] **Step 3: Implement the parsers**

Create `internal/netbuild/lanes.go`:

```go
// Package netbuild: lane-to-turn assignment. Populates Lane.AllowedTurns
// for every edge whose downstream node is a multi-edge intersection,
// either from the OSM `turn:lanes=*` tag or via geometric inference.
package netbuild

import (
	"strings"

	"github.com/lab1702/traffic-sim/internal/network"
)

// parseTurnLaneSpec parses one lane's spec from an OSM turn:lanes string.
// A spec can list multiple turn types separated by ';'. Unknown tokens
// are dropped. An empty spec ("" or "none") maps to TurnStraight.
func parseTurnLaneSpec(spec string) []network.TurnCategory {
	if spec == "" || spec == "none" {
		return []network.TurnCategory{network.TurnStraight}
	}
	var out []network.TurnCategory
	seen := map[network.TurnCategory]bool{}
	for _, tok := range strings.Split(spec, ";") {
		tok = strings.TrimSpace(tok)
		c, ok := turnTokenMap[tok]
		if !ok {
			continue
		}
		if !seen[c] {
			out = append(out, c)
			seen[c] = true
		}
	}
	return out
}

// parseTurnLanesString parses a full OSM turn:lanes value (pipe-delimited
// per-lane specs). Returns one entry per lane. Returns nil for empty input.
func parseTurnLanesString(s string) [][]network.TurnCategory {
	if s == "" {
		return nil
	}
	parts := strings.Split(s, "|")
	out := make([][]network.TurnCategory, len(parts))
	for i, p := range parts {
		out[i] = parseTurnLaneSpec(p)
	}
	return out
}

var turnTokenMap = map[string]network.TurnCategory{
	"left":           network.TurnLeft,
	"slight_left":    network.TurnLeft,
	"sharp_left":     network.TurnLeft,
	"merge_to_left":  network.TurnLeft,
	"right":          network.TurnRight,
	"slight_right":   network.TurnRight,
	"sharp_right":    network.TurnRight,
	"merge_to_right": network.TurnRight,
	"through":        network.TurnStraight,
	// "none" and "" handled by parseTurnLaneSpec directly.
	// "reverse" intentionally absent — U-turns are dropped.
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `go test ./internal/netbuild/ -run TestParseTurnLane -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/netbuild/lanes.go internal/netbuild/lanes_test.go
git commit -m "feat(netbuild): OSM turn:lanes token parser"
```

---

## Task 3: Geometric fallback assignment

**Files:**
- Modify: `internal/netbuild/lanes.go`
- Modify: `internal/netbuild/lanes_test.go`

Pure function: given the set of turn categories present at an intersection and a lane count, return per-lane allowed categories.

- [ ] **Step 1: Write the failing test**

Append to `internal/netbuild/lanes_test.go`:

```go
func TestGeometricLaneAssignment(t *testing.T) {
	L, S, R := network.TurnLeft, network.TurnStraight, network.TurnRight

	cases := []struct {
		name      string
		cats      []network.TurnCategory // set of categories present (order ignored)
		numLanes  int
		want      [][]network.TurnCategory // per-lane allowed categories
	}{
		{
			name: "one-lane gets everything",
			cats: []network.TurnCategory{L, S, R}, numLanes: 1,
			want: [][]network.TurnCategory{{L, S, R}},
		},
		{
			name: "two-lane with L+S+R",
			cats: []network.TurnCategory{L, S, R}, numLanes: 2,
			want: [][]network.TurnCategory{{R, S}, {L, S}},
		},
		{
			name: "three-lane with L+S+R",
			cats: []network.TurnCategory{L, S, R}, numLanes: 3,
			want: [][]network.TurnCategory{{R}, {S}, {L}},
		},
		{
			name: "four-lane with L+S+R",
			cats: []network.TurnCategory{L, S, R}, numLanes: 4,
			want: [][]network.TurnCategory{{R}, {S}, {S}, {L}},
		},
		{
			name: "two-lane with S+R only",
			cats: []network.TurnCategory{S, R}, numLanes: 2,
			want: [][]network.TurnCategory{{R, S}, {S}},
		},
		{
			name: "two-lane with S+L only",
			cats: []network.TurnCategory{S, L}, numLanes: 2,
			want: [][]network.TurnCategory{{S}, {L, S}},
		},
		{
			name: "two-lane with single category (straight)",
			cats: []network.TurnCategory{S}, numLanes: 2,
			want: [][]network.TurnCategory{{S}, {S}},
		},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			got := assignLanesGeometric(c.cats, c.numLanes)
			if !equalAssignments(got, c.want) {
				t.Errorf("got %v, want %v", got, c.want)
			}
		})
	}
}

// equalAssignments compares two per-lane assignments treating each lane's
// category list as an unordered set.
func equalAssignments(a, b [][]network.TurnCategory) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if !sameSet(a[i], b[i]) {
			return false
		}
	}
	return true
}

func sameSet(a, b []network.TurnCategory) bool {
	if len(a) != len(b) {
		return false
	}
	seen := map[network.TurnCategory]int{}
	for _, x := range a {
		seen[x]++
	}
	for _, x := range b {
		seen[x]--
		if seen[x] < 0 {
			return false
		}
	}
	return true
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./internal/netbuild/ -run TestGeometricLaneAssignment -v`
Expected: FAIL — `assignLanesGeometric` undefined.

- [ ] **Step 3: Implement geometric assignment**

Append to `internal/netbuild/lanes.go`:

```go
// assignLanesGeometric returns a per-lane list of allowed turn categories
// for an intersection where the given set of categories is present.
// Convention: lane 0 = rightmost; higher index = closer to road centerline.
// The input `cats` is treated as a set (duplicates ignored, order ignored).
func assignLanesGeometric(cats []network.TurnCategory, numLanes int) [][]network.TurnCategory {
	if numLanes <= 0 {
		return nil
	}
	hasL, hasS, hasR := false, false, false
	for _, c := range cats {
		switch c {
		case network.TurnLeft:
			hasL = true
		case network.TurnStraight:
			hasS = true
		case network.TurnRight:
			hasR = true
		}
	}
	out := make([][]network.TurnCategory, numLanes)

	// One-lane edge gets everything that's present.
	if numLanes == 1 {
		var all []network.TurnCategory
		if hasR {
			all = append(all, network.TurnRight)
		}
		if hasS {
			all = append(all, network.TurnStraight)
		}
		if hasL {
			all = append(all, network.TurnLeft)
		}
		out[0] = all
		return out
	}

	last := numLanes - 1
	for i := range out {
		// Default: middle lanes get straight.
		if hasS {
			out[i] = []network.TurnCategory{network.TurnStraight}
		}
	}
	if hasR {
		// Rightmost lane gets right turns (and keeps straight if present).
		if hasS {
			out[0] = []network.TurnCategory{network.TurnRight, network.TurnStraight}
		} else {
			out[0] = []network.TurnCategory{network.TurnRight}
		}
	}
	if hasL {
		// Leftmost lane gets left turns (and keeps straight if present).
		if hasS {
			out[last] = []network.TurnCategory{network.TurnLeft, network.TurnStraight}
		} else {
			out[last] = []network.TurnCategory{network.TurnLeft}
		}
	}
	// Special case: 3-lane with L+S+R — middle gets straight only, edges get
	// only their turn (no straight overlap). Strip straight from edge lanes.
	if numLanes == 3 && hasL && hasS && hasR {
		out[0] = []network.TurnCategory{network.TurnRight}
		out[1] = []network.TurnCategory{network.TurnStraight}
		out[2] = []network.TurnCategory{network.TurnLeft}
	}
	return out
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `go test ./internal/netbuild/ -run TestGeometricLaneAssignment -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/netbuild/lanes.go internal/netbuild/lanes_test.go
git commit -m "feat(netbuild): geometric lane-to-turn fallback assignment"
```

---

## Task 4: Per-edge lane assignment (orchestration)

**Files:**
- Modify: `internal/netbuild/lanes.go`
- Modify: `internal/netbuild/lanes_test.go`

Combine the OSM parser and geometric fallback into a single function that takes an incoming edge and its intersection's outgoing edges, returns the `AllowedTurns` to store per lane.

- [ ] **Step 1: Write the failing test**

Append to `internal/netbuild/lanes_test.go`:

```go
// buildTestNet constructs a simple intersection net: incoming edge 0 ends
// at node 0, with three outgoing edges 1 (right), 2 (straight), 3 (left).
// Lane count on the incoming edge is configurable.
func buildTestIntersection(numLanes int) *network.Network {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: -100, Y: 0}},  // upstream of incoming
		{ID: 2, Pos: network.Point{X: 100, Y: -100}}, // right destination
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},    // straight destination
		{ID: 4, Pos: network.Point{X: 100, Y: 100}},  // left destination
	}
	lanes := make([]network.Lane, numLanes)
	for i := range lanes {
		lanes[i] = network.Lane{Index: uint8(i)}
	}
	edges := []network.Edge{
		// Incoming: node 1 -> 0, heading east (+X).
		{ID: 0, From: 1, To: 0, Length: 100, SpeedLimit: 10, Lanes: lanes,
			Geometry: []network.Point{nodes[1].Pos, nodes[0].Pos}},
		// Outgoing right: 0 -> 2, heading SE (then south).
		{ID: 1, From: 0, To: 2, Length: 140, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[0].Pos, nodes[2].Pos}},
		// Outgoing straight: 0 -> 3, heading east.
		{ID: 2, From: 0, To: 3, Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[0].Pos, nodes[3].Pos}},
		// Outgoing left: 0 -> 4, heading NE (then north).
		{ID: 3, From: 0, To: 4, Length: 140, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[0].Pos, nodes[4].Pos}},
	}
	intersections := []network.Intersection{
		{ID: 0, NodeID: 0,
			Incoming: []network.EdgeID{0},
			Outgoing: []network.EdgeID{1, 2, 3},
		},
	}
	return &network.Network{Nodes: nodes, Edges: edges, Intersections: intersections}
}

func TestAssignAllowedTurnsForEdge_GeometricFallback_TwoLanes(t *testing.T) {
	net := buildTestIntersection(2)
	// No OSM way ID for incoming → geometric fallback.
	allowed := assignAllowedTurnsForEdge(net, 0, &net.Intersections[0], "", nil)

	// Expected: lane 0 = {right, straight}, lane 1 = {left, straight}.
	if len(allowed) != 2 {
		t.Fatalf("want 2 lane assignments, got %d", len(allowed))
	}
	if !containsEdge(allowed[0], 1) || !containsEdge(allowed[0], 2) {
		t.Errorf("lane 0 should include right (1) and straight (2), got %v", allowed[0])
	}
	if containsEdge(allowed[0], 3) {
		t.Errorf("lane 0 should NOT include left (3), got %v", allowed[0])
	}
	if !containsEdge(allowed[1], 3) || !containsEdge(allowed[1], 2) {
		t.Errorf("lane 1 should include left (3) and straight (2), got %v", allowed[1])
	}
	if containsEdge(allowed[1], 1) {
		t.Errorf("lane 1 should NOT include right (1), got %v", allowed[1])
	}
}

func TestAssignAllowedTurnsForEdge_GeometricFallback_ThreeLanes(t *testing.T) {
	net := buildTestIntersection(3)
	allowed := assignAllowedTurnsForEdge(net, 0, &net.Intersections[0], "", nil)

	// Expected: lane 0 = {right}, lane 1 = {straight}, lane 2 = {left}.
	if !containsEdge(allowed[0], 1) || containsEdge(allowed[0], 2) || containsEdge(allowed[0], 3) {
		t.Errorf("lane 0 wrong: %v", allowed[0])
	}
	if containsEdge(allowed[1], 1) || !containsEdge(allowed[1], 2) || containsEdge(allowed[1], 3) {
		t.Errorf("lane 1 wrong: %v", allowed[1])
	}
	if containsEdge(allowed[2], 1) || containsEdge(allowed[2], 2) || !containsEdge(allowed[2], 3) {
		t.Errorf("lane 2 wrong: %v", allowed[2])
	}
}

func TestAssignAllowedTurnsForEdge_OSMOverride(t *testing.T) {
	net := buildTestIntersection(2)
	// OSM says lane 0 = through+right, lane 1 = left only (matches geometric here,
	// but supply explicit data to confirm it's actually used).
	spec := parseTurnLanesString("through;right|left")
	allowed := assignAllowedTurnsForEdge(net, 0, &net.Intersections[0], "", spec)

	if !containsEdge(allowed[0], 1) || !containsEdge(allowed[0], 2) {
		t.Errorf("lane 0 should have right (1) and straight (2), got %v", allowed[0])
	}
	if !containsEdge(allowed[1], 3) {
		t.Errorf("lane 1 should have left (3), got %v", allowed[1])
	}
	if containsEdge(allowed[1], 2) {
		t.Errorf("lane 1 should NOT have straight (2) per explicit OSM, got %v", allowed[1])
	}
}

func TestAssignAllowedTurnsForEdge_BannedTurnExcluded(t *testing.T) {
	net := buildTestIntersection(2)
	// Ban the left turn (0 -> 3).
	net.Intersections[0].BannedTurns = []network.TurnRestriction{
		{From: 0, To: 3},
	}
	allowed := assignAllowedTurnsForEdge(net, 0, &net.Intersections[0], "", nil)

	for i, lane := range allowed {
		if containsEdge(lane, 3) {
			t.Errorf("lane %d should not include banned edge 3, got %v", i, lane)
		}
	}
}

func TestAssignAllowedTurnsForEdge_AllOutgoingReachable(t *testing.T) {
	net := buildTestIntersection(3)
	allowed := assignAllowedTurnsForEdge(net, 0, &net.Intersections[0], "", nil)
	for _, want := range []network.EdgeID{1, 2, 3} {
		reachable := false
		for _, lane := range allowed {
			if containsEdge(lane, want) {
				reachable = true
				break
			}
		}
		if !reachable {
			t.Errorf("edge %d unreachable from any lane", want)
		}
	}
}

func containsEdge(set []network.EdgeID, want network.EdgeID) bool {
	for _, e := range set {
		if e == want {
			return true
		}
	}
	return false
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `go test ./internal/netbuild/ -run TestAssignAllowedTurnsForEdge -v`
Expected: FAIL — `assignAllowedTurnsForEdge` undefined.

- [ ] **Step 3: Implement the orchestrator**

Append to `internal/netbuild/lanes.go`:

```go
// assignAllowedTurnsForEdge computes the AllowedTurns lists for each lane
// of `incoming` at intersection `x`.
//
// - turnLanesTag: the OSM turn:lanes value for the incoming direction
//   (forward or backward, already selected by the caller). Pass "" if
//   the tag is absent.
// - osmSpec: pre-parsed turn:lanes tokens (nil if no OSM data). When the
//   token count matches the lane count, this overrides geometric inference.
//
// Returns one []EdgeID per lane (same order as incoming.Lanes).
func assignAllowedTurnsForEdge(
	net *network.Network,
	incoming network.EdgeID,
	x *network.Intersection,
	turnLanesTag string, // reserved for future logging; current impl uses osmSpec
	osmSpec [][]network.TurnCategory,
) [][]network.EdgeID {
	inc := &net.Edges[incoming]
	numLanes := len(inc.Lanes)
	if numLanes == 0 {
		return nil
	}

	// Build banned-set keyed by outgoing edge.
	banned := make(map[network.EdgeID]bool)
	for _, br := range x.BannedTurns {
		if br.From == incoming {
			banned[br.To] = true
		}
	}

	// Classify each non-banned, non-U-turn outgoing edge.
	type outInfo struct {
		eid network.EdgeID
		cat network.TurnCategory
	}
	var outs []outInfo
	categoriesPresent := make(map[network.TurnCategory]bool)
	for _, oid := range x.Outgoing {
		if banned[oid] {
			continue
		}
		cat := network.ClassifyTurn(net, incoming, oid)
		if cat == network.TurnUTurn {
			continue
		}
		outs = append(outs, outInfo{oid, cat})
		categoriesPresent[cat] = true
	}
	if len(outs) == 0 {
		return make([][]network.EdgeID, numLanes)
	}

	// One-lane incoming or pass-through (only one allowed outgoing): every
	// lane gets every legal outgoing.
	if numLanes == 1 || len(outs) == 1 {
		all := make([]network.EdgeID, len(outs))
		for i, o := range outs {
			all[i] = o.eid
		}
		result := make([][]network.EdgeID, numLanes)
		for i := range result {
			result[i] = append([]network.EdgeID(nil), all...)
		}
		return result
	}

	// Decide per-lane categories: OSM if usable, else geometric.
	var perLane [][]network.TurnCategory
	if len(osmSpec) == numLanes {
		perLane = osmSpec
	} else {
		var presentList []network.TurnCategory
		for c := range categoriesPresent {
			presentList = append(presentList, c)
		}
		perLane = assignLanesGeometric(presentList, numLanes)
	}

	// Translate per-lane categories to per-lane outgoing edges.
	result := make([][]network.EdgeID, numLanes)
	for i, cats := range perLane {
		for _, o := range outs {
			for _, c := range cats {
				if c == o.cat {
					result[i] = append(result[i], o.eid)
					break
				}
			}
		}
	}

	// Sanity: every non-banned outgoing must be reachable from some lane.
	// If a category is present but unreachable, attach to the closest-side lane.
	for _, o := range outs {
		reachable := false
		for _, lane := range result {
			for _, e := range lane {
				if e == o.eid {
					reachable = true
					break
				}
			}
			if reachable {
				break
			}
		}
		if !reachable {
			target := 0
			if o.cat == network.TurnLeft {
				target = numLanes - 1
			}
			result[target] = append(result[target], o.eid)
		}
	}
	return result
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `go test ./internal/netbuild/ -run TestAssignAllowedTurnsForEdge -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/netbuild/lanes.go internal/netbuild/lanes_test.go
git commit -m "feat(netbuild): per-edge lane-to-turn assignment orchestrator"
```

---

## Task 5: Wire lane assignment into netbuild

**Files:**
- Modify: `internal/netbuild/netbuild.go`
- Modify: `internal/netbuild/lanes.go` (add the netbuild-level pass)
- Modify: `internal/netbuild/netbuild_test.go`

Add a post-processing pass after restrictions are applied that calls `assignAllowedTurnsForEdge` for every incoming edge of every multi-edge intersection.

- [ ] **Step 1: Write the failing test**

Append to `internal/netbuild/netbuild_test.go`:

```go
// Builds a + intersection like TestBuild_PlusShape but verifies that the
// incoming edges at the central intersection have their lane AllowedTurns
// populated (geometric fallback, since no turn:lanes tag).
func TestBuild_PopulatesAllowedTurnsAtIntersections(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0, 0.0),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0, -74.0005),
	}}
	// Two two-way ways crossing at node 5. Set lanes=4 so each direction has 2 lanes.
	primary := mkWay(10, "primary", false, 1, 5, 3)
	primary.Tags = append(primary.Tags, osm.Tag{Key: "lanes", Value: "4"})
	cross := mkWay(20, "primary", false, 2, 5, 4)
	cross.Tags = append(cross.Tags, osm.Tag{Key: "lanes", Value: "4"})
	feat.Ways = []*osm.Way{primary, cross}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	// Find an incoming edge to the central intersection and check that at
	// least one of its lanes has a non-empty AllowedTurns.
	var central *network.Intersection
	for i := range net.Intersections {
		if len(net.Intersections[i].Incoming) >= 3 {
			central = &net.Intersections[i]
			break
		}
	}
	if central == nil {
		t.Fatal("no central intersection found")
	}
	for _, eid := range central.Incoming {
		e := &net.Edges[eid]
		if len(e.Lanes) < 2 {
			continue // skip 1-lane edges (only populated when multi-lane)
		}
		anyPopulated := false
		for _, l := range e.Lanes {
			if len(l.AllowedTurns) > 0 {
				anyPopulated = true
				break
			}
		}
		if !anyPopulated {
			t.Errorf("incoming edge %d has 2 lanes but no AllowedTurns populated", eid)
		}
	}
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./internal/netbuild/ -run TestBuild_PopulatesAllowedTurnsAtIntersections -v`
Expected: FAIL — `AllowedTurns populated` is empty everywhere.

- [ ] **Step 3: Implement the netbuild-level pass**

Append to `internal/netbuild/lanes.go`:

```go
// turnLanesTagForDirection returns the OSM turn:lanes string for an edge
// in the given direction. forward=true uses turn:lanes:forward, falling
// back to turn:lanes; forward=false uses turn:lanes:backward, falling
// back to turn:lanes (which is generally not meaningful for two-way
// roads but is a reasonable last resort).
func turnLanesTagForDirection(tags osm.Tags, forward bool) string {
	directional := "turn:lanes:forward"
	if !forward {
		directional = "turn:lanes:backward"
	}
	var generic string
	for _, t := range tags {
		switch t.Key {
		case directional:
			return t.Value
		case "turn:lanes":
			generic = t.Value
		}
	}
	return generic
}

// populateAllowedTurns runs the per-incoming-edge lane assignment for every
// multi-edge intersection in `net`, writing into Lane.AllowedTurns in place.
// `feat` and `osmWayOfEdge` are used to look up OSM turn:lanes tags.
//
// `edgeIsForward[i]` reports whether edge i was constructed as the forward
// direction of its source OSM way. Two-way ways produce one forward and
// one reverse edge.
func populateAllowedTurns(
	net *network.Network,
	feat *osmload.Features,
	osmWayOfEdge []osm.WayID,
	edgeIsForward []bool,
) {
	for ix := range net.Intersections {
		x := &net.Intersections[ix]
		if len(x.Outgoing) < 2 {
			// Pass-through nodes: nothing to choose; leave AllowedTurns empty.
			continue
		}
		for _, inc := range x.Incoming {
			incEdge := &net.Edges[inc]
			if len(incEdge.Lanes) == 0 {
				continue
			}
			var spec [][]network.TurnCategory
			if int(inc) < len(osmWayOfEdge) {
				wayID := osmWayOfEdge[inc]
				if w, ok := feat.Ways[wayID]; ok && w != nil {
					forward := edgeIsForward[inc]
					tag := turnLanesTagForDirection(w.Tags, forward)
					if tag != "" {
						spec = parseTurnLanesString(tag)
					}
				}
			}
			allowed := assignAllowedTurnsForEdge(net, inc, x, "", spec)
			for li := range incEdge.Lanes {
				if li < len(allowed) {
					incEdge.Lanes[li].AllowedTurns = allowed[li]
				}
			}
		}
	}
}
```

`osmload.Features.Ways` is `[]*osm.Way`. Build a lookup map once at the top of `populateAllowedTurns`. Adjust the function body to:

```go
func populateAllowedTurns(
	net *network.Network,
	feat *osmload.Features,
	osmWayOfEdge []osm.WayID,
	edgeIsForward []bool,
) {
	wayByID := make(map[osm.WayID]*osm.Way, len(feat.Ways))
	for _, w := range feat.Ways {
		wayByID[w.ID] = w
	}
	for ix := range net.Intersections {
		x := &net.Intersections[ix]
		if len(x.Outgoing) < 2 {
			continue
		}
		for _, inc := range x.Incoming {
			incEdge := &net.Edges[inc]
			if len(incEdge.Lanes) == 0 {
				continue
			}
			var spec [][]network.TurnCategory
			if int(inc) < len(osmWayOfEdge) {
				wayID := osmWayOfEdge[inc]
				if w, ok := wayByID[wayID]; ok && w != nil {
					forward := edgeIsForward[inc]
					tag := turnLanesTagForDirection(w.Tags, forward)
					if tag != "" {
						spec = parseTurnLanesString(tag)
					}
				}
			}
			allowed := assignAllowedTurnsForEdge(net, inc, x, "", spec)
			for li := range incEdge.Lanes {
				if li < len(allowed) {
					incEdge.Lanes[li].AllowedTurns = allowed[li]
				}
			}
		}
	}
}
```

- [ ] **Step 4: Track edge direction in netbuild.go**

In `internal/netbuild/netbuild.go`, declare a parallel slice alongside `osmWayOfEdge` (near line 93):

```go
var edgeIsForward []bool
```

In the segment loop:

```go
edges = append(edges, network.Edge{
    ID: network.EdgeID(len(edges)), From: fromID, To: toID,
    Lanes: makeLanes(lanesPerDir), Length: length, SpeedLimit: speedFwd, Geometry: geom,
})
osmWayOfEdge = append(osmWayOfEdge, w.ID)
edgeIsForward = append(edgeIsForward, true)
if !oneway {
    revGeom := reverseGeom(geom)
    edges = append(edges, network.Edge{
        ID: network.EdgeID(len(edges)), From: toID, To: fromID,
        Lanes: makeLanes(lanesPerDir), Length: length, SpeedLimit: speedBwd, Geometry: revGeom,
    })
    osmWayOfEdge = append(osmWayOfEdge, w.ID)
    edgeIsForward = append(edgeIsForward, false)
}
```

Update `keepLargestComponent`'s signature and call site to pass `edgeIsForward` through the prune. The current signature (`internal/netbuild/netbuild.go:405-407`) is:

```go
func keepLargestComponent(nodes []network.Node, edges []network.Edge,
	xs []network.Intersection, segChains [][]network.NodeID, osmWayOfEdge []osm.WayID,
) ([]network.Node, []network.Edge, []network.Intersection, []osm.WayID, int) {
```

Change to:

```go
func keepLargestComponent(nodes []network.Node, edges []network.Edge,
	xs []network.Intersection, segChains [][]network.NodeID,
	osmWayOfEdge []osm.WayID, edgeIsForward []bool,
) ([]network.Node, []network.Edge, []network.Intersection, []osm.WayID, []bool, int) {
```

In the edge-keep loop (around line 460-473), add a parallel append to a new `newEdgeIsForward []bool`:

```go
var newEdges []network.Edge
var newOsmWayOf []osm.WayID
var newEdgeIsForward []bool
for i, e := range edges {
    if !keep(e.From) || !keep(e.To) {
        continue
    }
    e.ID = network.EdgeID(len(newEdges))
    e.From = newNodeID[e.From]
    e.To = newNodeID[e.To]
    newEdges = append(newEdges, e)
    if i < len(osmWayOfEdge) {
        newOsmWayOf = append(newOsmWayOf, osmWayOfEdge[i])
    }
    if i < len(edgeIsForward) {
        newEdgeIsForward = append(newEdgeIsForward, edgeIsForward[i])
    }
}
```

And return `newEdgeIsForward` alongside `newOsmWayOf`.

Update the single call site at `internal/netbuild/netbuild.go:170` to:

```go
nodes, edges, intersections, osmWayOfEdge, edgeIsForward, droppedComponents :=
    keepLargestComponent(nodes, edges, intersections, segChains, osmWayOfEdge, edgeIsForward)
```

- [ ] **Step 5: Add the call to populateAllowedTurns in Build()**

In `internal/netbuild/netbuild.go`, after step 6b (the `applyOSMRestrictions` call near line 176) and before step 7 (spatial grid construction), insert:

```go
// 6c. Populate Lane.AllowedTurns per the turn-aware-lane-choice design.
// Done after restrictions so BannedTurns is authoritative.
tmpNet := &network.Network{Nodes: nodes, Edges: edges, Intersections: intersections}
populateAllowedTurns(tmpNet, feat, osmWayOfEdge, edgeIsForward)
```

The `&network.Network{}` literal is acceptable here because `populateAllowedTurns` and `ClassifyTurn` only read fields that have already been populated (Edges, Intersections, Nodes). Writes go through pointers into the same `edges` slice.

- [ ] **Step 6: Run test to verify it passes**

Run: `go test ./internal/netbuild/ -run TestBuild_PopulatesAllowedTurnsAtIntersections -v`
Expected: PASS.

Also run the full netbuild suite: `go test ./internal/netbuild/`
Expected: all PASS.

- [ ] **Step 7: Commit**

```bash
git add internal/netbuild/lanes.go internal/netbuild/netbuild.go internal/netbuild/netbuild_test.go
git commit -m "feat(netbuild): populate Lane.AllowedTurns at multi-edge intersections"
```

---

## Task 6: Runtime turn-bias in `tryLaneChange`

**Files:**
- Modify: `internal/sim/lanechange.go`
- Modify: `internal/sim/lanechange_test.go`

Add a turn-aware branch that runs before the existing speed-driven logic.

- [ ] **Step 1: Write the failing test**

Append to `internal/sim/lanechange_test.go`:

```go
// TestLaneChange_TurnBias_LeftTurn_MigratesToLeftLane verifies that a
// vehicle on a 2-lane edge whose next route step is a left turn migrates
// from lane 0 to lane 1 within the trigger range.
func TestLaneChange_TurnBias_LeftTurn_MigratesToLeftLane(t *testing.T) {
	// Two edges meeting at node 1. Incoming edge 0 is 200m, 2 lanes.
	// Outgoing edge 1 is a left turn (geometry heads north).
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},     // upstream of incoming
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},   // intersection
		{ID: 2, Pos: network.Point{X: 200, Y: 100}}, // left turn destination
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 10,
			Lanes: []network.Lane{
				// Lane 0: explicitly only goes to edge 99 (not present) — incompatible with edge 1.
				// Non-empty list with no match → laneAllows returns false. Empty list means "any".
				{Index: 0, AllowedTurns: []network.EdgeID{99}},
				{Index: 1, AllowedTurns: []network.EdgeID{1}},
			},
			Geometry: []network.Point{nodes[0].Pos, nodes[1].Pos}},
		{ID: 1, From: 1, To: 2, Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[1].Pos, nodes[2].Pos}},
	}
	net := &network.Network{
		Nodes: nodes, Edges: edges,
		Intersections: []network.Intersection{
			{ID: 0, NodeID: 1,
				Incoming: []network.EdgeID{0},
				Outgoing: []network.EdgeID{1}},
		},
	}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)

	// Vehicle in lane 0, 100m down the incoming edge (100m to intersection).
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, Lane: 0, S: 100, V: 10},
	}
	w.nextID = 2

	for i := 0; i < 60; i++ { // 3 sim seconds
		w.Step()
	}
	v := &w.Vehicles[0]
	if v.Lane != 1 {
		t.Errorf("expected lane 1 (left-compatible) after bias, got lane %d", v.Lane)
	}
}

// TestLaneChange_TurnBias_BlockedBySafetyGap verifies that turn bias does
// NOT commit when the safety gap to a neighbor lane is blocked.
func TestLaneChange_TurnBias_BlockedBySafetyGap(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		{ID: 2, Pos: network.Point{X: 200, Y: 100}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 10,
			Lanes: []network.Lane{
				{Index: 0, AllowedTurns: []network.EdgeID{99}}, // sentinel: incompatible with edge 1
				{Index: 1, AllowedTurns: []network.EdgeID{1}},
			},
			Geometry: []network.Point{nodes[0].Pos, nodes[1].Pos}},
		{ID: 1, From: 1, To: 2, Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[1].Pos, nodes[2].Pos}},
	}
	net := &network.Network{
		Nodes: nodes, Edges: edges,
		Intersections: []network.Intersection{
			{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0}, Outgoing: []network.EdgeID{1}},
		},
	}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)

	// Ego in lane 0 at S=100, V=10. Blocker in lane 1 right next to ego
	// (front gap < safetyGapFront), moving at the same speed.
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, Lane: 0, S: 100, V: 10},
		{ID: 2, Route: []network.EdgeID{0, 1}, Edge: 0, Lane: 1, S: 105, V: 10},
	}
	w.nextID = 3

	// Run 5 ticks (0.25s). Not long enough for the blocker to clear, so the
	// bias must stay blocked.
	for i := 0; i < 5; i++ {
		w.Step()
	}
	if w.Vehicles[0].Lane != 0 {
		t.Errorf("ego should still be in lane 0 (gap blocked); got lane %d", w.Vehicles[0].Lane)
	}
}

// TestLaneChange_TurnBias_BeyondTrigger_NoChange verifies bias does not
// fire when the vehicle is more than 300 m from the intersection.
func TestLaneChange_TurnBias_BeyondTrigger_NoChange(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 1000, Y: 0}},
		{ID: 2, Pos: network.Point{X: 1000, Y: 100}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 1000, SpeedLimit: 10,
			Lanes: []network.Lane{
				{Index: 0, AllowedTurns: []network.EdgeID{99}}, // sentinel: incompatible with edge 1
				{Index: 1, AllowedTurns: []network.EdgeID{1}},
			},
			Geometry: []network.Point{nodes[0].Pos, nodes[1].Pos}},
		{ID: 1, From: 1, To: 2, Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[1].Pos, nodes[2].Pos}},
	}
	net := &network.Network{
		Nodes: nodes, Edges: edges,
		Intersections: []network.Intersection{
			{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0}, Outgoing: []network.EdgeID{1}},
		},
	}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)

	// Vehicle at S=100, so 900m to intersection > 300m trigger.
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, Lane: 0, S: 100, V: 10},
	}
	w.nextID = 2

	for i := 0; i < 20; i++ { // 1 sim second
		w.Step()
	}
	if w.Vehicles[0].Lane != 0 {
		t.Errorf("bias should not fire beyond trigger; lane changed to %d", w.Vehicles[0].Lane)
	}
}

// TestLaneChange_TurnBias_LastEdge_NoFire verifies bias is a no-op when
// the current edge is the last edge of the route.
func TestLaneChange_TurnBias_LastEdge_NoFire(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 10,
			Lanes: []network.Lane{
				{Index: 0, AllowedTurns: nil},
				{Index: 1, AllowedTurns: []network.EdgeID{99}},
			},
			Geometry: []network.Point{nodes[0].Pos, nodes[1].Pos}},
	}
	net := &network.Network{Nodes: nodes, Edges: edges}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)

	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0}, Edge: 0, Lane: 0, S: 100, V: 10},
	}
	w.nextID = 2

	for i := 0; i < 30; i++ {
		w.Step()
		if w.Vehicles[0].Despawned {
			break
		}
	}
	if !w.Vehicles[0].Despawned && w.Vehicles[0].Lane != 0 {
		t.Errorf("bias must be a no-op on last edge; lane=%d", w.Vehicles[0].Lane)
	}
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `go test ./internal/sim/ -run TestLaneChange_TurnBias -v`
Expected: at least `LeftTurn_MigratesToLeftLane` FAILs (existing code never changes lanes without speed-difference incentive).

- [ ] **Step 3: Add turn-bias to tryLaneChange**

Modify `internal/sim/lanechange.go`. Replace the entire `tryLaneChange` function with:

```go
const turnBiasRange = 300.0 // meters before the intersection

// tryLaneChange mutates v.Lane if a beneficial change is available.
// laneVehicles[lane] is a sorted-by-S slice of vehicle indices on that
// lane of the current edge.
//
// Two modes:
//   - Turn bias: within turnBiasRange of an intersection where v will turn,
//     shift toward the nearest lane whose AllowedTurns includes the next
//     route edge. Skips the speed-difference threshold but keeps safety gaps.
//   - Speed-driven (existing): catch a faster gap on a neighbor lane.
//     When turn bias is active, this still runs only if the ego lane is
//     already compatible, AND candidate lanes that would become
//     incompatible are rejected.
func tryLaneChange(v *Vehicle, vi int, laneVehicles map[uint8][]int, vs []Vehicle, net *network.Network) {
	if v.LaneChangeCooldown > 0 {
		return
	}
	edge := &net.Edges[v.Edge]
	numLanes := uint8(len(edge.Lanes))
	if numLanes < 2 {
		return
	}

	// --- Turn-bias context ---
	var nextE network.EdgeID
	turnContext := false
	if v.RouteIdx+1 < len(v.Route) {
		nextE = v.Route[v.RouteIdx+1]
		dToInt := edge.Length - v.S
		if dToInt <= turnBiasRange && len(edge.Lanes[v.Lane].AllowedTurns) > 0 {
			turnContext = true
		}
	}
	myCompatible := !turnContext || laneAllows(edge.Lanes[v.Lane].AllowedTurns, nextE)

	// --- Turn bias branch: ego in incompatible lane, must migrate ---
	if turnContext && !myCompatible {
		target, dl, ok := nearestCompatibleLane(edge.Lanes, v.Lane, nextE)
		if !ok {
			return // no compatible lane (shouldn't happen post-Step-4)
		}
		_ = target
		// Step one lane at a time.
		nl := int(v.Lane) + int(dl)
		if nl < 0 || nl >= int(numLanes) {
			return
		}
		other := laneVehicles[uint8(nl)]
		frontS, hasFront := nextAheadS(other, vs, v.S)
		rearS, hasRear := nextBehindS(other, vs, v.S)
		if hasFront && frontS-v.S-VehicleLength < safetyGapFront {
			return
		}
		if hasRear && v.S-rearS-VehicleLength < safetyGapRear {
			return
		}
		v.Lane = uint8(nl)
		v.LaneChangeCooldown = laneChangeCooldown
		return
	}

	// --- Speed-driven (existing logic) ---
	myLane := v.Lane
	same := laneVehicles[myLane]
	var myPos int = -1
	for i, idx := range same {
		if idx == vi {
			myPos = i
			break
		}
	}
	if myPos < 0 {
		return
	}
	var leaderV float64 = edge.SpeedLimit
	var leaderS float64 = edge.Length + 1e6
	if myPos+1 < len(same) {
		ld := &vs[same[myPos+1]]
		leaderV, leaderS = ld.V, ld.S
	}
	leaderGap := leaderS - v.S - VehicleLength
	if leaderGap > laneChangeCheckGap || edge.SpeedLimit-leaderV < vDiffThreshold {
		return
	}

	for _, dl := range []int8{-1, 1} {
		nl := int(myLane) + int(dl)
		if nl < 0 || nl >= int(numLanes) {
			continue
		}
		// Turn-aware suppression: when in compatible lane near a turn,
		// don't switch to a lane that's incompatible with the next turn.
		if turnContext && !laneAllows(edge.Lanes[nl].AllowedTurns, nextE) {
			continue
		}
		other := laneVehicles[uint8(nl)]
		frontS, hasFront := nextAheadS(other, vs, v.S)
		rearS, hasRear := nextBehindS(other, vs, v.S)
		if hasFront && frontS-v.S-VehicleLength < safetyGapFront {
			continue
		}
		if hasRear && v.S-rearS-VehicleLength < safetyGapRear {
			continue
		}
		v.Lane = uint8(nl)
		v.LaneChangeCooldown = laneChangeCooldown
		return
	}
}

// laneAllows reports whether `nextE` is in the lane's AllowedTurns list.
// Empty list means "any outgoing edge" (per the network.Lane schema doc).
func laneAllows(allowed []network.EdgeID, nextE network.EdgeID) bool {
	if len(allowed) == 0 {
		return true
	}
	for _, e := range allowed {
		if e == nextE {
			return true
		}
	}
	return false
}

// nearestCompatibleLane returns the index, direction step (±1), and ok
// flag for the lane closest to `fromLane` whose AllowedTurns includes
// `nextE`. Tie-breaks toward lower index (rightmost side).
func nearestCompatibleLane(lanes []network.Lane, fromLane uint8, nextE network.EdgeID) (uint8, int8, bool) {
	bestIdx := uint8(0)
	bestDist := 1 << 30
	found := false
	for i, l := range lanes {
		if !laneAllows(l.AllowedTurns, nextE) {
			continue
		}
		d := int(i) - int(fromLane)
		ad := d
		if ad < 0 {
			ad = -ad
		}
		// Tie-break toward lower index.
		if ad < bestDist || (ad == bestDist && uint8(i) < bestIdx) {
			bestDist = ad
			bestIdx = uint8(i)
			found = true
		}
	}
	if !found {
		return 0, 0, false
	}
	dl := int8(1)
	if int(bestIdx) < int(fromLane) {
		dl = -1
	}
	return bestIdx, dl, true
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `go test ./internal/sim/ -run TestLaneChange -v`
Expected: all PASS (the new tests plus the existing `TestLaneChange_OvertakesSlowLeader`).

- [ ] **Step 5: Commit**

```bash
git add internal/sim/lanechange.go internal/sim/lanechange_test.go
git commit -m "feat(sim): turn-aware lane bias in tryLaneChange"
```

---

## Task 7: Intersection snap & lane carry-over

**Files:**
- Modify: `internal/sim/vehicle.go`
- Create: `internal/sim/vehicle_test.go`

When `RouteIdx++` advances to a new edge, classify the just-completed turn and set `v.Lane` on the new edge.

- [ ] **Step 1: Write the failing tests**

Create `internal/sim/vehicle_test.go`:

```go
package sim

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

// 4-way intersection at node 2. Edges:
//
//	0: 1->2 incoming from west (heading east)
//	1: 2->3 right turn (south)
//	2: 2->4 straight (east)
//	3: 2->5 left turn (north)
//
// Outgoing edges have configurable lane counts.
func makeCarryoverNet(outNumLanes int) *network.Network {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}}, // unused placeholder
		{ID: 1, Pos: network.Point{X: -100, Y: 0}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 0, Y: -100}}, // south of 2
		{ID: 4, Pos: network.Point{X: 100, Y: 0}},  // east of 2
		{ID: 5, Pos: network.Point{X: 0, Y: 100}},  // north of 2
	}
	outLanes := make([]network.Lane, outNumLanes)
	for i := range outLanes {
		outLanes[i] = network.Lane{Index: uint8(i)}
	}
	edges := []network.Edge{
		{ID: 0, From: 1, To: 2, Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}, {Index: 1}, {Index: 2}},
			Geometry: []network.Point{nodes[1].Pos, nodes[2].Pos}},
		{ID: 1, From: 2, To: 3, Length: 100, SpeedLimit: 10,
			Lanes:    append([]network.Lane(nil), outLanes...),
			Geometry: []network.Point{nodes[2].Pos, nodes[3].Pos}},
		{ID: 2, From: 2, To: 4, Length: 100, SpeedLimit: 10,
			Lanes:    append([]network.Lane(nil), outLanes...),
			Geometry: []network.Point{nodes[2].Pos, nodes[4].Pos}},
		{ID: 3, From: 2, To: 5, Length: 100, SpeedLimit: 10,
			Lanes:    append([]network.Lane(nil), outLanes...),
			Geometry: []network.Point{nodes[2].Pos, nodes[5].Pos}},
	}
	return &network.Network{Nodes: nodes, Edges: edges}
}

func TestStepIDM_LaneCarryOver_RightTurn_SnapsToLane0(t *testing.T) {
	net := makeCarryoverNet(3) // outgoing edges have 3 lanes
	v := &Vehicle{
		ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, Lane: 2, // far-left lane
		S: 99, V: 10,
	}
	stepIDM(v, 10, 0, 0, false, net, DefaultIDM(), 1.0) // 1s tick: 100 + 10 = 110m total > 100m edge length → crosses
	if v.Edge != 1 {
		t.Fatalf("expected edge 1 after crossing, got %d", v.Edge)
	}
	if v.Lane != 0 {
		t.Errorf("right turn should snap to lane 0, got %d", v.Lane)
	}
}

func TestStepIDM_LaneCarryOver_LeftTurn_SnapsToLastLane(t *testing.T) {
	net := makeCarryoverNet(3)
	v := &Vehicle{
		ID: 1, Route: []network.EdgeID{0, 3}, Edge: 0, Lane: 0,
		S: 99, V: 10,
	}
	stepIDM(v, 10, 0, 0, false, net, DefaultIDM(), 1.0)
	if v.Edge != 3 {
		t.Fatalf("expected edge 3 after crossing, got %d", v.Edge)
	}
	if int(v.Lane) != 2 {
		t.Errorf("left turn should snap to lane N-1 (=2), got %d", v.Lane)
	}
}

func TestStepIDM_LaneCarryOver_Straight_PreservesLane(t *testing.T) {
	net := makeCarryoverNet(3)
	v := &Vehicle{
		ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, Lane: 1,
		S: 99, V: 10,
	}
	stepIDM(v, 10, 0, 0, false, net, DefaultIDM(), 1.0)
	if v.Edge != 2 {
		t.Fatalf("expected edge 2 after crossing, got %d", v.Edge)
	}
	if v.Lane != 1 {
		t.Errorf("straight should preserve lane, got %d", v.Lane)
	}
}

func TestStepIDM_LaneCarryOver_Straight_ClampsWhenNarrowing(t *testing.T) {
	net := makeCarryoverNet(1) // outgoing edges have only 1 lane
	v := &Vehicle{
		ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, Lane: 2, // was in lane 2
		S: 99, V: 10,
	}
	stepIDM(v, 10, 0, 0, false, net, DefaultIDM(), 1.0)
	if v.Lane != 0 {
		t.Errorf("straight onto 1-lane should clamp to 0, got %d", v.Lane)
	}
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `go test ./internal/sim/ -run TestStepIDM_LaneCarryOver -v`
Expected: FAIL — current code preserves `v.Lane` literally, so left/right turns won't snap.

- [ ] **Step 3: Implement the lane carry-over rule**

Modify `internal/sim/vehicle.go`. Replace the route-advance block (currently lines 71-81):

```go
for v.S >= edge.Length {
    v.S -= edge.Length
    v.RouteIdx++
    if v.RouteIdx >= len(v.Route) {
        v.Despawned = true
        v.S = 0
        return
    }
    v.Edge = v.Route[v.RouteIdx]
    edge = &net.Edges[v.Edge]
}
```

With:

```go
for v.S >= edge.Length {
    v.S -= edge.Length
    prevEdge := v.Edge
    prevLane := v.Lane
    v.RouteIdx++
    if v.RouteIdx >= len(v.Route) {
        v.Despawned = true
        v.S = 0
        return
    }
    v.Edge = v.Route[v.RouteIdx]
    edge = &net.Edges[v.Edge]

    // Lane carry-over: pick the new lane based on the just-completed
    // turn. This is both the normal post-turn carry-over AND the snap
    // fallback when bias didn't get us to a compatible lane in time.
    cat := network.ClassifyTurn(net, prevEdge, v.Edge)
    nLanes := uint8(len(edge.Lanes))
    switch cat {
    case network.TurnRight:
        v.Lane = 0
    case network.TurnLeft:
        if nLanes > 0 {
            v.Lane = nLanes - 1
        } else {
            v.Lane = 0
        }
    case network.TurnStraight:
        if uint8(prevLane) >= nLanes && nLanes > 0 {
            v.Lane = nLanes - 1
        } else {
            v.Lane = prevLane
        }
    default: // TurnUTurn or unclassified
        v.Lane = 0
    }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `go test ./internal/sim/ -run TestStepIDM_LaneCarryOver -v`
Expected: all PASS.

Also run full sim suite: `go test ./internal/sim/`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/sim/vehicle.go internal/sim/vehicle_test.go
git commit -m "feat(sim): intersection-snap lane carry-over by turn category"
```

---

## Task 8: Snap-fallback diagnostic warning

**Files:**
- Modify: `internal/sim/vehicle.go`
- Modify: `internal/sim/vehicle_test.go`

When the previous lane was incompatible with the just-taken turn (i.e., bias failed and the snap was a teleport), emit a `slog.Warn`.

- [ ] **Step 1: Write the failing test**

Append to `internal/sim/vehicle_test.go`:

```go
import (
	"bytes"
	"log/slog"
	"strings"
)

func TestStepIDM_LaneCarryOver_EmitsSnapWarning(t *testing.T) {
	net := makeCarryoverNet(3)
	// Populate AllowedTurns so lane 0 is right-only, lane 2 is left-only.
	net.Edges[0].Lanes[0].AllowedTurns = []network.EdgeID{1}
	net.Edges[0].Lanes[1].AllowedTurns = []network.EdgeID{2}
	net.Edges[0].Lanes[2].AllowedTurns = []network.EdgeID{3}

	var buf bytes.Buffer
	prev := slog.Default()
	slog.SetDefault(slog.New(slog.NewTextHandler(&buf, &slog.HandlerOptions{Level: slog.LevelDebug})))
	t.Cleanup(func() { slog.SetDefault(prev) })

	// Vehicle in lane 0 (right-only) trying to go left (edge 3).
	v := &Vehicle{
		ID: 42, Route: []network.EdgeID{0, 3}, Edge: 0, Lane: 0,
		S: 99, V: 10,
	}
	stepIDM(v, 10, 0, 0, false, net, DefaultIDM(), 1.0)

	if !strings.Contains(buf.String(), "turn-lane snap fallback") {
		t.Errorf("expected snap-fallback warning; log was: %s", buf.String())
	}
	if !strings.Contains(buf.String(), "vehicle_id=42") {
		t.Errorf("expected vehicle_id=42 in warning; log was: %s", buf.String())
	}
}

func TestStepIDM_LaneCarryOver_NoWarningWhenBiasSucceeded(t *testing.T) {
	net := makeCarryoverNet(3)
	net.Edges[0].Lanes[0].AllowedTurns = []network.EdgeID{1}
	net.Edges[0].Lanes[1].AllowedTurns = []network.EdgeID{2}
	net.Edges[0].Lanes[2].AllowedTurns = []network.EdgeID{3}

	var buf bytes.Buffer
	prev := slog.Default()
	slog.SetDefault(slog.New(slog.NewTextHandler(&buf, &slog.HandlerOptions{Level: slog.LevelDebug})))
	t.Cleanup(func() { slog.SetDefault(prev) })

	// Vehicle in lane 2 (left-compatible) taking the left turn (edge 3): bias OK.
	v := &Vehicle{
		ID: 43, Route: []network.EdgeID{0, 3}, Edge: 0, Lane: 2,
		S: 99, V: 10,
	}
	stepIDM(v, 10, 0, 0, false, net, DefaultIDM(), 1.0)

	if strings.Contains(buf.String(), "turn-lane snap fallback") {
		t.Errorf("expected NO snap-fallback warning when bias succeeded; log: %s", buf.String())
	}
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./internal/sim/ -run TestStepIDM_LaneCarryOver_EmitsSnapWarning -v`
Expected: FAIL — no warning emitted.

- [ ] **Step 3: Implement the diagnostic**

Modify `internal/sim/vehicle.go`. At the top, add the `log/slog` import if not already present:

```go
import (
    "log/slog"
    "math"

    "github.com/lab1702/traffic-sim/internal/network"
)
```

In the route-advance block (added in Task 7), after determining the turn category but before the `switch cat`, add the snap-fallback check:

```go
cat := network.ClassifyTurn(net, prevEdge, v.Edge)
nLanes := uint8(len(edge.Lanes))

// Diagnostic: warn when the previous lane was incompatible with the
// just-taken turn — bias didn't get us there, so this snap is a teleport.
prevLanes := net.Edges[prevEdge].Lanes
if int(prevLane) < len(prevLanes) {
    allowed := prevLanes[prevLane].AllowedTurns
    if len(allowed) > 0 {
        compat := false
        for _, e := range allowed {
            if e == v.Edge {
                compat = true
                break
            }
        }
        if !compat {
            slog.Warn("turn-lane snap fallback",
                "vehicle_id", v.ID,
                "prev_edge", prevEdge,
                "prev_lane", prevLane,
                "new_edge", v.Edge,
                "turn_cat", cat,
            )
        }
    }
}

switch cat {
// ...rest of the switch as in Task 7
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `go test ./internal/sim/ -run TestStepIDM_LaneCarryOver -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/sim/vehicle.go internal/sim/vehicle_test.go
git commit -m "feat(sim): warn when turn-lane bias fails and snap fallback runs"
```

---

## Task 9: End-to-end snap-rate sanity check

**Files:**
- Modify: `internal/e2e/e2e_test.go`

Make the existing E2E run capture WARN-level logs and assert the snap-fallback count stays below a small threshold. The threshold is intentionally generous — this test catches regressions (e.g., trigger range halved by mistake), not absolute quality.

- [ ] **Step 1: Add the new assertion to the existing E2E test**

In `internal/e2e/e2e_test.go`, replace the existing `slog.SetDefault` / `slog.New` setup (if any) with a buffered handler. If the test currently uses the default global logger, install a capturing handler before the run.

Insert at the start of `TestE2E_RealOSM_HeadlessRun`, before the call to `osmload.Load`:

```go
var logBuf bytes.Buffer
prevLogger := slog.Default()
slog.SetDefault(slog.New(slog.NewTextHandler(&logBuf, &slog.HandlerOptions{Level: slog.LevelWarn})))
t.Cleanup(func() { slog.SetDefault(prevLogger) })
```

Add this import if missing: `"log/slog"`.

After `w.Run(60.0)`, append:

```go
snapWarnings := strings.Count(logBuf.String(), "turn-lane snap fallback")
// Generous bound: < 1 snap warning per spawned vehicle on average. Catches
// regressions where bias range or compatibility check breaks entirely.
spawned := int(w.NextID) // approx; route generation rate is fixed for the seed
if snapWarnings > spawned {
    t.Errorf("snap fallback fired too often: %d warnings vs %d vehicles spawned",
        snapWarnings, spawned)
}
t.Logf("snap fallback warnings: %d / spawned: %d", snapWarnings, spawned)
```

Add this import if missing: `"strings"`.

Note: `w.NextID` may not be exported. If not, expose it via a small `(*World).Spawned() int` accessor, or use `len(w.Vehicles) + despawn_count`. The simplest path: add a counter `w.totalSpawned` incremented in the spawn path, exposed via a method. If the test only needs a coarse denominator, computing `len(w.Vehicles)` (currently alive) is acceptable for the regression check — adjust the comment:

```go
// Use alive-vehicle count as a rough denominator; this is a regression
// guard, not a quality metric. A live vehicle count of N implies at least
// N successful spawns.
denom := len(w.Vehicles) + 1
if snapWarnings > 10*denom {
    t.Errorf("snap fallback fired too often: %d warnings, alive=%d",
        snapWarnings, len(w.Vehicles))
}
t.Logf("snap fallback warnings: %d / alive: %d", snapWarnings, len(w.Vehicles))
```

- [ ] **Step 2: Run the e2e test (requires fixture)**

Run: `TRAFFIC_SIM_E2E_OSM=path/to/fixture.osm.pbf go test -tags=e2e ./internal/e2e/ -run TestE2E_RealOSM_HeadlessRun -v`

If `TRAFFIC_SIM_E2E_OSM` is not set, the test is skipped — that's acceptable for CI environments without the fixture. Locally, the user should have a fixture from `internal/e2e/testdata/README.md`.

Expected (when fixture is available): PASS, with a log line like `snap fallback warnings: <n> / alive: <m>`.

- [ ] **Step 3: Commit**

```bash
git add internal/e2e/e2e_test.go
git commit -m "test(e2e): bound turn-lane snap-fallback rate"
```

---

## Task 10: Final verification

- [ ] **Step 1: Run the full test suite**

Run: `go test ./...`
Expected: all PASS.

Run: `go test -tags=e2e ./internal/e2e/...`
Expected: PASS (or SKIP if no fixture).

Run: `go vet ./...`
Expected: clean.

- [ ] **Step 2: Rebuild binaries**

```bash
go build -o trafficsim.exe ./cmd/trafficsim
go build -o tracereplay.exe ./cmd/tracereplay
```

- [ ] **Step 3: Visual smoke test**

Launch `trafficsim.exe` against a multi-lane fixture. Watch a 2-lane road approaching an intersection where some vehicles need to turn left — they should visibly migrate to lane 1 (innermost) on the approach. Right-turning vehicles should stay in or migrate to lane 0 (curb side). At the intersection, observe that vehicles depart on the correct lane of the new edge.

This step is documented for the user to perform manually; agents executing the plan can mark this step complete after the binary build succeeds.

- [ ] **Step 4: Final commit if anything was touched in Step 3**

Only if visual smoke test surfaced a fix. Otherwise this step is a no-op.

---

## Spec Coverage Check

| Spec section | Covered by |
|---|---|
| Goal / strictness | Tasks 6 (bias), 7 (snap) |
| Plumbing preconditions: per-direction lane slices | Task 1 |
| Plumbing preconditions: osmWayOfEdge | Task 5 (already exists; only adds `edgeIsForward`) |
| Lane assignment Step 1 (classify outgoing) | Task 4 |
| Lane assignment Step 2 (OSM turn:lanes) | Tasks 2, 4, 5 |
| Lane assignment Step 3 (geometric fallback) | Task 3 |
| Lane assignment Step 4 (sanity check) | Task 4 (the "Sanity" block) |
| Runtime bias: trigger check | Task 6 |
| Runtime bias: compatibility check | Task 6 |
| Runtime bias: behavior split | Task 6 |
| Runtime bias: skip threshold but keep safety gaps | Task 6 |
| Intersection snap & carry-over | Task 7 |
| Snap diagnostic warning | Task 8 |
| Testing Layer 1 (lane assignment) | Tasks 2, 3, 4, 5 |
| Testing Layer 2 (turn bias) | Task 6 |
| Testing Layer 3 (snap & carry-over) | Tasks 7, 8 |
| Testing Layer 4 (e2e) | Task 9 |
