# Direction-Tag + Interior-Node Sign Resolution — Phase 2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the two Phase 1 deferrals: (1) `direction=forward/backward` refinement on intersection-node `highway=stop` / `highway=give_way` signs, (2) interior-node sign resolution where mappers place the sign at the stop-line position rather than the intersection node.

**Architecture:** Modify the existing `applyNodeLevelSign` in `internal/netbuild/control.go` to consult `direction=` on the intersection node and apply only to matching-direction approaches. Add a new `applyInteriorNodeSign` that runs LAST in `resolveControls`, walking each approach edge's underlying OSM way from the intersection back toward its From node looking for sign-tagged interior shaping nodes; the closest-to-X sign wins. Plumb `edges` and an `edgeFromOSM` closure through `resolveControls` so both rules can resolve approach→way→direction.

**Tech Stack:** Go 1.x; existing `github.com/paulmach/osm` dependency only.

**Spec:** `docs/superpowers/specs/2026-05-15-direction-tag-phase-2-design.md`

---

## File map

| File | Change |
|---|---|
| `internal/netbuild/control.go` | Modify `applyNodeLevelSign` to consult `direction=*`. Add helpers `approachDirectionOnWay` and `applyInteriorNodeSign` + `interiorSignFor`. Update `resolveControls` to accept `edges` and build `edgeFromOSM`. |
| `internal/netbuild/netbuild.go` | Update the call to `resolveControls` to pass `edges`. |
| `internal/netbuild/control_test.go` | 8 new tests covering direction-tag refinement and interior-node sign resolution. |

---

## Task 1: Plumb `edges` and `edgeFromOSM` into `resolveControls`

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/netbuild/control.go`
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/netbuild/netbuild.go`

Phase 2 rules need to look up each approach edge's `From` node in OSM-NodeID space to compare against the way's node sequence. Add `edges` and an `edgeFromOSM` closure to `resolveControls`.

- [ ] **Step 1: Update `resolveControls` signature and body in `internal/netbuild/control.go`**

Find:

```go
func resolveControls(
	xs []network.Intersection,
	feat *osmload.Features,
	osmWayOfEdge []osm.WayID,
	osmNodeOf func(network.NodeID) (osm.NodeID, bool),
) {
```

Replace with:

```go
func resolveControls(
	xs []network.Intersection,
	feat *osmload.Features,
	osmWayOfEdge []osm.WayID,
	osmNodeOf func(network.NodeID) (osm.NodeID, bool),
	edges []network.Edge,
) {
```

Then, immediately after the existing `wayByID` and `classOfEdge` setup (and before the `for i := range xs` loop), add:

```go
	edgeFromOSM := func(eid network.EdgeID) (osm.NodeID, bool) {
		if int(eid) >= len(edges) {
			return 0, false
		}
		return osmNodeOf(edges[eid].From)
	}
```

Inside the per-intersection loop, also extract `xOSMID` alongside `nodeTags`:

```go
	for i := range xs {
		x := &xs[i]
		var nodeTags osm.Tags
		var xOSMID osm.NodeID
		if osmID, ok := osmNodeOf(x.NodeID); ok {
			xOSMID = osmID
			if n, ok2 := feat.Nodes[osmID]; ok2 && n != nil {
				nodeTags = n.Tags
			}
		}

		applyClassFallback(x, classOfEdge)
		applyStopAllOrMinor(x, nodeTags, classOfEdge)
		applyNodeLevelSign(x, nodeTags)
	}
```

(Note: we leave `applyNodeLevelSign` with its current single-tag signature for now — Task 2 changes it. We're just plumbing the new locals into scope so they're ready.)

To keep the file compiling, mark the unused `edgeFromOSM` and `xOSMID` to avoid Go's "declared and not used" error — they'll be used in Tasks 2 and 3. Add `_, _ = edgeFromOSM, xOSMID` at the top of the for-loop body, after extracting `xOSMID`:

```go
	for i := range xs {
		x := &xs[i]
		var nodeTags osm.Tags
		var xOSMID osm.NodeID
		if osmID, ok := osmNodeOf(x.NodeID); ok {
			xOSMID = osmID
			if n, ok2 := feat.Nodes[osmID]; ok2 && n != nil {
				nodeTags = n.Tags
			}
		}
		_ = xOSMID // used in Tasks 2/3

		applyClassFallback(x, classOfEdge)
		applyStopAllOrMinor(x, nodeTags, classOfEdge)
		applyNodeLevelSign(x, nodeTags)
	}
```

(Note: `edgeFromOSM` is a closure defined outside the loop; Go won't complain about it being unused since it's an assignable variable, only about declared-and-never-read locals. Just to be safe, also include `_ = edgeFromOSM` once after its declaration.)

- [ ] **Step 2: Update the `Build` call site in `internal/netbuild/netbuild.go`**

Find the existing call to `resolveControls`:

```go
	resolveControls(intersections, feat, osmWayOfEdge, osmNodeOf)
```

Change to:

```go
	resolveControls(intersections, feat, osmWayOfEdge, osmNodeOf, edges)
```

The local `edges` slice is already in scope at this call site.

- [ ] **Step 3: Build**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go build ./...`
Expected: exit 0.

- [ ] **Step 4: Run all tests**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./...`
Expected: PASS. No behavior change yet — `edgeFromOSM` and `xOSMID` are unused.

- [ ] **Step 5: Commit**

```
git add internal/netbuild/control.go internal/netbuild/netbuild.go
git commit -m "feat(netbuild): plumb edges + edgeFromOSM into resolveControls"
```

---

## Task 2: Direction-tag refinement on intersection-node signs

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/netbuild/control.go`
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/netbuild/control_test.go`

TDD: write a test that exposes the missing `direction=forward` behavior, watch it fail, then implement.

- [ ] **Step 1: Write the failing test**

Append to `internal/netbuild/control_test.go`:

```go
// TestNetbuild_DirectionForward: a 4-way crossing where the intersection
// node carries `highway=stop direction=forward`. Only the approach
// traversing the way in the forward direction (lower-to-higher index
// in the way's node sequence) should get ControlStop. The opposing
// approach on the same way AND the approaches on the crossing way
// should NOT be stopped (they may have other Controls from class
// fallback, just not from this tag).
func TestNetbuild_DirectionForward(t *testing.T) {
	// Layout (planar approximation):
	//   N is at lat 40.0010, S at 39.9990 → "lower index first" way
	//   means we list S, X, N as the node sequence; vehicle going from
	//   S to X (heading N) is moving "forward" along the way.
	// Use a primary+residential cross so class-fallback produces
	// ControlNone on the primary approaches; then highway=stop with
	// direction=forward applies on top.
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 39.9990, -74.0005), // S origin (N-S way, forward)
		2: mkNode(2, 40.0010, -74.0005), // N origin (N-S way, backward)
		3: mkNode(3, 40.0000, -74.0010), // W origin (E-W way)
		4: mkNode(4, 40.0000, -74.0000), // E origin
		5: mkNode(5, 40.0000, -74.0005, "highway", "stop", "direction", "forward"),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 5, 2),     // N-S way: forward = S→N
		mkWay(20, "primary", false, 3, 5, 4),     // E-W way: forward = W→E
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Intersections) != 1 {
		t.Fatalf("want 1 intersection, got %d", len(net.Intersections))
	}
	x := net.Intersections[0]

	// Exactly one approach should be ControlStop — the one whose
	// underlying way edge goes from a node with lower way-index to
	// the X node. The other three approaches should NOT be Stop from
	// this tag. (They may be ControlAllWayStop or ControlNone from
	// equal-class fallback.)
	stopCount := 0
	for i := range x.Incoming {
		if x.IncomingControl[i] == network.ControlStop {
			stopCount++
		}
	}
	if stopCount != 1 {
		t.Errorf("direction=forward should mark exactly 1 approach as Stop, got %d", stopCount)
		for i := range x.Incoming {
			t.Logf("  Incoming[%d] edge=%d control=%v", i, x.Incoming[i], x.IncomingControl[i])
		}
	}
}
```

- [ ] **Step 2: Run it — expect failure**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/netbuild/ -run TestNetbuild_DirectionForward -v -count=1`
Expected: FAIL. The current `applyNodeLevelSign` applies the Stop to ALL approaches (4 of 4), not just the forward one.

- [ ] **Step 3: Replace `applyNodeLevelSign` and add `approachDirectionOnWay`**

In `internal/netbuild/control.go`, find the existing `applyNodeLevelSign`:

```go
func applyNodeLevelSign(x *network.Intersection, tags osm.Tags) {
	var target network.Control
	hasSign := false
	for _, t := range tags {
		if t.Key == "highway" && t.Value == "stop" {
			target = network.ControlStop
			hasSign = true
		}
		if t.Key == "highway" && t.Value == "give_way" {
			target = network.ControlYield
			hasSign = true
		}
	}
	if !hasSign {
		return
	}
	for j := range x.IncomingControl {
		if x.IncomingControl[j] == network.ControlAllWayStop {
			continue
		}
		x.IncomingControl[j] = target
	}
}
```

Replace with:

```go
// applyNodeLevelSign handles highway=stop and highway=give_way tags on
// the intersection node. A direction= tag refines which approaches it
// applies to:
//   - direction=forward: only approaches whose direction-on-way is forward.
//   - direction=backward: only approaches whose direction-on-way is backward.
//   - no direction tag: all approaches (Phase 1 lenient behavior).
//
// Skips approaches already promoted to ControlAllWayStop.
func applyNodeLevelSign(
	x *network.Intersection,
	tags osm.Tags,
	wayByID map[osm.WayID]*osm.Way,
	osmWayOfEdge []osm.WayID,
	edgeFromOSM func(network.EdgeID) (osm.NodeID, bool),
	xOSMID osm.NodeID,
) {
	var target network.Control
	hasSign := false
	direction := ""
	for _, t := range tags {
		if t.Key == "highway" && t.Value == "stop" {
			target, hasSign = network.ControlStop, true
		}
		if t.Key == "highway" && t.Value == "give_way" {
			target, hasSign = network.ControlYield, true
		}
		if t.Key == "direction" && (t.Value == "forward" || t.Value == "backward") {
			direction = t.Value
		}
	}
	if !hasSign {
		return
	}
	for j, eid := range x.Incoming {
		if x.IncomingControl[j] == network.ControlAllWayStop {
			continue
		}
		if direction == "" {
			x.IncomingControl[j] = target
			continue
		}
		approachDir := approachDirectionOnWay(eid, xOSMID, wayByID, osmWayOfEdge, edgeFromOSM)
		if approachDir == direction {
			x.IncomingControl[j] = target
		}
	}
}

// approachDirectionOnWay returns "forward" or "backward" for approach
// edge eid arriving at intersection node xOSMID, based on whether the
// edge's From node appears before or after xOSMID in the underlying
// OSM way's node sequence. Returns empty string if the direction
// cannot be determined (way missing, X not in the way, etc.).
func approachDirectionOnWay(
	eid network.EdgeID,
	xOSMID osm.NodeID,
	wayByID map[osm.WayID]*osm.Way,
	osmWayOfEdge []osm.WayID,
	edgeFromOSM func(network.EdgeID) (osm.NodeID, bool),
) string {
	if int(eid) >= len(osmWayOfEdge) {
		return ""
	}
	way, ok := wayByID[osmWayOfEdge[eid]]
	if !ok || way == nil {
		return ""
	}
	fromOSM, ok := edgeFromOSM(eid)
	if !ok {
		return ""
	}
	xIdx, fromIdx := -1, -1
	for i, n := range way.Nodes {
		if n.ID == xOSMID && xIdx < 0 {
			xIdx = i
		}
		if n.ID == fromOSM && fromIdx < 0 {
			fromIdx = i
		}
	}
	if xIdx < 0 || fromIdx < 0 {
		return ""
	}
	if fromIdx < xIdx {
		return "forward"
	}
	if fromIdx > xIdx {
		return "backward"
	}
	return ""
}
```

- [ ] **Step 4: Update the call site in `resolveControls`**

In `internal/netbuild/control.go`, find the call to `applyNodeLevelSign` inside `resolveControls`:

```go
		applyNodeLevelSign(x, nodeTags)
```

Replace with:

```go
		applyNodeLevelSign(x, nodeTags, wayByID, osmWayOfEdge, edgeFromOSM, xOSMID)
```

Also remove the `_ = xOSMID` placeholder (and `_ = edgeFromOSM` if you added it) since both are now used.

- [ ] **Step 5: Run the new test**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/netbuild/ -run TestNetbuild_DirectionForward -v -count=1`
Expected: PASS.

- [ ] **Step 6: Run the full netbuild + repo suite**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./...`
Expected: PASS. The Phase 1 tests `TestNetbuild_HighwayStopOnNode` and `TestNetbuild_HighwayGiveWayOnNode` use intersection nodes WITHOUT direction tags, so they still get the lenient behavior and continue to pass.

- [ ] **Step 7: Commit**

```
git add internal/netbuild/control.go internal/netbuild/control_test.go
git commit -m "feat(netbuild): direction-tag refinement for intersection-node signs"
```

---

## Task 3: `direction=backward` + missing-direction tests

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/netbuild/control_test.go`

Two more direction-tag tests to lock in behavior across the three cases (forward / backward / missing).

- [ ] **Step 1: Append the tests**

```go
// TestNetbuild_DirectionBackward: same fixture as DirectionForward
// but direction=backward. The opposite approach gets ControlStop.
func TestNetbuild_DirectionBackward(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 39.9990, -74.0005),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0000, -74.0010),
		4: mkNode(4, 40.0000, -74.0000),
		5: mkNode(5, 40.0000, -74.0005, "highway", "stop", "direction", "backward"),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 5, 2),
		mkWay(20, "primary", false, 3, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]

	stopCount := 0
	for i := range x.Incoming {
		if x.IncomingControl[i] == network.ControlStop {
			stopCount++
		}
	}
	if stopCount != 1 {
		t.Errorf("direction=backward should mark exactly 1 approach as Stop, got %d", stopCount)
	}
}

// TestNetbuild_DirectionMissingStillLenient: when direction= tag is
// absent, the Phase 1 lenient behavior is preserved — sign applies to
// every approach (subject to the AllWayStop skip).
func TestNetbuild_DirectionMissingStillLenient(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 39.9990, -74.0005),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0000, -74.0010),
		4: mkNode(4, 40.0000, -74.0000),
		5: mkNode(5, 40.0000, -74.0005, "highway", "stop"), // no direction
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 5, 2),
		mkWay(20, "primary", false, 3, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]

	// Without direction, all 4 approaches get ControlStop. (Two ways
	// are equal-class so the class fallback would have produced
	// ControlAllWayStop, but applyNodeLevelSign skips AllWayStop
	// approaches. Verify the result accounts for this.)
	for i, c := range x.IncomingControl {
		if c != network.ControlStop && c != network.ControlAllWayStop {
			t.Errorf("approach %d should be Stop or AllWayStop, got %v", i, c)
		}
	}
}
```

- [ ] **Step 2: Run them**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/netbuild/ -run "TestNetbuild_DirectionBackward|TestNetbuild_DirectionMissingStillLenient" -v -count=1`
Expected: PASS for both.

- [ ] **Step 3: Commit**

```
git add internal/netbuild/control_test.go
git commit -m "test(netbuild): direction=backward and direction-missing cases"
```

---

## Task 4: Interior-node sign resolution

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/netbuild/control.go`
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/netbuild/control_test.go`

TDD: write a test where an interior shaping node carries `highway=stop`, watch it fail, then implement `applyInteriorNodeSign`.

- [ ] **Step 1: Write the failing test**

Append to `internal/netbuild/control_test.go`:

```go
// TestNetbuild_InteriorNodeStop: a way passes through a shaping node
// tagged `highway=stop` between its endpoint and the intersection. The
// approach edge whose geometry contains that node should get
// ControlStop; the other approach (no interior sign) follows fallback.
func TestNetbuild_InteriorNodeStop(t *testing.T) {
	// Layout: an E-W primary road, plus an N-S residential approach.
	// Way 10 = primary E-W, nodes [1, 5, 4]. Node 5 is the intersection.
	// Way 20 = residential N-S, nodes [2, 6, 5]. Node 6 is an interior
	// shaping node tagged `highway=stop` — placed between the way's
	// start (2) and the intersection (5). This is the typical
	// stop-line tagging pattern.
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010),
		2: mkNode(2, 40.0010, -74.0005), // N residential start
		3: mkNode(3, 40.0000, -74.0000),
		4: mkNode(4, 40.0000, -74.0000),                                // unused
		5: mkNode(5, 40.0000, -74.0005),                                // intersection
		6: mkNode(6, 40.0005, -74.0005, "highway", "stop"),             // interior stop-line node
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 5, 3),    // primary E-W
		mkWay(20, "residential", false, 2, 6, 5), // residential, with stop-line at node 6
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Intersections) != 1 {
		t.Fatalf("want 1 intersection, got %d", len(net.Intersections))
	}
	x := net.Intersections[0]

	// The residential approach should be ControlStop (from interior tag).
	// The primary approach(es) follow class-fallback — None on a
	// higher-priority road meeting a residential.
	var sawResidentialStop bool
	for i, eid := range x.Incoming {
		hw := highwayOfEdge(net, eid, feat)
		c := x.IncomingControl[i]
		if hw == "residential" && c == network.ControlStop {
			sawResidentialStop = true
		}
		if hw == "primary" && c == network.ControlStop {
			t.Errorf("primary approach should not be Stop, got %v", c)
		}
	}
	if !sawResidentialStop {
		t.Error("residential approach should be ControlStop from interior-node tag")
	}
}
```

- [ ] **Step 2: Run it — expect failure**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/netbuild/ -run TestNetbuild_InteriorNodeStop -v -count=1`
Expected: FAIL. The residential approach gets `ControlStop` only from class-fallback (primary vs residential), but actually wait — class fallback DOES assign `ControlStop` to lower-class approaches. Let me re-examine.

Class fallback: primary vs residential = unequal classes → residential gets `ControlStop`, primary stays `ControlNone`. So the residential approach IS already `ControlStop` from class fallback. This test would pass by coincidence without the interior-node feature.

Adjust the fixture to make the residential approach NOT get Stop from class fallback. Easiest: use two equal-class ways so class fallback gives `ControlAllWayStop`, then the interior-node Stop "downgrades" from AllWayStop... no wait, the AllWayStop skip-guard prevents that.

Better: use two equal-class roads (both residential). Class fallback gives `ControlAllWayStop` everywhere. The interior-node sign should NOT downgrade AllWayStop (per the skip-guard). This is actually TestNetbuild_InteriorNodeDoesNotDowngradeAllWayStop (Task 6) — useful but doesn't prove the positive case.

Best: use primary + tertiary (also unequal class, so residential becomes the lower one and gets Stop from class fallback). Hmm.

Let me re-think. The ONLY way to verify interior-node Stop "wins" is to set up a case where without it, the approach would NOT be Stop, and with it, the approach IS Stop.

Use: residential + service (both lower-class), but service is even lower than residential. So service gets Stop (class fallback). Residential stays None. Now add interior-stop tag to the residential approach. Expected: residential approach also becomes Stop.

Wait actually that's still complex. Simpler: use TWO ways of the SAME class (so class fallback would give AllWayStop everywhere). But the AllWayStop skip-guard prevents interior-node from overriding. So that doesn't work either.

OK actually best: use a single approach intersection (degree-1, just a dead-end). No: dead-ends don't make intersections.

How about a 3-way T where the through-road and stem are different classes? Like primary E-W + residential stem from N. Class fallback: primary approaches (both directions) stay None; residential stem gets Stop. Now add interior-node `highway=give_way` (Yield, not Stop) on the residential approach. Expected: residential approach becomes Yield (interior wins over class-fallback Stop).

Or even simpler: 4-way primary+primary cross (equal class → AllWayStop). Interior `highway=give_way` should not downgrade. Same AllWayStop test.

You know what, the cleanest test is: a 4-way of two primary roads (equal-class → AllWayStop everywhere). Add an interior `highway=stop` on ONE approach. With AllWayStop skip-guard, the interior-stop has no effect (skipped). Verify: that approach is still AllWayStop. Same as the "doesn't downgrade" test.

Hmm none of these prove the positive case where interior-stop adds Stop where there wasn't one.

Going back to first principles: when WOULD interior-stop ADD a Stop?
- An approach is currently `ControlNone` (priority road), no class-fallback Stop, no intersection-node sign, but has an interior `highway=stop`. After Phase 2: that approach becomes Stop.
- Example: a 3-way where the through-road is primary (both directions are None from class fallback), and the side road is also primary OR the geometry doesn't trigger fallback... but with same class, fallback gives AllWayStop.

OK the cleanest setup: 4-way with the same way (one straight primary road, two through directions), plus an ALL-WAY stop sign disabled by using priority instead. Like one primary + one service. Service gets Stop (class fallback). Primary gets None. Add interior `highway=give_way` on the PRIMARY approach. Verify: primary gets Yield.

Fixture:
- Way 10: primary E-W, nodes [1, 5, 4] (where 5 is intersection).
- Way 20: service N-S, nodes [2, 5, 3].
- Now we want an interior-tagged node on the primary approach. Edge geometry from 1 → 5 has interior shaping nodes? No — there are no shaping nodes in [1, 5]. Edges are point-to-point.

To have an interior shaping node, the way needs MORE nodes between endpoints/intersections. Let's add one: Way 10 nodes = [1, 6, 5, 4]. Node 6 is an interior shaping node on the W→C approach. Tag node 6 with `highway=give_way`.

Then: approach edge from W to C has geometry [1, 6, 5]. Wait — the edge is split at 5 (intersection), so the edge from 1 is to 5. Its geometry is [pos(1), pos(6), pos(5)]. Node 6 is interior to this edge.

With interior-node sign feature, this edge should get ControlYield. Without it, it stays ControlNone (primary class).

This is the cleanest positive-case test fixture. Let me update.

The test fixture I wrote uses residential+primary, where the residential approach already gets Stop from class fallback. That's confused. Let me redo.

Replace the Step 1 test with:

```go
// TestNetbuild_InteriorNodeStop: a primary way has an interior shaping
// node tagged `highway=stop`. The approach edge whose geometry contains
// that node should get ControlStop, overriding the class-fallback
// ControlNone that a primary approach would otherwise have.
func TestNetbuild_InteriorNodeStop(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010), // W primary endpoint
		2: mkNode(2, 40.0010, -74.0005), // N service endpoint
		3: mkNode(3, 40.0000, -74.0000), // E primary endpoint
		4: mkNode(4, 39.9990, -74.0005), // S service endpoint
		5: mkNode(5, 40.0000, -74.0005), // intersection
		6: mkNode(6, 40.0000, -74.0008, "highway", "stop"), // interior shaping node on W approach, tagged stop
	}}
	feat.Ways = []*osm.Way{
		// Primary W-E way passing through node 6 (interior) and 5 (intersection).
		mkWay(10, "primary", false, 1, 6, 5, 3),
		// Service N-S way through the intersection.
		mkWay(20, "service", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Intersections) != 1 {
		t.Fatalf("want 1 intersection, got %d", len(net.Intersections))
	}
	x := net.Intersections[0]

	// The W approach (primary edge with interior stop node) gets Stop.
	// The E approach (primary, no interior sign) stays None per class fallback.
	// The service approaches (lower class) get Stop from class fallback.
	var sawPrimaryStop, sawPrimaryNone bool
	for i, eid := range x.Incoming {
		hw := highwayOfEdge(net, eid, feat)
		c := x.IncomingControl[i]
		if hw == "primary" && c == network.ControlStop {
			sawPrimaryStop = true
		}
		if hw == "primary" && c == network.ControlNone {
			sawPrimaryNone = true
		}
	}
	if !sawPrimaryStop {
		t.Error("primary approach with interior stop-tagged node should be ControlStop")
	}
	if !sawPrimaryNone {
		t.Error("the other primary approach (no interior tag) should remain ControlNone")
	}
}
```

- [ ] **Step 2: Run it — expect failure**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/netbuild/ -run TestNetbuild_InteriorNodeStop -v -count=1`
Expected: FAIL. Without `applyInteriorNodeSign`, the W primary approach stays at `ControlNone` (class fallback). The "sawPrimaryStop" check fails.

- [ ] **Step 3: Implement `applyInteriorNodeSign` and `interiorSignFor`**

In `internal/netbuild/control.go`, add these functions (place them after `applyNodeLevelSign` and its helper, before `applyStopAllOrMinor` or anywhere in the file):

```go
// applyInteriorNodeSign overrides per-approach Control based on
// highway=stop or highway=give_way tags on interior shaping nodes —
// nodes between the approach edge's From intersection and the
// intersection X along the underlying OSM way. Mappers conventionally
// place sign tags at the physical stop-line position rather than at
// the intersection node, so honoring those tags gives per-approach
// precision.
//
// Runs last in the resolution chain so interior tags win over
// intersection-node tags when both apply to the same approach. Skips
// approaches already promoted to ControlAllWayStop.
func applyInteriorNodeSign(
	x *network.Intersection,
	wayByID map[osm.WayID]*osm.Way,
	osmWayOfEdge []osm.WayID,
	edgeFromOSM func(network.EdgeID) (osm.NodeID, bool),
	xOSMID osm.NodeID,
	nodeByID map[osm.NodeID]*osm.Node,
) {
	for j, eid := range x.Incoming {
		if x.IncomingControl[j] == network.ControlAllWayStop {
			continue
		}
		sign := interiorSignFor(eid, xOSMID, wayByID, osmWayOfEdge, edgeFromOSM, nodeByID)
		if sign != network.ControlNone {
			x.IncomingControl[j] = sign
		}
	}
}

// interiorSignFor walks the underlying OSM way's node sequence between
// (exclusive) the approach edge's From node and the intersection node
// xOSMID, looking for the closest sign-tagged interior shaping node.
// Returns ControlStop for highway=stop, ControlYield for highway=give_way,
// or ControlNone if no sign-tagged interior node exists. The walk
// starts at xOSMID and steps toward fromOSM so the FIRST tag encountered
// is the one closest to X (the stop-line position).
func interiorSignFor(
	eid network.EdgeID,
	xOSMID osm.NodeID,
	wayByID map[osm.WayID]*osm.Way,
	osmWayOfEdge []osm.WayID,
	edgeFromOSM func(network.EdgeID) (osm.NodeID, bool),
	nodeByID map[osm.NodeID]*osm.Node,
) network.Control {
	if int(eid) >= len(osmWayOfEdge) {
		return network.ControlNone
	}
	way, ok := wayByID[osmWayOfEdge[eid]]
	if !ok || way == nil {
		return network.ControlNone
	}
	fromOSM, ok := edgeFromOSM(eid)
	if !ok {
		return network.ControlNone
	}
	xIdx, fromIdx := -1, -1
	for i, n := range way.Nodes {
		if n.ID == xOSMID && xIdx < 0 {
			xIdx = i
		}
		if n.ID == fromOSM && fromIdx < 0 {
			fromIdx = i
		}
	}
	if xIdx < 0 || fromIdx < 0 || xIdx == fromIdx {
		return network.ControlNone
	}
	step := -1
	if fromIdx > xIdx {
		step = 1
	}
	for i := xIdx + step; i != fromIdx; i += step {
		n := way.Nodes[i]
		node, ok := nodeByID[n.ID]
		if !ok || node == nil {
			continue
		}
		for _, t := range node.Tags {
			if t.Key == "highway" && t.Value == "stop" {
				return network.ControlStop
			}
			if t.Key == "highway" && t.Value == "give_way" {
				return network.ControlYield
			}
		}
	}
	return network.ControlNone
}
```

- [ ] **Step 4: Add the call to `applyInteriorNodeSign` in `resolveControls`**

In `internal/netbuild/control.go`, find the per-intersection loop body. After the `applyNodeLevelSign(...)` call, add:

```go
		applyInteriorNodeSign(x, wayByID, osmWayOfEdge, edgeFromOSM, xOSMID, feat.Nodes)
```

The full loop body now reads:

```go
	for i := range xs {
		x := &xs[i]
		var nodeTags osm.Tags
		var xOSMID osm.NodeID
		if osmID, ok := osmNodeOf(x.NodeID); ok {
			xOSMID = osmID
			if n, ok2 := feat.Nodes[osmID]; ok2 && n != nil {
				nodeTags = n.Tags
			}
		}

		applyClassFallback(x, classOfEdge)
		applyStopAllOrMinor(x, nodeTags, classOfEdge)
		applyNodeLevelSign(x, nodeTags, wayByID, osmWayOfEdge, edgeFromOSM, xOSMID)
		applyInteriorNodeSign(x, wayByID, osmWayOfEdge, edgeFromOSM, xOSMID, feat.Nodes)
	}
```

- [ ] **Step 5: Run the new test**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/netbuild/ -run TestNetbuild_InteriorNodeStop -v -count=1`
Expected: PASS.

- [ ] **Step 6: Run the full repo suite**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./...`
Expected: PASS.

- [ ] **Step 7: Commit**

```
git add internal/netbuild/control.go internal/netbuild/control_test.go
git commit -m "feat(netbuild): interior-node sign resolution"
```

---

## Task 5: Interior-node give-way + override tests

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/netbuild/control_test.go`

- [ ] **Step 1: Append both tests**

```go
// TestNetbuild_InteriorNodeGiveWay: same shape as InteriorNodeStop but
// the interior tag is highway=give_way → ControlYield.
func TestNetbuild_InteriorNodeGiveWay(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0000, -74.0000),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0000, -74.0005),
		6: mkNode(6, 40.0000, -74.0008, "highway", "give_way"),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 6, 5, 3),
		mkWay(20, "service", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]

	var sawPrimaryYield bool
	for i, eid := range x.Incoming {
		hw := highwayOfEdge(net, eid, feat)
		c := x.IncomingControl[i]
		if hw == "primary" && c == network.ControlYield {
			sawPrimaryYield = true
		}
	}
	if !sawPrimaryYield {
		t.Error("primary approach with interior give_way-tagged node should be ControlYield")
	}
}

// TestNetbuild_InteriorNodeOverridesIntersectionNode: intersection node
// has highway=give_way (which would set Yield on all approaches via
// Section 1) AND one approach has an interior highway=stop. The
// approach with the interior sign gets Stop (interior wins); other
// approaches get Yield.
func TestNetbuild_InteriorNodeOverridesIntersectionNode(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0000, -74.0000),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0000, -74.0005, "highway", "give_way"), // intersection give_way
		6: mkNode(6, 40.0000, -74.0008, "highway", "stop"),    // interior on W approach
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 6, 5, 3),
		mkWay(20, "service", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]

	// W approach (primary, interior stop) -> Stop (interior wins).
	// E approach (primary, no interior) -> Yield (from intersection-node tag).
	// Service approaches -> Yield (from intersection-node tag).
	stopCount, yieldCount := 0, 0
	for i := range x.Incoming {
		switch x.IncomingControl[i] {
		case network.ControlStop:
			stopCount++
		case network.ControlYield:
			yieldCount++
		}
	}
	if stopCount != 1 {
		t.Errorf("expected exactly 1 Stop (interior wins), got %d", stopCount)
	}
	if yieldCount < 1 {
		t.Errorf("expected ≥1 Yield from intersection-node tag, got %d", yieldCount)
	}
}
```

- [ ] **Step 2: Run them**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/netbuild/ -run "TestNetbuild_InteriorNodeGiveWay|TestNetbuild_InteriorNodeOverridesIntersectionNode" -v -count=1`
Expected: PASS.

- [ ] **Step 3: Commit**

```
git add internal/netbuild/control_test.go
git commit -m "test(netbuild): interior-node give-way + intersection-node override"
```

---

## Task 6: AllWayStop skip + closest-to-X tests

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/netbuild/control_test.go`

- [ ] **Step 1: Append both tests**

```go
// TestNetbuild_InteriorNodeDoesNotDowngradeAllWayStop: stop=all on the
// intersection node promotes every approach to ControlAllWayStop. An
// interior highway=give_way on one approach must NOT downgrade it —
// the AllWayStop skip-guard protects the strictest control from
// being weakened.
func TestNetbuild_InteriorNodeDoesNotDowngradeAllWayStop(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0000, -74.0000),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0000, -74.0005, "stop", "all"),
		6: mkNode(6, 40.0000, -74.0008, "highway", "give_way"),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 6, 5, 3),
		mkWay(20, "service", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]

	for i, c := range x.IncomingControl {
		if c != network.ControlAllWayStop {
			t.Errorf("approach %d should be AllWayStop (stop=all), got %v", i, c)
		}
	}
}

// TestNetbuild_InteriorNodeClosestToXWins: an approach has TWO sign-
// tagged interior nodes. The one geographically closer to X (the
// intersection) wins, because the walk starts at xIdx and steps toward
// fromIdx, returning the first match.
func TestNetbuild_InteriorNodeClosestToXWins(t *testing.T) {
	// Way: 1 (start) -> 7 (far interior, highway=stop) -> 6 (near
	// interior, highway=give_way) -> 5 (intersection) -> 3.
	// The W approach should get ControlYield (node 6 is closer to 5).
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0020),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0000, -74.0000),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0000, -74.0005),
		6: mkNode(6, 40.0000, -74.0008, "highway", "give_way"), // closer to 5
		7: mkNode(7, 40.0000, -74.0015, "highway", "stop"),     // farther from 5
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 7, 6, 5, 3),
		mkWay(20, "service", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]

	var sawPrimaryYield bool
	for i, eid := range x.Incoming {
		hw := highwayOfEdge(net, eid, feat)
		c := x.IncomingControl[i]
		if hw == "primary" && c == network.ControlYield {
			sawPrimaryYield = true
		}
		if hw == "primary" && c == network.ControlStop {
			t.Errorf("primary approach should be Yield (closest interior tag wins), got Stop")
		}
	}
	if !sawPrimaryYield {
		t.Error("primary approach should be ControlYield from closer interior node")
	}
}
```

- [ ] **Step 2: Run them**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/netbuild/ -run "TestNetbuild_InteriorNodeDoesNotDowngradeAllWayStop|TestNetbuild_InteriorNodeClosestToXWins" -v -count=1`
Expected: PASS.

- [ ] **Step 3: Commit**

```
git add internal/netbuild/control_test.go
git commit -m "test(netbuild): AllWayStop preservation + closest-interior-node wins"
```

---

## Task 7: Final verification

**Files:** none modified.

- [ ] **Step 1: Full repo suite**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./...`
Expected: all PASS.

- [ ] **Step 2: Determinism**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -run TestWorld_TraceDeterminism -v -count=1`
Expected: PASS. Phase 2 is netbuild-only — sim's RNG sequence is unchanged.

- [ ] **Step 3: Benchmarks**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -bench=BenchmarkTick -benchtime=2s -run=^$`
Expected: ns/op essentially unchanged from post-Phase-3 baseline. Phase 2 affects only netbuild, not the per-tick hot path.

- [ ] **Step 4: No commit (verification only)**

Phase 2 is complete. Direction-tag refinement on intersection-node signs + interior-node sign resolution both in place, with closest-to-X tie-break and AllWayStop preservation.

---

## Out of scope (deferred)

- Direction-tag disambiguation at multi-way intersections (which way does `direction=forward` refer to). Phase 2 applies to all approaches whose direction-on-their-way matches; over-applies in rare ambiguous cases.
- Way-level signs (`stop=yes` on a way) — not standard OSM convention.
- Sign tags on intersection nodes shared with other roads (the "stop sign at a shared corner" case).
- Way endpoint nodes tagged with stop signs (uncommon).
