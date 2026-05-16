# Direction-Tag + Interior-Node Sign Resolution — Phase 2 Design

**Date:** 2026-05-15
**Status:** Approved (brainstorming phase)

## Goal

Close the two pieces of OSM sign-tag handling deferred from Phase 1:

1. **Direction-tag refinement** — when an intersection node carries `highway=stop direction=forward` (or `direction=backward`), apply the sign only to approaches whose direction-on-the-way matches the tag, instead of applying lenently to all approaches.

2. **Interior-node sign resolution** — when a non-intersection shaping node along an approach's way carries `highway=stop` or `highway=give_way`, apply the sign to the specific approach whose edge geometry contains that node. This is the most common real-world OSM tagging pattern (mappers place the sign at the stop-line position, not at the intersection node).

Both pieces close gaps acknowledged in Phase 1's spec under "Out of scope (deferred)."

## Major decisions (from brainstorming)

| Axis | Decision |
|---|---|
| Scope | Both direction-tag refinement AND interior-node sign resolution. |
| Implementation style | Inline per-approach lookups in each new rule. No shared `approachMetadata` map (over-engineered for two consumers). |
| Resolution order | `applyInteriorNodeSign` runs LAST — interior tags win over intersection-node tags for the same approach (interior tags represent the physical stop-line position). |
| Ambiguity at multi-way intersections | `direction=forward` on a node where multiple ways meet applies to all approaches whose direction-on-their-way matches. Stricter than the existing lenient behavior; may over-apply in rare cases where the tag refers to one specific way. |
| Closest-to-X tie-break | When multiple sign-tagged interior nodes exist on the same approach (rare), the one closest to X wins. |
| AllWayStop preservation | Non-directional `applyNodeLevelSign` and `applyInteriorNodeSign` skip approaches already promoted to `ControlAllWayStop` (strictest-wins). The directional branch (`direction=forward`/`backward`) does NOT skip — an explicit directional tag from the mapper overrides class-inferred AllWayStop for the matching approach. Phase 1's strictest-wins still applies to non-directional rules. |

## Resolution order

```go
applyClassFallback(x, classOfEdge)                              // Default
applyStopAllOrMinor(x, nodeTags, classOfEdge)                   // stop=all / stop=minor
applyNodeLevelSign(x, nodeTags, ..., direction-aware)           // highway=stop on X with direction= refinement
applyInteriorNodeSign(x, ..., interior shaping nodes)           // highway=stop on stop-line node — overrides
```

Strictest-wins for AllWayStop is preserved by skip-guards in the later rules.

## Direction-tag refinement (Section 1)

### Modified `applyNodeLevelSign`

The Phase 1 implementation applies node-level signs lenently to all approaches. Phase 2 extends it to honor a `direction=forward` / `direction=backward` tag on the intersection node:

```go
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
```

### `approachDirectionOnWay` helper

Given an approach edge `eid` arriving at intersection node `xOSMID`, determine whether the vehicle traversed the underlying OSM way in the forward direction (lower-to-higher index in `way.Nodes`) or the backward direction:

```go
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

**Why "fromIdx < xIdx" is forward:** OSM ways store nodes in order; traversing 0 → N-1 is the way's canonical forward direction. A vehicle arriving at X from a node *earlier* in the sequence is moving forward.

## Interior-node sign resolution (Section 2)

### New `applyInteriorNodeSign`

Runs after all other rules. Walks each approach's underlying way between (exclusive) the From intersection and X, looking for shaping nodes tagged `highway=stop` or `highway=give_way`. The CLOSEST-to-X sign wins:

```go
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

### Why closest-to-X wins

The walk starts at `xIdx` and steps *toward* `fromIdx`. The FIRST sign-tagged node encountered is the one geographically nearest to X — the actual stop-line position. Any sign-tagged node further upstream is an unusual tagging artifact and is ignored.

### osmload retention

No osmload change required. Phase 1's `isControlNode` already retains nodes carrying `highway=stop` / `highway=give_way` tags. Interior shaping nodes referenced by a kept way are retained via `want[n.ID]`. Both code paths are unchanged.

## Caller plumbing

`resolveControls` gains one new closure and one new argument:

```go
func resolveControls(
    xs []network.Intersection,
    feat *osmload.Features,
    osmWayOfEdge []osm.WayID,
    osmNodeOf func(network.NodeID) (osm.NodeID, bool),
    edges []network.Edge, // NEW
) {
    // ... existing setup of wayByID, classOfEdge ...

    edgeFromOSM := func(eid network.EdgeID) (osm.NodeID, bool) {
        if int(eid) >= len(edges) {
            return 0, false
        }
        return osmNodeOf(edges[eid].From)
    }

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
}
```

The `Build` call site passes the local `edges` slice into `resolveControls`.

## Cross-cutting

### Determinism

No new randomness or non-deterministic iteration. Walks over `way.Nodes` and `x.Incoming` are slice iterations in stable order. `TestWorld_TraceDeterminism` is unaffected.

### Trace format

Unchanged. Sign resolution affects only the `IncomingControl` slice on intersections, which is built once at netbuild and never written to the trace.

### Performance

All work happens once per intersection at netbuild time. For each intersection with degree D and ways of length L:
- `applyNodeLevelSign`: O(D × L) for direction lookups.
- `applyInteriorNodeSign`: O(D × L) for interior walks.

Typical D ≤ 8 and L ≤ 30; cost is negligible vs the existing netbuild work.

### Renderer

No renderer changes. Sign placement remains invisible at the rendering layer; behavior shows in vehicle motion.

## Testing

### New tests in `internal/netbuild/control_test.go`

- `TestNetbuild_DirectionForward` — 4-way crossing. Intersection node tagged `highway=stop direction=forward` (set on the N-S way's intersection node). Verify only the forward-direction N-S approach gets `ControlStop`; the backward N-S approach and both E-W approaches do not.
- `TestNetbuild_DirectionBackward` — same fixture, `direction=backward`. Opposite N-S approach gets `ControlStop`.
- `TestNetbuild_DirectionMissingStillLenient` — same fixture, no `direction` tag. All approaches at the intersection get `ControlStop` (Phase 1 lenient behavior preserved).
- `TestNetbuild_InteriorNodeStop` — single approach whose way has an interior shaping node tagged `highway=stop` between From and X. That approach gets `ControlStop`. Another approach without an interior sign follows class-fallback.
- `TestNetbuild_InteriorNodeGiveWay` — same as above but `highway=give_way` → `ControlYield`.
- `TestNetbuild_InteriorNodeOverridesIntersectionNode` — intersection node has `highway=give_way` AND one approach has an interior `highway=stop`. The approach with the interior sign gets `ControlStop`; other approaches get `ControlYield` (from the intersection-node tag).
- `TestNetbuild_InteriorNodeDoesNotDowngradeAllWayStop` — `stop=all` on the intersection node promotes everything to `ControlAllWayStop`. An approach with interior `highway=give_way` stays `ControlAllWayStop` (the skip-AllWayStop guard works).
- `TestNetbuild_InteriorNodeClosestToXWins` — approach with two sign-tagged interior nodes: one further from X tagged `highway=stop`, one closer to X tagged `highway=give_way`. Approach gets `ControlYield`.

### Existing tests

`TestNetbuild_HighwayStopOnNode` and `TestNetbuild_HighwayGiveWayOnNode` (Phase 1) use intersection nodes WITHOUT direction tags — Phase 2 preserves their behavior. No fixture updates needed.

## Files changed

- `internal/netbuild/control.go` — modify `applyNodeLevelSign`, add `approachDirectionOnWay`, `applyInteriorNodeSign`, `interiorSignFor`. Update `resolveControls` to plumb `edges` and `edgeFromOSM`.
- `internal/netbuild/netbuild.go` — call site of `resolveControls` passes `edges` slice.
- `internal/netbuild/control_test.go` — 8 new tests.

## Out of scope (deferred)

- Direction-tag disambiguation at multi-way intersections (which way does `direction=forward` refer to). Phase 2 applies to all approaches whose direction-on-their-way matches; this over-applies in rare cases.
- Way-level signs (`stop=yes` on a way) — not standard OSM convention.
- Sign tags on intersection nodes shared with other roads (the "stop sign at a shared corner" case beyond simple two-way intersections).
- Way endpoint nodes tagged with stop signs (uncommon).

## Known follow-ups

After Phase 2 lands, the per-approach Control resolution is essentially complete for the common OSM tagging patterns. The next visible artifact at unsignalized intersections is the over-applies-to-all-matching-direction case for ambiguous `direction=*` tags at multi-way intersections — likely rare enough in real OSM data to defer indefinitely.
