# Incident-Level Indicator — Design

**Date:** 2026-05-22
**Status:** Approved (brainstorming phase)

## Goal

When a user Shift+clicks an edge in the live viewer to cycle its incident, make
the result legible: clearly show **what incident level the clicked edge is now
at**, and provide a **color legend** so the overlay colors are interpretable.

Today Shift+click advances the clicked edge's severity
(`none → slowdown → lane-closed → fully-closed → none`) and paints the edge in a
severity color, but there is no readout of which level an edge is at and no key
explaining the colors. This feature adds that feedback without changing the
per-edge cycling interaction.

## Major decisions (from brainstorming)

| Axis | Decision |
|---|---|
| Interaction model | Unchanged — per-edge cycling on Shift+click. No global "selected level" / brush. |
| What the indicator reflects | The clicked edge's own current incident level. |
| Display style | Highlight the clicked edge + show its level in an on-screen panel, mirroring the existing intersection selection + detail panel. |
| Legend | A color key (swatch + label per severity) using the exact overlay colors, anchored bottom-left. |
| Panel contents | `Edge #N`, the incident level, plus a little context (length / speed limit / lane count). |
| Scope | Viewer-only. No changes to sim, trace, snapshot schema, routing, or determinism. |
| Replay | Works in `tracereplay` for free (same `render.Viewport`): the legend always shows, and Shift+click selects/inspects an edge read-only (no injection, since `OnIncident` is nil there). |

## Architecture & data flow

All changes are in the `internal/render` package (the Ebitengine viewer). No new
state crosses the sim/render boundary; the panel reads the clicked edge's level
from the already-published `snapshot.Snapshot.Incidents`, so it reflects the live
state and updates immediately when the same edge is re-clicked.

```
Update (per frame):
  Shift+click edge → hitTestEdge → select edge (hasEdgeSelection, selectedEdge)
                                  → clear intersection selection
                                  → OnIncident(...) if set        (existing)
  plain click intersection → select intersection → clear edge selection (existing + clear)
  right click → clear both selections

Draw (per frame):
  drawRoadBands                                   (existing)
  snap := Buf.Read()                              (existing)
  highlight selected edge (if any)                [NEW] — before overlay
  drawIncidents(snap)                             (existing)
  ... vehicles, signals ...                       (existing)
  DrawHUD(...)                                     (existing)
  DrawIncidentLegend(...)                          [NEW] — bottom-left
  if intersection selected: DrawSelectionPanel    (existing)
  if edge selected:        DrawEdgePanel          [NEW] — same offset
```

Mutual exclusivity: at most one of `hasSelection` (intersection) and
`hasEdgeSelection` (edge) is true, so only one detail panel is ever drawn, in the
same screen region used today.

## Component 1 — Edge selection state (`internal/render/viewport.go`)

Add two fields to `Viewport`, mirroring the existing `hasSelection bool` /
`selectedID network.IntersectionID`:

```go
	// hasEdgeSelection / selectedEdge track an edge the user Shift+clicked, so
	// its current incident level can be highlighted and shown in a panel. At
	// most one of hasSelection (intersection) and hasEdgeSelection (edge) is
	// true at a time.
	hasEdgeSelection bool
	selectedEdge     network.EdgeID
```

### `Update` changes

In the click-without-drag handler, the Shift branch additionally records the
selection and clears the intersection selection:

```go
		if shiftHeld() {
			if eid, ok := v.hitTestEdge(mx, my); ok {
				v.selectedEdge = eid
				v.hasEdgeSelection = true
				v.hasSelection = false // edge and intersection selection are exclusive
				if v.OnIncident != nil {
					v.OnIncident(uint32(eid), nextSeverity(v.severityOf(eid)))
				}
			}
		} else if id, ok := v.hitTestIntersection(mx, my); ok {
			v.selectedID = id
			v.hasSelection = true
			v.hasEdgeSelection = false // exclusive with edge selection
		} else {
			v.hasSelection = false
			v.hasEdgeSelection = false
		}
```

Selecting an edge no longer depends on `OnIncident` being non-nil — so in
`tracereplay` (where `OnIncident` is nil) Shift+click still selects an edge for
read-only inspection; it just doesn't inject anything.

The existing right-click clear is extended to clear the edge selection too:

```go
	if ebiten.IsMouseButtonPressed(ebiten.MouseButtonRight) {
		v.hasSelection = false
		v.hasEdgeSelection = false
	}
```

## Component 2 — Edge highlight (`internal/render/viewport.go`)

A new method draws a selection halo on the selected edge, using the existing
intersection selection color (`color.RGBA{180, 180, 255, 255}`). It is called in
`Draw` immediately after `snap := v.Buf.Read()` and **before**
`v.drawIncidents(screen, snap)`, so a severity overlay (if any) is painted on top
inside the halo:

```go
// drawEdgeSelection strokes the selected edge in the selection color, a couple
// of pixels wider than the incident overlay, so it reads as a highlight halo
// even when the edge also carries a severity color. Uses the From/To node
// fallback for edges with <2 geometry points, like hitTestEdge/drawIncidents.
func (v *Viewport) drawEdgeSelection(screen *ebiten.Image) {
	if !v.hasEdgeSelection || int(v.selectedEdge) >= len(v.Net.Edges) {
		return
	}
	e := &v.Net.Edges[v.selectedEdge]
	pts := e.Geometry
	if len(pts) < 2 {
		pts = []network.Point{v.Net.Nodes[e.From].Pos, v.Net.Nodes[e.To].Pos}
	}
	clr := color.RGBA{180, 180, 255, 255}
	w := float32(e.Width*v.zoom) + 5
	if w < minRoadStrokePx+5 {
		w = minRoadStrokePx + 5
	}
	for j := 0; j+1 < len(pts); j++ {
		x1, y1 := v.toScreen(pts[j].X, pts[j].Y)
		x2, y2 := v.toScreen(pts[j+1].X, pts[j+1].Y)
		vector.StrokeLine(screen, x1, y1, x2, y2, w, clr, true)
	}
}
```

`Draw` call sites:

```go
	snap := v.Buf.Read()
	v.drawEdgeSelection(screen) // halo under the overlay   [NEW]
	v.drawIncidents(screen, snap)
```

## Component 3 — Severity name helper + edge panel + legend (`internal/render/hud.go`)

### `severityName`

```go
// severityName maps a snapshot.Sev* value to a human label for panels/legend.
func severityName(sev uint8) string {
	switch sev {
	case snapshot.SevSlowdown:
		return "slowdown"
	case snapshot.SevLaneClose:
		return "lane closed"
	case snapshot.SevFullClose:
		return "fully closed"
	default:
		return "none"
	}
}
```

### `DrawEdgePanel`

Mirrors `DrawSelectionPanel` (same text style via `ebitenutil.DebugPrintAt`,
same `startY`/`hudLineHeight` layout). Reads the current level from the snapshot
the same way the viewer's `severityOf` does — by scanning `snap.Incidents` — so
the panel reflects live state:

```go
// DrawEdgePanel renders the selected edge's id, current incident level, and a
// little context, mirroring DrawSelectionPanel.
func DrawEdgePanel(screen *ebiten.Image, net *network.Network, snap snapshot.Snapshot, id network.EdgeID, startY int) {
	if int(id) >= len(net.Edges) {
		return
	}
	e := &net.Edges[id]
	sev := uint8(snapshot.SevNone)
	for _, inc := range snap.Incidents {
		if inc.EdgeID == uint32(id) {
			sev = inc.Severity
			break
		}
	}
	lines := []string{
		fmt.Sprintf("Edge #%d", id),
		fmt.Sprintf("  incident: %s", severityName(sev)),
		fmt.Sprintf("  %.0f m, limit %.0f m/s, %d lane(s)", e.Length, e.SpeedLimit, len(e.Lanes)),
	}
	for i, line := range lines {
		ebitenutil.DebugPrintAt(screen, line, 8, startY+i*hudLineHeight)
	}
}
```

### `DrawIncidentLegend`

A small key drawn bottom-left: one colored swatch + label per severity, using the
**exact** colors `drawIncidents` uses, so the key matches the overlay. Swatches
via `vector.DrawFilledRect`, labels via `ebitenutil.DebugPrintAt`.

```go
// DrawIncidentLegend draws a color key for the incident overlay, anchored to the
// bottom-left of the screen. Colors match drawIncidents exactly.
func DrawIncidentLegend(screen *ebiten.Image, screenHeight int) {
	type row struct {
		clr   color.RGBA
		label string
	}
	rows := []row{
		{color.RGBA{240, 180, 0, 220}, "slowdown"},
		{color.RGBA{240, 120, 0, 230}, "lane closed"},
		{color.RGBA{230, 40, 40, 240}, "fully closed"},
	}
	const sw = 10 // swatch size px
	x := 8
	// Stack the rows upward from near the bottom edge.
	yTop := screenHeight - 8 - len(rows)*hudLineHeight
	ebitenutil.DebugPrintAt(screen, "incidents:", x, yTop-hudLineHeight)
	for i, r := range rows {
		y := yTop + i*hudLineHeight
		vector.DrawFilledRect(screen, float32(x), float32(y)+2, sw, sw, r.clr, false)
		ebitenutil.DebugPrintAt(screen, r.label, x+sw+6, y)
	}
}
```

### `Draw` call sites (`viewport.go`)

```go
	DrawHUD(screen, snap.SimTime, len(snap.Vehicles), len(snap.Incidents), viewWidthM, viewHeightM, stats)
	DrawIncidentLegend(screen, v.Height)                               // [NEW]
	if v.hasSelection {
		DrawSelectionPanel(screen, v.Net, snap, v.selectedID, 8+hudLineCount*hudLineHeight+8)
	}
	if v.hasEdgeSelection {                                            // [NEW]
		DrawEdgePanel(screen, v.Net, snap, v.selectedEdge, 8+hudLineCount*hudLineHeight+8)
	}
```

(Because the two selections are mutually exclusive, the two panels never draw at
the same offset simultaneously.)

`hud.go` already imports `fmt`, `image/color`, `ebitenutil`, `vector`, `network`,
and `snapshot` for the existing HUD/panel code — confirm and add any missing
import (`vector` is used by `viewport.go`; verify it's imported in `hud.go`, add
if needed).

## Cross-cutting

### Determinism / sim / trace

Unaffected. This is purely viewer presentation. No sim state, routing, trace
events, or snapshot schema change. `TestWorld_TraceDeterminism` is untouched.

### Replay (`tracereplay`)

`tracereplay` constructs the same `render.Viewport` (`cmd/tracereplay/main.go`)
and does not set `OnIncident`. With Component 1's change, Shift+click there
selects an edge and the panel shows its current (replayed) incident level —
useful read-only inspection — while the legend always shows. No `tracereplay`
code changes are required.

### Performance

Negligible: one extra polyline stroke for the selected edge, one small panel, and
a 3-row legend per frame, all O(1)/O(small). `DrawEdgePanel`'s scan of
`snap.Incidents` is over the active-incident count (typically a handful) and runs
only when an edge is selected.

## Testing

### New test — `internal/render/viewport_test.go`

- `TestSeverityName` — `severityName` returns the right label for `SevNone`,
  `SevSlowdown`, `SevLaneClose`, `SevFullClose` (and an unknown value falls back
  to "none").

### Existing tests

`TestSegDist2`, `TestNextSeverity_Cycles`, `TestHitTestEdge` stay green.

### Drawing functions

`drawEdgeSelection`, `DrawEdgePanel`, and `DrawIncidentLegend` are rendering code
that requires an `*ebiten.Image`; they follow the existing untested-draw
convention in this package (`DrawHUD`, `DrawSelectionPanel`, `drawRoadBands`,
`drawIncidents` have no unit tests). They are covered by `go build`/`go vet`
compile checks and manual viewer verification.

### Suite

`go build ./...`, `go vet ./...`, `go test ./... -count=1` all green.

## Files changed

- `internal/render/viewport.go` — `hasEdgeSelection`/`selectedEdge` fields;
  `Update` edge-selection + exclusivity + right-click clear; `drawEdgeSelection`
  method; `Draw` calls for the halo, legend, and edge panel.
- `internal/render/hud.go` — `severityName`, `DrawEdgePanel`,
  `DrawIncidentLegend` (and any missing import).
- `internal/render/viewport_test.go` — `TestSeverityName`.
- `README.md` — one line in the incidents section noting the selection
  highlight/panel and the color legend.

## Out of scope (deferred)

- A global "selected level" / brush interaction (pick a level, then apply) —
  explicitly rejected in brainstorming in favor of keeping per-edge cycling.
- Floating on-map text labels for each incident edge.
- Showing incident metadata beyond level + basic edge facts (e.g. duration, since
  incidents have no duration in v1).
- Any sim/trace/replay behavior change.
