# Incident-Level Indicator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** After a Shift+click cycles an edge's incident, highlight that edge and show its current incident level in a panel, plus a color legend matching the overlay — so the selected level is unmistakable.

**Architecture:** Viewer-only changes in `internal/render`. New mutually-exclusive edge-selection state on `Viewport` (mirroring intersection selection); a selection halo drawn under the incident overlay; an edge detail panel and a color legend in `hud.go`; a `severityName` helper. No sim/trace/snapshot/determinism changes; it also works read-only in `tracereplay` because it shares the same `Viewport`.

**Tech Stack:** Go, Ebitengine v2 (`ebitenutil` text, `vector` shapes), the existing `snapshot.Snapshot` render hand-off.

**Spec:** `docs/superpowers/specs/2026-05-22-incident-level-indicator-design.md`

**Branch:** work on the existing `feat/incident-level-indicator` branch (already checked out; the spec commit is there). End every commit message body with:
```
Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
```

---

## File Structure

| File | Responsibility | Action |
|---|---|---|
| `internal/render/hud.go` | `severityName` helper, `DrawEdgePanel`, `DrawIncidentLegend` | Modify |
| `internal/render/viewport.go` | edge-selection state, `Update` wiring, `drawEdgeSelection`, `Draw` calls | Modify |
| `internal/render/viewport_test.go` | `TestSeverityName` | Modify |
| `README.md` | one-line note in the incidents section | Modify |

Tasks are ordered so each leaves the package compiling and tests green. Most steps are rendering code that can't be unit-tested with an `*ebiten.Image`; per the spec they follow the existing untested-draw convention (`DrawHUD`, `DrawSelectionPanel`, `drawRoadBands`, `drawIncidents` have no unit tests) and are verified by `go build` + `go vet` + manual viewer use. The one pure helper (`severityName`) is unit-tested.

---

## Task 1: `severityName` helper

**Files:**
- Modify: `internal/render/hud.go` (add `severityName` after `DrawSelectionPanel`, which ends at line 128)
- Test: `internal/render/viewport_test.go` (append)

- [ ] **Step 1: Write the failing test**

Append to `internal/render/viewport_test.go` (the file already imports `testing` and `github.com/lab1702/traffic-sim/internal/snapshot`):

```go
func TestSeverityName(t *testing.T) {
	cases := map[uint8]string{
		snapshot.SevNone:      "none",
		snapshot.SevSlowdown:  "slowdown",
		snapshot.SevLaneClose: "lane closed",
		snapshot.SevFullClose: "fully closed",
		uint8(99):             "none", // unknown falls back to none
	}
	for sev, want := range cases {
		if got := severityName(sev); got != want {
			t.Fatalf("severityName(%d) = %q, want %q", sev, got, want)
		}
	}
}
```

- [ ] **Step 2: Run to confirm it fails**

Run: `go test ./internal/render/ -run TestSeverityName`
Expected: FAIL — `undefined: severityName`.

- [ ] **Step 3: Add the helper**

In `internal/render/hud.go`, after the closing `}` of `DrawSelectionPanel` (line 128), add:

```go
// severityName maps a snapshot.Sev* value to a human label for the edge panel
// and the incident legend.
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

- [ ] **Step 4: Run to verify it passes**

Run: `go test ./internal/render/ -run TestSeverityName -v`
Expected: PASS.
Also run `go test ./internal/render/ -count=1` (whole package) — PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/render/hud.go internal/render/viewport_test.go
git commit -m "$(cat <<'EOF'
feat(render): add severityName helper for incident labels

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: Edge selection state + Update wiring

**Files:**
- Modify: `internal/render/viewport.go` (struct fields; the click-without-drag block; the right-click clear)

This task records which edge was Shift+clicked so later tasks can highlight it and show its level. There is no unit test (the behavior is mouse/keyboard-driven inside Ebitengine's `Update`); it is verified by `go build`/`go vet` and by the later visible behavior.

- [ ] **Step 1: Add the selection-state fields**

In `internal/render/viewport.go`, the `Viewport` struct has (around lines 138-139):

```go
	hasSelection bool
	selectedID   network.IntersectionID
```

Immediately after those two lines, add:

```go
	// hasEdgeSelection / selectedEdge track an edge the user Shift+clicked, so
	// its current incident level can be highlighted and shown in a panel. At
	// most one of hasSelection (intersection) and hasEdgeSelection (edge) is
	// true at a time.
	hasEdgeSelection bool
	selectedEdge     network.EdgeID
```

- [ ] **Step 2: Wire edge selection into the click handler**

In `Update`, find this existing block (the click-without-drag handler):

```go
		if v.dragging && !v.movedSinceDown {
			// Shift+click cycles an incident on the nearest edge; plain click
			// selects an intersection.
			if shiftHeld() {
				if eid, ok := v.hitTestEdge(mx, my); ok && v.OnIncident != nil {
					v.OnIncident(uint32(eid), nextSeverity(v.severityOf(eid)))
				}
			} else if id, ok := v.hitTestIntersection(mx, my); ok {
				v.selectedID = id
				v.hasSelection = true
			} else {
				v.hasSelection = false
			}
		}
```

Replace it with:

```go
		if v.dragging && !v.movedSinceDown {
			// Shift+click selects an edge and cycles its incident; plain click
			// selects an intersection. The two selections are mutually exclusive.
			if shiftHeld() {
				if eid, ok := v.hitTestEdge(mx, my); ok {
					v.selectedEdge = eid
					v.hasEdgeSelection = true
					v.hasSelection = false
					if v.OnIncident != nil {
						v.OnIncident(uint32(eid), nextSeverity(v.severityOf(eid)))
					}
				}
			} else if id, ok := v.hitTestIntersection(mx, my); ok {
				v.selectedID = id
				v.hasSelection = true
				v.hasEdgeSelection = false
			} else {
				v.hasSelection = false
				v.hasEdgeSelection = false
			}
		}
```

(Note: selecting an edge no longer requires `OnIncident != nil`, so in `tracereplay` Shift+click selects an edge for read-only inspection while still injecting nothing.)

- [ ] **Step 3: Extend the right-click clear**

Find the existing right-click clear:

```go
	// Right click clears the selection.
	if ebiten.IsMouseButtonPressed(ebiten.MouseButtonRight) {
		v.hasSelection = false
	}
```

Replace it with:

```go
	// Right click clears any selection.
	if ebiten.IsMouseButtonPressed(ebiten.MouseButtonRight) {
		v.hasSelection = false
		v.hasEdgeSelection = false
	}
```

- [ ] **Step 4: Build and vet**

Run: `go build ./...` — clean.
Run: `go vet ./internal/render/` — clean.
Run: `go test ./internal/render/ -count=1` — PASS (no behavior change to existing tests).

- [ ] **Step 5: Commit**

```bash
git add internal/render/viewport.go
git commit -m "$(cat <<'EOF'
feat(render): track Shift-clicked edge selection (exclusive with intersection)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: Edge selection highlight

**Files:**
- Modify: `internal/render/viewport.go` (`drawEdgeSelection` method + a call in `Draw`)

- [ ] **Step 1: Add the highlight method**

In `internal/render/viewport.go`, the `drawIncidents` method begins at:

```go
func (v *Viewport) drawIncidents(screen *ebiten.Image, snap snapshot.Snapshot) {
```

Immediately BEFORE that method, add `drawEdgeSelection`:

```go
// drawEdgeSelection strokes the selected edge in the selection color, a few
// pixels wider than the incident overlay, so it reads as a highlight halo even
// when the edge also carries a severity color. Uses the From/To node fallback
// for edges with <2 geometry points, like hitTestEdge / drawIncidents.
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

(`color`, `vector`, `network`, `minRoadStrokePx`, and `toScreen` are all already used in `viewport.go`.)

- [ ] **Step 2: Call it in Draw, under the overlay**

In `Draw`, find these two consecutive lines (the incident-overlay block):

```go
	snap := v.Buf.Read()
	v.drawIncidents(screen, snap)
```

Insert the highlight call between them so the halo is drawn first and the severity overlay sits on top:

```go
	snap := v.Buf.Read()
	v.drawEdgeSelection(screen)
	v.drawIncidents(screen, snap)
```

- [ ] **Step 3: Build and vet**

Run: `go build ./...` — clean.
Run: `go vet ./internal/render/` — clean.
Run: `go test ./internal/render/ -count=1` — PASS.

- [ ] **Step 4: Commit**

```bash
git add internal/render/viewport.go
git commit -m "$(cat <<'EOF'
feat(render): highlight the selected incident edge

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Edge detail panel

**Files:**
- Modify: `internal/render/hud.go` (`DrawEdgePanel`)
- Modify: `internal/render/viewport.go` (call `DrawEdgePanel` in `Draw`)

- [ ] **Step 1: Add DrawEdgePanel**

In `internal/render/hud.go`, after the `severityName` function added in Task 1, add:

```go
// DrawEdgePanel renders the selected edge's id, current incident level, and a
// little context, mirroring DrawSelectionPanel. The level is read from the
// snapshot so it reflects live state (re-clicking the same edge updates it).
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

(`fmt`, `ebitenutil`, `network`, `snapshot`, and `hudLineHeight` are already imported/defined in `hud.go`. No new imports needed for this task.)

- [ ] **Step 2: Call it in Draw**

In `internal/render/viewport.go` `Draw`, find the existing intersection-panel block:

```go
	if v.hasSelection {
		// HUD lines start at y=8; selection panel starts just below them.
		DrawSelectionPanel(screen, v.Net, snap, v.selectedID, 8+hudLineCount*hudLineHeight+8)
	}
```

Immediately after that block, add the edge-panel block (same offset; the two are mutually exclusive so they never overlap):

```go
	if v.hasEdgeSelection {
		DrawEdgePanel(screen, v.Net, snap, v.selectedEdge, 8+hudLineCount*hudLineHeight+8)
	}
```

- [ ] **Step 3: Build and vet**

Run: `go build ./...` — clean.
Run: `go vet ./internal/render/` — clean.
Run: `go test ./internal/render/ -count=1` — PASS.

- [ ] **Step 4: Commit**

```bash
git add internal/render/hud.go internal/render/viewport.go
git commit -m "$(cat <<'EOF'
feat(render): show selected edge's incident level in a panel

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: Incident color legend

**Files:**
- Modify: `internal/render/hud.go` (imports + `DrawIncidentLegend`)
- Modify: `internal/render/viewport.go` (call `DrawIncidentLegend` in `Draw`)

- [ ] **Step 1: Add the imports DrawIncidentLegend needs**

`internal/render/hud.go` currently imports:

```go
import (
	"fmt"
	"sort"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
)
```

Add `"image/color"` to the first group and the ebiten `vector` package to the second, so it reads:

```go
import (
	"fmt"
	"image/color"
	"sort"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/hajimehoshi/ebiten/v2/vector"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
)
```

- [ ] **Step 2: Add DrawIncidentLegend**

In `internal/render/hud.go`, after `DrawEdgePanel` (added in Task 4), add:

```go
// DrawIncidentLegend draws a color key for the incident overlay, anchored to the
// bottom-left of the screen. The swatch colors match drawIncidents exactly so
// the key matches what's painted on the map.
func DrawIncidentLegend(screen *ebiten.Image, screenHeight int) {
	rows := []struct {
		clr   color.RGBA
		label string
	}{
		{color.RGBA{240, 180, 0, 220}, "slowdown"},
		{color.RGBA{240, 120, 0, 230}, "lane closed"},
		{color.RGBA{230, 40, 40, 240}, "fully closed"},
	}
	const sw = 10 // swatch size, px
	x := 8
	// Stack the rows upward from near the bottom edge, with a header above them.
	yTop := screenHeight - 8 - len(rows)*hudLineHeight
	ebitenutil.DebugPrintAt(screen, "incidents:", x, yTop-hudLineHeight)
	for i, r := range rows {
		y := yTop + i*hudLineHeight
		vector.DrawFilledRect(screen, float32(x), float32(y)+2, sw, sw, r.clr, false)
		ebitenutil.DebugPrintAt(screen, r.label, x+sw+6, y)
	}
}
```

- [ ] **Step 3: Call it in Draw**

In `internal/render/viewport.go` `Draw`, find the `DrawHUD(...)` call:

```go
	DrawHUD(screen, snap.SimTime, len(snap.Vehicles), len(snap.Incidents), viewWidthM, viewHeightM, stats)
```

Immediately after it, add:

```go
	DrawIncidentLegend(screen, v.Height)
```

- [ ] **Step 4: Build and vet**

Run: `go build ./...` — clean (the new `image/color` and `vector` imports in `hud.go` must be used — they are, by `DrawIncidentLegend`).
Run: `go vet ./internal/render/` — clean.
Run: `go test ./internal/render/ -count=1` — PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/render/hud.go internal/render/viewport.go
git commit -m "$(cat <<'EOF'
feat(render): draw incident color legend (bottom-left)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: Docs + full verification

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Note the indicator in the README incidents section**

In `README.md`, find the `### Incidents (interactive)` section. Find this sentence inside it:

```markdown
Incidents stay until you clear them (cycle back to `none`). The active count is
shown in the HUD.
```

Replace it with:

```markdown
Incidents stay until you clear them (cycle back to `none`). The active count is
shown in the HUD, with a color legend (bottom-left) for the overlay colors.
Shift+clicking an edge also selects it: the edge is highlighted and a panel shows
its current incident level.
```

- [ ] **Step 2: Verify the README edit**

Run: `grep -n "color legend (bottom-left)" README.md`
Expected: prints the updated line.

- [ ] **Step 3: Full verification**

Run: `go build ./...` — clean.
Run: `go vet ./...` — clean.
Run: `go test ./... -count=1` — all packages PASS (this is a viewer-only change; `internal/sim` determinism and all other suites are unaffected).

- [ ] **Step 4: Commit**

```bash
git add README.md
git commit -m "$(cat <<'EOF'
docs(readme): note incident legend and edge selection panel

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

- [ ] **Step 5: Rebuild the root binaries (so the running app reflects the change)**

```bash
go build ./cmd/trafficsim/ && go build ./cmd/tracereplay/
```

Expected: both binaries build with no output. (They are gitignored; do not commit them.)

---

## Self-Review Notes

- **Spec coverage:** selection state + exclusivity + replay-friendly select (Task 2); highlight halo under overlay (Task 3); edge panel with level + context read live from snapshot (Task 4); legend with overlay-matching colors, bottom-left (Task 5); `severityName` helper (Task 1, used by Tasks 4 & 5); README note (Task 6); viewer-only / no sim-trace-determinism change (verified Task 6 Step 3). All spec sections map to a task.
- **Type/name consistency:** `hasEdgeSelection`/`selectedEdge`, `drawEdgeSelection`, `DrawEdgePanel`, `DrawIncidentLegend`, `severityName` are used identically across tasks; `DrawEdgePanel`/`DrawIncidentLegend` signatures match their call sites in Task 4/5; legend colors `{240,180,0,220}`/`{240,120,0,230}`/`{230,40,40,240}` match `drawIncidents` (Task 9 of the incidents feature) exactly.
- **No placeholders:** every code step shows complete code; every verification step gives the command and expected result. Rendering functions have no unit tests by the package's existing convention (stated in the plan and spec), with `severityName` covered by `TestSeverityName`.
