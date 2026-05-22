// Package render is the Ebitengine-based viewer. It reads snapshots from
// a snapshot.Buffer and a (read-only) Network to draw the map background.
package render

import (
	"image/color"
	"math"
	"sort"
	"time"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	"github.com/hajimehoshi/ebiten/v2/vector"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
)

// Renderer reads SignalView.Mode using the shared snapshot constants
// (which are kept in sync with sim.SignalMode by a sim-package test).
const (
	modeNormal = snapshot.ModeNormal
	modeFlashA = snapshot.ModeFlashA
	modeFlashB = snapshot.ModeFlashB
	modeOff    = snapshot.ModeOff
)

// vehicleLength is the bumper-to-bumper length used to draw vehicles.
// Duplicated here to avoid importing sim; must match sim.VehicleLength.
const vehicleLength = 5.0

// laneWidth is how wide each lane is drawn in world meters. Used to
// offset vehicles laterally so the lane they occupy is visible. Kept in
// sync with network.LaneWidthMeters so netbuild's width estimate and the
// renderer's lane offsets share the same assumption.
const laneWidth = network.LaneWidthMeters

// minRoadStrokePx is the minimum band thickness in pixels. Below this,
// road geometry vanishes at low zoom; the floor keeps the network legible
// as an outline when zoomed all the way out.
const minRoadStrokePx = 1.5

// roadColorByClass picks the band color for each OSM tier. Palette is
// tuned for the dark map background ({20,20,24}): arterials keep a hint
// of warm hue so the hierarchy is still legible, but every tier is dim
// enough that vehicles (bright green/red) and turn signals (bright
// yellow) own the foreground. Link variants collapse to their parent
// class upstream, so ramps share the mainline's color.
var roadColorByClass = map[network.RoadClass]color.RGBA{
	network.ClassMotorway:     {110, 64, 26, 255}, // dark amber
	network.ClassTrunk:        {100, 50, 38, 255}, // dark red-orange
	network.ClassPrimary:      {95, 78, 38, 255},  // dark gold
	network.ClassSecondary:    {80, 74, 46, 255},  // muted amber
	network.ClassTertiary:     {64, 66, 56, 255},  // dim olive
	network.ClassResidential:  {52, 54, 60, 255},  // dark cool gray
	network.ClassUnclassified: {47, 49, 54, 255},  // dim gray
	network.ClassLivingStreet: {44, 48, 53, 255},  // dark slate
	network.ClassService:      {38, 40, 46, 255},  // barely above bg
	network.ClassUnknown:      {42, 44, 50, 255},  // fallback
}

func roadColor(c network.RoadClass) color.RGBA {
	if rc, ok := roadColorByClass[c]; ok {
		return rc
	}
	return roadColorByClass[network.ClassUnknown]
}

// Thresholds for coloring vehicles by motion state. accelDeadband
// classifies |A| below this as "steady speed" (not accel/decel).
// stillSpeedThresh classifies speed below this as "standing still".
const (
	accelDeadband    = 0.05 // m/s^2
	stillSpeedThresh = 0.1  // m/s
)

// Turn-signal rendering. Offset and radius are both in world meters so
// the dot scales with zoom the same way the vehicle line does.
const (
	turnSignalOffset  = 1.5 // m perpendicular to heading
	turnSignalRadiusM = 0.5 // m
)

var turnSignalColor = color.RGBA{255, 200, 0, 255}

// laneOffset returns the perpendicular-right distance, in world meters,
// from the road centerline at which a vehicle in lane `lane` of an edge
// with `numLanes` lanes should be drawn. Convention: lane 0 = rightmost
// (curb side); higher indices move toward the centerline.
//
// On a two-way edge each direction occupies the right half of the road
// (lanes stack outward from the centerline). On a one-way edge lanes
// span the full road and fan out symmetrically around the centerline.
func laneOffset(lane uint8, numLanes int, hasReverse bool) float64 {
	if numLanes <= 0 {
		return 0
	}
	i := float64(lane)
	if int(lane) >= numLanes {
		i = float64(numLanes - 1)
	}
	n := float64(numLanes)
	if hasReverse {
		return (n - i - 0.5) * laneWidth
	}
	return ((n-1)/2 - i) * laneWidth
}

type Viewport struct {
	Net    *network.Network
	Buf    *snapshot.Buffer
	Width  int
	Height int

	// edgeHasReverse[i] is true iff edge i belongs to a bidirectional road
	// (its reverse twin exists). Computed once at construction; used to
	// decide whether to offset vehicles to the right of the centerline.
	edgeHasReverse []bool

	// edgeDrawOrder lists edge indices in the order they should be
	// painted: low-priority classes (service, residential) first, then
	// arterials on top so they remain visible where they cross local
	// streets. Computed once at construction since Edge.Class is immutable.
	edgeDrawOrder []int

	// Pan/zoom state.
	camX, camY float64 // world meters at screen center
	zoom       float64 // pixels per meter

	prevMouseX, prevMouseY int
	dragging               bool

	// Click-vs-drag detection.
	mouseDownX, mouseDownY int
	movedSinceDown         bool

	// Selection state (signal-toggle UI).
	hasSelection bool
	selectedID   network.IntersectionID

	// hasEdgeSelection / selectedEdge track an edge the user Shift+clicked, so
	// its current incident level can be highlighted and shown in a panel. At
	// most one of hasSelection (intersection) and hasEdgeSelection (edge) is
	// true at a time.
	hasEdgeSelection bool
	selectedEdge     network.EdgeID

	// OnSetMode, if non-nil, is invoked when the user presses N/Y/R/O
	// while an intersection is selected. The callback must be non-blocking
	// and goroutine-safe; the typical wiring is to push onto a channel
	// the sim drains each tick. Mode values: 0=normal, 1=flash_a,
	// 2=flash_b, 3=off (mirroring sim.SignalMode).
	OnSetMode func(intersectionID uint32, mode uint8)

	// OnIncident, if non-nil, is invoked when the user Shift+clicks an edge.
	// severity uses the snapshot.Sev* values. Same non-blocking, goroutine-
	// safe contract as OnSetMode (typically pushes onto a channel).
	OnIncident func(edgeID uint32, severity uint8)
}

func NewViewport(net *network.Network, buf *snapshot.Buffer, w, h int) *Viewport {
	b := net.Bounds
	cx := (b.MinX + b.MaxX) / 2
	cy := (b.MinY + b.MaxY) / 2
	// Zoom so the network fits the window with some margin.
	zx := float64(w) / (b.MaxX - b.MinX) * 0.9
	zy := float64(h) / (b.MaxY - b.MinY) * 0.9
	z := math.Min(zx, zy)
	if z <= 0 || math.IsNaN(z) || math.IsInf(z, 0) {
		z = 1
	}
	return &Viewport{
		Net: net, Buf: buf, Width: w, Height: h,
		edgeHasReverse: computeReverseEdgeMap(net),
		edgeDrawOrder:  computeEdgeDrawOrder(net),
		camX:           cx, camY: cy, zoom: z,
	}
}

// computeEdgeDrawOrder returns edge indices sorted by ascending class
// priority (low priority first) so arterials paint over local streets.
// sort.SliceStable preserves source order within each class.
func computeEdgeDrawOrder(net *network.Network) []int {
	order := make([]int, len(net.Edges))
	for i := range order {
		order[i] = i
	}
	sort.SliceStable(order, func(a, b int) bool {
		return net.Edges[order[a]].Class.Priority() < net.Edges[order[b]].Class.Priority()
	})
	return order
}

// computeReverseEdgeMap returns a parallel slice where entry i is true
// iff edges[i] has a twin going the opposite direction between the same
// node pair (i.e. it's part of a two-way road).
func computeReverseEdgeMap(net *network.Network) []bool {
	type pair struct{ a, b network.NodeID }
	exists := make(map[pair]struct{}, len(net.Edges))
	for i := range net.Edges {
		e := &net.Edges[i]
		exists[pair{e.From, e.To}] = struct{}{}
	}
	out := make([]bool, len(net.Edges))
	for i := range net.Edges {
		e := &net.Edges[i]
		if _, ok := exists[pair{e.To, e.From}]; ok {
			out[i] = true
		}
	}
	return out
}

// World->screen transformation. Y is flipped (world y up, screen y down).
func (v *Viewport) toScreen(x, y float64) (float32, float32) {
	sx := float64(v.Width)/2 + (x-v.camX)*v.zoom
	sy := float64(v.Height)/2 - (y-v.camY)*v.zoom
	return float32(sx), float32(sy)
}

func (v *Viewport) Update() error {
	mx, my := ebiten.CursorPosition()

	// Left mouse: pan if dragged, select on click-without-drag.
	if ebiten.IsMouseButtonPressed(ebiten.MouseButtonLeft) {
		if !v.dragging {
			v.dragging = true
			v.mouseDownX, v.mouseDownY = mx, my
			v.movedSinceDown = false
		}
		// Once movement crosses a small threshold, we're panning.
		if absInt(mx-v.mouseDownX) > 3 || absInt(my-v.mouseDownY) > 3 {
			v.movedSinceDown = true
		}
		if v.movedSinceDown {
			dx := float64(mx-v.prevMouseX) / v.zoom
			dy := float64(my-v.prevMouseY) / v.zoom
			v.camX -= dx
			v.camY += dy // screen-y inverted
		}
	} else {
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
		v.dragging = false
	}
	v.prevMouseX, v.prevMouseY = mx, my

	// Right click clears any selection.
	if ebiten.IsMouseButtonPressed(ebiten.MouseButtonRight) {
		v.hasSelection = false
		v.hasEdgeSelection = false
	}

	// Hotkeys while something is selected.
	if v.hasSelection && v.OnSetMode != nil {
		if inpututil.IsKeyJustPressed(ebiten.KeyN) {
			v.OnSetMode(uint32(v.selectedID), modeNormal)
		}
		if inpututil.IsKeyJustPressed(ebiten.KeyY) {
			v.OnSetMode(uint32(v.selectedID), modeFlashA)
		}
		if inpututil.IsKeyJustPressed(ebiten.KeyR) {
			v.OnSetMode(uint32(v.selectedID), modeFlashB)
		}
		if inpututil.IsKeyJustPressed(ebiten.KeyO) {
			v.OnSetMode(uint32(v.selectedID), modeOff)
		}
	}

	// Zoom: mouse wheel.
	_, wheelY := ebiten.Wheel()
	if wheelY != 0 {
		factor := math.Pow(1.1, wheelY)
		v.zoom *= factor
		if v.zoom < 0.001 {
			v.zoom = 0.001
		}
	}

	// Clamp camera to bounds.
	b := v.Net.Bounds
	if v.camX < b.MinX {
		v.camX = b.MinX
	}
	if v.camX > b.MaxX {
		v.camX = b.MaxX
	}
	if v.camY < b.MinY {
		v.camY = b.MinY
	}
	if v.camY > b.MaxY {
		v.camY = b.MaxY
	}
	return nil
}

func absInt(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

// hitTestIntersection converts a screen-space mouse position to a world
// coordinate and returns the nearest signalized intersection within a
// 30 m radius. Returns false if nothing is close enough.
func (v *Viewport) hitTestIntersection(mx, my int) (network.IntersectionID, bool) {
	wx := v.camX + (float64(mx)-float64(v.Width)/2)/v.zoom
	wy := v.camY - (float64(my)-float64(v.Height)/2)/v.zoom
	const radius = 30.0
	bestD2 := radius * radius
	var bestID network.IntersectionID
	found := false
	for i := range v.Net.Intersections {
		x := &v.Net.Intersections[i]
		if !x.HasSignal {
			continue
		}
		p := v.Net.Nodes[x.NodeID].Pos
		dx := p.X - wx
		dy := p.Y - wy
		d2 := dx*dx + dy*dy
		if d2 < bestD2 {
			bestD2 = d2
			bestID = x.ID
			found = true
		}
	}
	return bestID, found
}

// segDist2 returns the squared distance from point (px,py) to the segment
// (ax,ay)-(bx,by).
func segDist2(px, py, ax, ay, bx, by float64) float64 {
	dx, dy := bx-ax, by-ay
	if dx == 0 && dy == 0 {
		ex, ey := px-ax, py-ay
		return ex*ex + ey*ey
	}
	t := ((px-ax)*dx + (py-ay)*dy) / (dx*dx + dy*dy)
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}
	cx, cy := ax+t*dx, ay+t*dy
	ex, ey := px-cx, py-cy
	return ex*ex + ey*ey
}

// hitTestEdge returns the nearest edge to the screen-space cursor within a
// 30 m radius (point-to-polyline distance). Mirrors hitTestIntersection.
func (v *Viewport) hitTestEdge(mx, my int) (network.EdgeID, bool) {
	wx := v.camX + (float64(mx)-float64(v.Width)/2)/v.zoom
	wy := v.camY - (float64(my)-float64(v.Height)/2)/v.zoom
	const radius = 30.0
	bestD2 := radius * radius
	var bestID network.EdgeID
	found := false
	for i := range v.Net.Edges {
		e := &v.Net.Edges[i]
		pts := e.Geometry
		if len(pts) < 2 {
			pts = []network.Point{v.Net.Nodes[e.From].Pos, v.Net.Nodes[e.To].Pos}
		}
		for j := 0; j+1 < len(pts); j++ {
			d2 := segDist2(wx, wy, pts[j].X, pts[j].Y, pts[j+1].X, pts[j+1].Y)
			if d2 < bestD2 {
				bestD2 = d2
				bestID = network.EdgeID(i)
				found = true
			}
		}
	}
	return bestID, found
}

// shiftHeld reports whether either Shift key is down.
func shiftHeld() bool {
	return ebiten.IsKeyPressed(ebiten.KeyShiftLeft) || ebiten.IsKeyPressed(ebiten.KeyShiftRight)
}

// severityOf returns the current incident severity on an edge from the latest
// snapshot (snapshot.Sev* value), or SevNone.
func (v *Viewport) severityOf(eid network.EdgeID) uint8 {
	snap := v.Buf.Read()
	for _, inc := range snap.Incidents {
		if inc.EdgeID == uint32(eid) {
			return inc.Severity
		}
	}
	return snapshot.SevNone
}

// nextSeverity advances the click cycle none -> Slowdown -> LaneClose ->
// FullClose -> none.
func nextSeverity(cur uint8) uint8 {
	switch cur {
	case snapshot.SevNone:
		return snapshot.SevSlowdown
	case snapshot.SevSlowdown:
		return snapshot.SevLaneClose
	case snapshot.SevLaneClose:
		return snapshot.SevFullClose
	default:
		return snapshot.SevNone
	}
}

func (v *Viewport) Draw(screen *ebiten.Image) {
	bg := color.RGBA{20, 20, 24, 255}
	screen.Fill(bg)

	// Draw edges as shaded bands sized by Edge.Width (OSM `width=*` tag
	// when present, lane-count estimate otherwise). For two-way roads
	// both directions carry the same Width and overlay each other on the
	// shared centerline, painting a single band of the road's full width.
	v.drawRoadBands(screen)

	snap := v.Buf.Read()
	v.drawIncidents(screen, snap)

	// Blink phase for flash modes: on for 500ms, off for 500ms (1 Hz).
	blinkOn := (time.Now().UnixMilli()/500)%2 == 0

	// Draw signals as small colored circles. Color and visibility depend
	// on mode. Selection ring (if any) is drawn underneath.
	const (
		dotRadius     = 4.0
		selectionRing = 9.0
	)
	colorDark := color.RGBA{60, 60, 70, 255}
	colorGreen := color.RGBA{0, 200, 0, 255}
	colorYellow := color.RGBA{220, 200, 0, 255}
	colorRed := color.RGBA{220, 60, 60, 255}
	colorSelect := color.RGBA{180, 180, 255, 255}

	for _, s := range snap.Signals {
		x, y := v.toScreen(s.X, s.Y)

		if v.hasSelection && network.IntersectionID(s.IntersectionID) == v.selectedID {
			vector.StrokeCircle(screen, x, y, selectionRing, 1.5, colorSelect, true)
		}

		switch s.Mode {
		case modeOff:
			// Power-out: dark dot so the signal location is still visible
			// but clearly inactive.
			vector.DrawFilledCircle(screen, x, y, dotRadius, colorDark, true)
		case modeFlashA, modeFlashB:
			// Blinking: alternate colored dot with dark off-frame.
			if !blinkOn {
				vector.DrawFilledCircle(screen, x, y, dotRadius, colorDark, true)
				continue
			}
			c := colorRed
			if !s.IsRed {
				c = colorYellow // priority axis blinks yellow
			}
			vector.DrawFilledCircle(screen, x, y, dotRadius, c, true)
		default:
			// Normal mode.
			c := colorGreen
			if s.IsYellow {
				c = colorYellow
			}
			if s.IsRed {
				c = colorRed
			}
			vector.DrawFilledCircle(screen, x, y, dotRadius, c, true)
		}
	}

	// Draw a small red "no" marker at intersections with banned turns.
	// Circle outline plus a diagonal slash, drawn at the intersection
	// center (the NodeID position), not at the per-approach offset.
	const noRingRadius = 6.0
	colorNoMark := color.RGBA{220, 60, 60, 255}
	for i := range v.Net.Intersections {
		x := &v.Net.Intersections[i]
		if len(x.BannedTurns) == 0 {
			continue
		}
		p := v.Net.Nodes[x.NodeID].Pos
		cx, cy := v.toScreen(p.X, p.Y)
		vector.StrokeCircle(screen, cx, cy, noRingRadius, 1.5, colorNoMark, true)
		// Diagonal slash NW->SE through the ring.
		d := float32(noRingRadius) * 0.707 // sin(45°)
		vector.StrokeLine(screen, cx-d, cy-d, cx+d, cy+d, 1.5, colorNoMark, true)
	}

	// Draw vehicles as 5 m line segments in world units, oriented along
	// the lane tangent. (vh.X, vh.Y) is the front bumper; the back bumper
	// is 5 m back along the heading. Vehicles are shifted perpendicular
	// to the heading by laneOffset() so the occupied lane is visible.
	// Pure world units: at low zoom the line shrinks below a pixel and
	// effectively disappears.
	//
	// Color reflects motion state: green when accelerating or moving at
	// steady speed, red when standing still or decelerating.
	vehColorGreen := color.RGBA{0, 230, 0, 255}
	vehColorRed := color.RGBA{230, 50, 50, 255}
	for _, vh := range snap.Vehicles {
		cosH := math.Cos(vh.Heading)
		sinH := math.Sin(vh.Heading)
		numLanes := 0
		hasReverse := false
		if int(vh.EdgeID) < len(v.Net.Edges) {
			numLanes = len(v.Net.Edges[vh.EdgeID].Lanes)
		}
		if int(vh.EdgeID) < len(v.edgeHasReverse) {
			hasReverse = v.edgeHasReverse[vh.EdgeID]
		}
		d := laneOffset(vh.Lane, numLanes, hasReverse)
		// Perpendicular-right of heading H in math coords (y up) is
		// (sin H, -cos H). Scale by the signed lane offset.
		offX := sinH * d
		offY := -cosH * d
		frontX := vh.X + offX
		frontY := vh.Y + offY
		backX := frontX - vehicleLength*cosH
		backY := frontY - vehicleLength*sinH
		fx, fy := v.toScreen(frontX, frontY)
		rx, ry := v.toScreen(backX, backY)
		// Color: red if standing still or decelerating, else green.
		// Thresholds reject IDM numerical noise around zero.
		c := vehColorGreen
		if vh.Accel < -accelDeadband || (math.Abs(vh.Accel) <= accelDeadband && vh.Speed < stillSpeedThresh) {
			c = vehColorRed
		}
		vector.StrokeLine(screen, fx, fy, rx, ry, 1.5, c, true)

		// Turn signal: yellow dot at the front bumper, offset perpendicular
		// to heading on the indicated side. Blinks at 1 Hz using the same
		// blink phase as traffic-signal flashers. Radius is in world units
		// so the dot scales with zoom alongside the vehicle line.
		if vh.TurnSignal != 0 && blinkOn {
			// Perpendicular-left of heading H is (-sin H, cos H); right is
			// (sin H, -cos H). TurnSignal: +1 = left, -1 = right.
			sigOffX := -sinH * float64(vh.TurnSignal) * turnSignalOffset
			sigOffY := cosH * float64(vh.TurnSignal) * turnSignalOffset
			sx, sy := v.toScreen(frontX+sigOffX, frontY+sigOffY)
			pxRadius := float32(turnSignalRadiusM * v.zoom)
			vector.DrawFilledCircle(screen, sx, sy, pxRadius, turnSignalColor, true)
		}
	}

	viewWidthM := float64(v.Width) / v.zoom
	viewHeightM := float64(v.Height) / v.zoom
	stats := computeSpeedStats(snap.Vehicles)
	DrawHUD(screen, snap.SimTime, len(snap.Vehicles), len(snap.Incidents), viewWidthM, viewHeightM, stats)
	if v.hasSelection {
		// HUD lines start at y=8; selection panel starts just below them.
		DrawSelectionPanel(screen, v.Net, snap, v.selectedID, 8+hudLineCount*hudLineHeight+8)
	}
}

// drawRoadBands strokes each edge's polyline at a thickness derived from
// Edge.Width (meters → pixels via the current zoom). Uses vector.Path with
// round joins/caps so corners and dead-ends look clean. Single-segment
// edges fall through to vector.StrokeLine since vector.Path is overkill
// there.
func (v *Viewport) drawRoadBands(screen *ebiten.Image) {
	strokeOpts := &vector.StrokeOptions{
		LineCap:  vector.LineCapRound,
		LineJoin: vector.LineJoinRound,
	}
	drawOpts := &vector.DrawPathOptions{AntiAlias: true}
	// ColorScale must be re-applied per edge since each class has its own
	// color. Reset() between edges keeps successive ScaleWithColor calls
	// from compounding into a multiplied tint.
	for _, ei := range v.edgeDrawOrder {
		e := &v.Net.Edges[ei]
		g := e.Geometry
		if len(g) < 2 {
			continue
		}
		w := float32(e.Width * v.zoom)
		if w < minRoadStrokePx {
			w = minRoadStrokePx
		}
		clr := roadColor(e.Class)
		if len(g) == 2 {
			x1, y1 := v.toScreen(g[0].X, g[0].Y)
			x2, y2 := v.toScreen(g[1].X, g[1].Y)
			vector.StrokeLine(screen, x1, y1, x2, y2, w, clr, true)
			continue
		}
		path := &vector.Path{}
		x, y := v.toScreen(g[0].X, g[0].Y)
		path.MoveTo(x, y)
		for j := 1; j < len(g); j++ {
			x, y = v.toScreen(g[j].X, g[j].Y)
			path.LineTo(x, y)
		}
		strokeOpts.Width = w
		drawOpts.ColorScale.Reset()
		drawOpts.ColorScale.ScaleWithColor(clr)
		vector.StrokePath(screen, path, strokeOpts, drawOpts)
	}
}

// drawIncidents overlays each active-incident edge in a severity color,
// slightly thicker than the road band so it reads as a highlight.
func (v *Viewport) drawIncidents(screen *ebiten.Image, snap snapshot.Snapshot) {
	for _, inc := range snap.Incidents {
		if int(inc.EdgeID) >= len(v.Net.Edges) {
			continue
		}
		if inc.Severity == snapshot.SevNone {
			continue
		}
		e := &v.Net.Edges[inc.EdgeID]
		pts := e.Geometry
		if len(pts) < 2 {
			pts = []network.Point{v.Net.Nodes[e.From].Pos, v.Net.Nodes[e.To].Pos}
		}
		var clr color.RGBA
		switch inc.Severity {
		case snapshot.SevSlowdown:
			clr = color.RGBA{240, 180, 0, 220} // amber
		case snapshot.SevLaneClose:
			clr = color.RGBA{240, 120, 0, 230} // orange
		default:
			clr = color.RGBA{230, 40, 40, 240} // red (full close)
		}
		w := float32(e.Width*v.zoom) + 2
		if w < minRoadStrokePx+2 {
			w = minRoadStrokePx + 2
		}
		for j := 0; j+1 < len(pts); j++ {
			x1, y1 := v.toScreen(pts[j].X, pts[j].Y)
			x2, y2 := v.toScreen(pts[j+1].X, pts[j+1].Y)
			vector.StrokeLine(screen, x1, y1, x2, y2, w, clr, true)
		}
	}
}

// Layout is called by Ebitengine on every frame with the current outside
// window size. We return the same size so the logical and physical
// resolutions match (1:1 pixel scaling) and update Width/Height so the
// world-to-screen math reflects the resized window. The camera position
// and zoom stay put — a resize reveals more or less surrounding area, it
// doesn't re-fit the network to the new size.
func (v *Viewport) Layout(outW, outH int) (int, int) {
	v.Width = outW
	v.Height = outH
	return outW, outH
}
