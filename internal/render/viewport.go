// Package render is the Ebitengine-based viewer. It reads snapshots from
// a snapshot.Buffer and a (read-only) Network to draw the map background.
package render

import (
	"image/color"
	"math"
	"time"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	"github.com/hajimehoshi/ebiten/v2/vector"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
)

// SignalView.Mode constants — duplicated here to avoid importing sim.
// These must match the values of sim.SignalMode.
const (
	modeNormal  = 0
	modeFlashA  = 1
	modeFlashB  = 2
	modeOff     = 3
)

type Viewport struct {
	Net    *network.Network
	Buf    *snapshot.Buffer
	Width  int
	Height int

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

	// OnSetMode, if non-nil, is invoked when the user presses N/Y/R/O
	// while an intersection is selected. The callback must be non-blocking
	// and goroutine-safe; the typical wiring is to push onto a channel
	// the sim drains each tick. Mode values: 0=normal, 1=flash_a,
	// 2=flash_b, 3=off (mirroring sim.SignalMode).
	OnSetMode func(intersectionID uint32, mode uint8)
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
		camX: cx, camY: cy, zoom: z,
	}
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
			// Click without drag → try to select an intersection.
			if id, ok := v.hitTestIntersection(mx, my); ok {
				v.selectedID = id
				v.hasSelection = true
			} else {
				v.hasSelection = false
			}
		}
		v.dragging = false
	}
	v.prevMouseX, v.prevMouseY = mx, my

	// Right click clears the selection.
	if ebiten.IsMouseButtonPressed(ebiten.MouseButtonRight) {
		v.hasSelection = false
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

func (v *Viewport) Draw(screen *ebiten.Image) {
	bg := color.RGBA{20, 20, 24, 255}
	screen.Fill(bg)

	// Draw edges as polylines.
	roadColor := color.RGBA{120, 120, 130, 255}
	for i := range v.Net.Edges {
		g := v.Net.Edges[i].Geometry
		for j := 1; j < len(g); j++ {
			x1, y1 := v.toScreen(g[j-1].X, g[j-1].Y)
			x2, y2 := v.toScreen(g[j].X, g[j].Y)
			vector.StrokeLine(screen, x1, y1, x2, y2, 1.5, roadColor, true)
		}
	}

	snap := v.Buf.Read()

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

	// Draw vehicles.
	vehColor := color.RGBA{230, 230, 240, 255}
	for _, vh := range snap.Vehicles {
		x, y := v.toScreen(vh.X, vh.Y)
		vector.DrawFilledCircle(screen, x, y, 2.5, vehColor, true)
	}

	DrawHUD(screen, snap.SimTime, len(snap.Vehicles))
	if v.hasSelection {
		// HUD draws two lines starting at y=8; selection panel starts
		// just below with a small gap.
		DrawSelectionPanel(screen, v.Net, snap, v.selectedID, 8+2*hudLineHeight+8)
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
