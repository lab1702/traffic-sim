// Package render is the Ebitengine-based viewer. It reads snapshots from
// a snapshot.Buffer and a (read-only) Network to draw the map background.
package render

import (
	"image/color"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
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
	// Pan: drag with left mouse.
	mx, my := ebiten.CursorPosition()
	if ebiten.IsMouseButtonPressed(ebiten.MouseButtonLeft) {
		if v.dragging {
			dx := float64(mx-v.prevMouseX) / v.zoom
			dy := float64(my-v.prevMouseY) / v.zoom
			v.camX -= dx
			v.camY += dy // screen-y inverted
		}
		v.dragging = true
	} else {
		v.dragging = false
	}
	v.prevMouseX, v.prevMouseY = mx, my

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

	// Draw signals as small colored circles.
	for _, s := range snap.Signals {
		x, y := v.toScreen(s.X, s.Y)
		c := color.RGBA{0, 200, 0, 255} // green
		if s.IsYellow {
			c = color.RGBA{220, 200, 0, 255}
		}
		if s.IsRed {
			c = color.RGBA{220, 60, 60, 255}
		}
		vector.DrawFilledCircle(screen, x, y, 4, c, true)
	}

	// Draw vehicles.
	vehColor := color.RGBA{230, 230, 240, 255}
	for _, vh := range snap.Vehicles {
		x, y := v.toScreen(vh.X, vh.Y)
		vector.DrawFilledCircle(screen, x, y, 2.5, vehColor, true)
	}

	DrawHUD(screen, snap.SimTime, len(snap.Vehicles))
}

func (v *Viewport) Layout(_, _ int) (int, int) {
	return v.Width, v.Height
}
