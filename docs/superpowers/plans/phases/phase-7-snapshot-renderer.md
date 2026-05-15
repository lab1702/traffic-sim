# Phase 7 — Snapshot + Renderer

**Milestone:** `trafficsim run <file>` (no `--headless`) opens an Ebitengine window showing the network with moving vehicles, traffic signals colored by state, a HUD (sim time, vehicle count, FPS), and pan/zoom controls.

---

### Task 7.1: Snapshot type + double buffer

**Files:**
- Create: `internal/snapshot/snapshot.go`
- Create: `internal/snapshot/snapshot_test.go`

- [ ] **Step 1: Write the failing test**

Write `internal/snapshot/snapshot_test.go`:
```go
package snapshot

import (
	"sync/atomic"
	"testing"
	"time"
)

func TestDoubleBuffer_SwapIsAtomic(t *testing.T) {
	b := New()
	var stop atomic.Bool

	// Writer goroutine: fills back-buffer with monotonically increasing Tick.
	go func() {
		var k uint64
		for !stop.Load() {
			s := Snapshot{Tick: k}
			b.Publish(s)
			k++
		}
	}()

	// Reader: reads many times, asserts monotonicity.
	var lastTick uint64
	deadline := time.Now().Add(200 * time.Millisecond)
	for time.Now().Before(deadline) {
		got := b.Read()
		if got.Tick < lastTick {
			t.Fatalf("non-monotonic tick: %d < %d", got.Tick, lastTick)
		}
		lastTick = got.Tick
	}
	stop.Store(true)
}
```

- [ ] **Step 2: Run to verify failure**

Run: `go test ./internal/snapshot/ -v -race`
Expected: FAIL — package doesn't exist.

- [ ] **Step 3: Implement double buffer**

Write `internal/snapshot/snapshot.go`:
```go
// Package snapshot provides a single-producer/single-consumer pointer
// swap of read-only Snapshot values. Writer always writes a new value
// then atomically updates the pointer; readers always see a complete value.
package snapshot

import (
	"sync/atomic"

	"github.com/lab1702/traffic-sim/internal/network"
)

type Snapshot struct {
	Tick     uint64
	SimTime  float64
	Vehicles []VehicleView
	Signals  []SignalView
	Bounds   network.BoundingBox
}

type VehicleView struct {
	ID      uint32
	X, Y    float64
	Heading float64 // radians (atan2(dy, dx))
	Speed   float64
}

type SignalView struct {
	IntersectionID uint32
	X, Y           float64
	IsRed          bool
	IsYellow       bool
}

type Buffer struct {
	front atomic.Pointer[Snapshot]
}

func New() *Buffer {
	b := &Buffer{}
	b.front.Store(&Snapshot{})
	return b
}

// Publish swaps in a new snapshot. Caller must not mutate s after.
func (b *Buffer) Publish(s Snapshot) {
	b.front.Store(&s)
}

// Read returns the latest published snapshot. May be the empty snapshot
// if nothing has been published yet.
func (b *Buffer) Read() Snapshot {
	return *b.front.Load()
}
```

- [ ] **Step 4: Run tests**

Run: `go test ./internal/snapshot/ -v -race`
Expected: PASS, no race.

- [ ] **Step 5: Commit**

```bash
git add internal/snapshot/
git commit -m "feat(snapshot): atomic-pointer double buffer for sim->render handoff"
```

---

### Task 7.2: World publishes snapshots

**Files:**
- Modify: `internal/sim/world.go`

- [ ] **Step 1: Add snapshot publishing to World**

Read `internal/sim/world.go`, then:

1. Add field:
```go
SnapshotBuf *snapshot.Buffer
```
And add import for `github.com/lab1702/traffic-sim/internal/snapshot`.

2. In `NewWorld`, initialize: `SnapshotBuf: snapshot.New(),`.

3. After the stepping loop in `Step()`, before `w.compact()`, build and publish a snapshot:
```go
w.publishSnapshot()
```

4. Add the method:
```go
func (w *World) publishSnapshot() {
	if w.SnapshotBuf == nil {
		return
	}
	views := make([]snapshot.VehicleView, 0, len(w.Vehicles))
	for i := range w.Vehicles {
		v := &w.Vehicles[i]
		if v.Despawned {
			continue
		}
		x, y, hd := positionOnEdge(w.Net, v.Edge, v.S)
		views = append(views, snapshot.VehicleView{
			ID: uint32(v.ID), X: x, Y: y, Heading: hd, Speed: v.V,
		})
	}
	sigs := make([]snapshot.SignalView, 0, len(w.SignalStates))
	for i, st := range w.SignalStates {
		if st == nil {
			continue
		}
		x := &w.Net.Intersections[i]
		node := w.Net.Nodes[x.NodeID]
		// "Is red" for visualization: red if no incoming edge is currently green.
		isRed := true
		for j := range x.Incoming {
			if st.GreenFor(j) {
				isRed = false
				break
			}
		}
		sigs = append(sigs, snapshot.SignalView{
			IntersectionID: uint32(x.ID),
			X: node.Pos.X, Y: node.Pos.Y,
			IsRed: isRed, IsYellow: st.IsYellow,
		})
	}
	w.SnapshotBuf.Publish(snapshot.Snapshot{
		Tick: w.Tick, SimTime: w.SimTime,
		Vehicles: views, Signals: sigs, Bounds: w.Net.Bounds,
	})
}
```

5. Add helper:
```go
import "math" // if not already imported

// positionOnEdge returns (x, y, heading) for the point S meters along
// edge's polyline geometry. Linear interpolation between vertices.
func positionOnEdge(net *network.Network, eid network.EdgeID, s float64) (float64, float64, float64) {
	e := &net.Edges[eid]
	g := e.Geometry
	if len(g) < 2 {
		return 0, 0, 0
	}
	remaining := s
	for i := 1; i < len(g); i++ {
		dx := g[i].X - g[i-1].X
		dy := g[i].Y - g[i-1].Y
		segLen := math.Sqrt(dx*dx + dy*dy)
		if remaining <= segLen || i == len(g)-1 {
			t := 0.0
			if segLen > 0 {
				t = remaining / segLen
			}
			if t > 1 {
				t = 1
			}
			x := g[i-1].X + dx*t
			y := g[i-1].Y + dy*t
			heading := math.Atan2(dy, dx)
			return x, y, heading
		}
		remaining -= segLen
	}
	return g[len(g)-1].X, g[len(g)-1].Y, 0
}
```

- [ ] **Step 2: Verify build and tests**

Run: `go test ./...`
Expected: green.

- [ ] **Step 3: Commit**

```bash
git add internal/sim/world.go
git commit -m "feat(sim): publish render snapshots each tick via double buffer"
```

---

### Task 7.3: Ebitengine viewport — render edges + bounds

**Files:**
- Create: `internal/render/viewport.go`

- [ ] **Step 1: Write the viewport module**

Write `internal/render/viewport.go`:
```go
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
}

func (v *Viewport) Layout(_, _ int) (int, int) {
	return v.Width, v.Height
}
```

- [ ] **Step 2: Verify build**

Run: `go build ./internal/render/`
Expected: exit 0.

- [ ] **Step 3: Commit**

```bash
git add internal/render/viewport.go
git commit -m "feat(render): Ebitengine viewport with pan/zoom and snapshot reading"
```

---

### Task 7.4: HUD overlay

**Files:**
- Create: `internal/render/hud.go`
- Modify: `internal/render/viewport.go` (call HUD in Draw)

- [ ] **Step 1: Write HUD module**

Write `internal/render/hud.go`:
```go
package render

import (
	"fmt"
	"image/color"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
)

// DrawHUD renders text overlay (sim time, vehicle count, FPS).
func DrawHUD(screen *ebiten.Image, simTime float64, vehicleCount int) {
	line1 := fmt.Sprintf("sim t=%.1fs  vehicles=%d", simTime, vehicleCount)
	line2 := fmt.Sprintf("FPS=%.1f  TPS=%.1f", ebiten.ActualFPS(), ebiten.ActualTPS())
	ebitenutil.DebugPrintAt(screen, line1, 8, 8)
	ebitenutil.DebugPrintAt(screen, line2, 8, 24)
	_ = color.White
}
```

- [ ] **Step 2: Call HUD from viewport Draw**

In `internal/render/viewport.go`, at the bottom of `Draw`, before returning, add:
```go
DrawHUD(screen, snap.SimTime, len(snap.Vehicles))
```

- [ ] **Step 3: Build**

Run: `go build ./...`
Expected: exit 0.

- [ ] **Step 4: Commit**

```bash
git add internal/render/
git commit -m "feat(render): HUD overlay with sim time, vehicle count, FPS"
```

---

### Task 7.5: Wire renderer into `trafficsim run` (non-headless path)

**Files:**
- Modify: `cmd/trafficsim/main.go`

- [ ] **Step 1: Read main.go**

Read `cmd/trafficsim/main.go`.

- [ ] **Step 2: Replace the non-headless branch**

In `runRun`, replace the `if !*headless { fmt.Fprintln... os.Exit(2) }` block with renderer wiring:

```go
if *headless {
	if *duration == 0 {
		fmt.Fprintln(os.Stderr, "error: --headless requires --duration > 0")
		os.Exit(2)
	}
	w.Run(duration.Seconds())
	fmt.Printf("done. final_vehicles=%d ticks=%d sim_time=%.2fs\n",
		len(w.Vehicles), w.Tick, w.SimTime)
	return
}

// Live mode: sim runs on its own goroutine at 20 Hz wall-clock,
// renderer runs at Ebitengine's default frame rate (~60 FPS).
stop := make(chan struct{})
go func() {
	ticker := time.NewTicker(time.Duration(sim.DefaultDt * float64(time.Second)))
	defer ticker.Stop()
	for {
		select {
		case <-stop:
			return
		case <-ticker.C:
			w.Step()
		}
	}
}()

vp := render.NewViewport(net, w.SnapshotBuf, 1280, 800)
ebiten.SetWindowSize(1280, 800)
ebiten.SetWindowTitle("traffic-sim")
if err := ebiten.RunGame(&gameAdapter{vp: vp}); err != nil {
	close(stop)
	slog.Error("ebiten exited", "err", err)
	os.Exit(1)
}
close(stop)
```

- [ ] **Step 3: Add gameAdapter type and imports**

At the bottom of `cmd/trafficsim/main.go`:
```go
// gameAdapter wraps a Viewport into Ebitengine's Game interface.
type gameAdapter struct {
	vp *render.Viewport
}

func (g *gameAdapter) Update() error                       { return g.vp.Update() }
func (g *gameAdapter) Draw(screen *ebiten.Image)           { g.vp.Draw(screen) }
func (g *gameAdapter) Layout(w, h int) (int, int)          { return g.vp.Layout(w, h) }
```

Update imports in `cmd/trafficsim/main.go`:
```go
import (
	"flag"
	"fmt"
	"log/slog"
	"os"
	"time"

	"github.com/hajimehoshi/ebiten/v2"

	"github.com/lab1702/traffic-sim/internal/config"
	"github.com/lab1702/traffic-sim/internal/netbuild"
	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/lab1702/traffic-sim/internal/render"
	"github.com/lab1702/traffic-sim/internal/sim"
)
```

- [ ] **Step 4: Build**

Run: `go build ./cmd/trafficsim/`
Expected: exit 0.

- [ ] **Step 5: Manual smoke test**

Download a small OSM extract (e.g., from BBBike https://extract.bbbike.org for a small city — 5-20 MB max). Save as `testdata-osm/your-city.osm.pbf`.

Run:
```powershell
.\trafficsim.exe run testdata-osm\your-city.osm.pbf --spawn-rate 20
```
Expected: a window opens showing the road network in light gray, with small white dots moving along roads. Some intersections show green/yellow/red circles. The HUD shows sim time, vehicle count, FPS/TPS. Mouse drag pans; wheel zooms.

If anything goes wrong (window doesn't open, vehicles don't move, signals don't appear), note the symptom — debugging is part of this task.

- [ ] **Step 6: Commit**

```bash
git add cmd/trafficsim/main.go
git commit -m "feat(cli): wire renderer for non-headless trafficsim run"
```

---

**Phase 7 done when:**
- `go build ./...` succeeds.
- `trafficsim run <file>` opens a window with moving vehicles and signals.
- Pan/zoom work.
- `--headless` still works as before.
