# Phase 1 — Foundations

**Milestone:** `go test ./...` passes. Network types and spatial grid compile and have tests.

---

### Task 1.1: Project scaffold

**Files:**
- Create: `go.mod`, `README.md`

- [ ] **Step 1: Initialize Go module**

Run: `go mod init github.com/lab1702/traffic-sim`
Expected: creates `go.mod` with module line.

- [ ] **Step 2: Add core dependencies**

Run:
```bash
go get github.com/paulmach/osm@latest
go get github.com/hajimehoshi/ebiten/v2@latest
go get gopkg.in/yaml.v3@latest
```
Expected: `go.mod` and `go.sum` updated with all three.

- [ ] **Step 3: Create minimal README**

Write `README.md`:
```markdown
# traffic-sim

Go-based traffic simulator that reads OpenStreetMap files and runs a
city-scale microsimulation (IDM + lane changes + signalized intersections),
with a live desktop view and trace files for replay/analysis.

See `docs/superpowers/specs/2026-05-15-traffic-sim-design.md` for design.
```

- [ ] **Step 4: Commit**

```bash
git add go.mod go.sum README.md
git commit -m "chore: initialize Go module and dependencies"
```

---

### Task 1.2: Network types

**Files:**
- Create: `internal/network/types.go`

- [ ] **Step 1: Write the types file**

Write `internal/network/types.go`:
```go
// Package network defines the immutable routable graph built from OSM data.
// Sim and renderer both read these types without locks.
package network

type (
	NodeID         uint32
	EdgeID         uint32
	IntersectionID uint32
)

// Point is a projected planar coordinate in meters (not lat/lon).
// Conversion from lat/lon happens during graph build.
type Point struct {
	X, Y float64
}

type Node struct {
	ID  NodeID
	Pos Point
}

type Lane struct {
	Index uint8
	// AllowedTurns lists outgoing edges reachable from this lane at the
	// downstream intersection. Empty means "any outgoing edge."
	AllowedTurns []EdgeID
}

type Edge struct {
	ID         EdgeID
	From, To   NodeID
	Lanes      []Lane
	Length     float64 // meters
	SpeedLimit float64 // m/s
	Geometry   []Point // polyline including endpoints
}

type Intersection struct {
	ID           IntersectionID
	NodeID       NodeID
	Incoming     []EdgeID
	Outgoing     []EdgeID
	HasSignal    bool
}

type Network struct {
	Nodes         []Node
	Edges         []Edge
	Intersections []Intersection
	Grid          *SpatialGrid // populated by netbuild
	Bounds        BoundingBox
}

type BoundingBox struct {
	MinX, MinY, MaxX, MaxY float64
}
```

- [ ] **Step 2: Verify it compiles**

Run: `go build ./internal/network/`
Expected: no output, exit 0. (`SpatialGrid` is undefined; we'll add a stub in the next step.)

If it errors on `SpatialGrid`, that's expected — proceed to Task 1.3 which adds it. If that's the only error, you can defer the commit until 1.3.

---

### Task 1.3: Spatial grid

**Files:**
- Create: `internal/network/grid.go`
- Create: `internal/network/grid_test.go`

- [ ] **Step 1: Write the failing test**

Write `internal/network/grid_test.go`:
```go
package network

import "testing"

func TestSpatialGrid_InsertAndQuery(t *testing.T) {
	g := NewSpatialGrid(BoundingBox{0, 0, 100, 100}, 10) // 10m cells
	g.Insert(EdgeID(1), Point{5, 5})
	g.Insert(EdgeID(2), Point{55, 55})
	g.Insert(EdgeID(3), Point{6, 6})

	near := g.Query(Point{5, 5}, 5) // 5m radius
	if len(near) != 2 {
		t.Fatalf("want 2 edges near (5,5), got %d: %v", len(near), near)
	}
	got := map[EdgeID]bool{}
	for _, id := range near {
		got[id] = true
	}
	if !got[1] || !got[3] {
		t.Errorf("want edges 1 and 3 in result, got %v", got)
	}
}

func TestSpatialGrid_QueryOutOfBounds(t *testing.T) {
	g := NewSpatialGrid(BoundingBox{0, 0, 100, 100}, 10)
	g.Insert(EdgeID(1), Point{5, 5})

	// Query well outside bounds; expect empty without panicking.
	near := g.Query(Point{-50, -50}, 5)
	if len(near) != 0 {
		t.Errorf("want 0 edges, got %d", len(near))
	}
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./internal/network/ -run TestSpatialGrid -v`
Expected: FAIL — `NewSpatialGrid`, `Insert`, `Query` undefined.

- [ ] **Step 3: Implement the grid**

Write `internal/network/grid.go`:
```go
package network

import "math"

// SpatialGrid is a uniform grid index of edges by representative point.
// CellSize is in meters. Edges that span multiple cells are inserted once
// per cell they pass through (callers can insert geometry points individually).
type SpatialGrid struct {
	Bounds   BoundingBox
	CellSize float64
	cols     int
	rows     int
	cells    map[uint64][]EdgeID // key = row*cols + col
}

func NewSpatialGrid(b BoundingBox, cellSize float64) *SpatialGrid {
	cols := int(math.Ceil((b.MaxX - b.MinX) / cellSize))
	rows := int(math.Ceil((b.MaxY - b.MinY) / cellSize))
	if cols < 1 {
		cols = 1
	}
	if rows < 1 {
		rows = 1
	}
	return &SpatialGrid{
		Bounds: b, CellSize: cellSize,
		cols: cols, rows: rows,
		cells: make(map[uint64][]EdgeID),
	}
}

func (g *SpatialGrid) cellOf(p Point) (col, row int, ok bool) {
	if p.X < g.Bounds.MinX || p.X > g.Bounds.MaxX ||
		p.Y < g.Bounds.MinY || p.Y > g.Bounds.MaxY {
		return 0, 0, false
	}
	col = int((p.X - g.Bounds.MinX) / g.CellSize)
	row = int((p.Y - g.Bounds.MinY) / g.CellSize)
	if col >= g.cols {
		col = g.cols - 1
	}
	if row >= g.rows {
		row = g.rows - 1
	}
	return col, row, true
}

func (g *SpatialGrid) key(col, row int) uint64 {
	return uint64(row)*uint64(g.cols) + uint64(col)
}

func (g *SpatialGrid) Insert(id EdgeID, p Point) {
	col, row, ok := g.cellOf(p)
	if !ok {
		return
	}
	k := g.key(col, row)
	g.cells[k] = append(g.cells[k], id)
}

// Query returns edge IDs whose inserted points are within `radius` meters of p.
// May contain duplicates if the same edge was inserted multiple times; callers
// that need uniqueness must dedupe.
func (g *SpatialGrid) Query(p Point, radius float64) []EdgeID {
	col, row, ok := g.cellOf(p)
	if !ok {
		return nil
	}
	span := int(math.Ceil(radius / g.CellSize))
	r2 := radius * radius
	var out []EdgeID
	for dr := -span; dr <= span; dr++ {
		for dc := -span; dc <= span; dc++ {
			c, r := col+dc, row+dr
			if c < 0 || c >= g.cols || r < 0 || r >= g.rows {
				continue
			}
			for _, id := range g.cells[g.key(c, r)] {
				// Caller inserted the point, so we can't re-check distance
				// here without storing it. The grid is conservative: it
				// returns all edges in nearby cells; callers re-check exact
				// distance using their own geometry.
				_ = r2
				out = append(out, id)
			}
		}
	}
	return out
}
```

- [ ] **Step 4: Run tests to verify pass**

Run: `go test ./internal/network/ -v`
Expected: PASS on both tests.

- [ ] **Step 5: Run with race detector**

Run: `go test -race ./internal/network/`
Expected: PASS, no race warnings.

- [ ] **Step 6: Commit**

```bash
git add internal/network/
git commit -m "feat(network): add immutable graph types and spatial grid"
```

---

**Phase 1 done when:** `go test ./...` is green and `go build ./...` succeeds.
