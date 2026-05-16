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
	cells    map[uint64][]gridEntry // key = row*cols + col
}

type gridEntry struct {
	id EdgeID
	p  Point
}

// NewSpatialGrid constructs a uniform grid index over the given bounds.
// cellSize is in meters; smaller cells use more memory but yield tighter
// candidate sets. A typical city-scale value is 50m. cols/rows are
// clamped to at least 1.
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
		cells: make(map[uint64][]gridEntry),
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
	g.cells[k] = append(g.cells[k], gridEntry{id: id, p: p})
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
			for _, e := range g.cells[g.key(c, r)] {
				dx := e.p.X - p.X
				dy := e.p.Y - p.Y
				if dx*dx+dy*dy <= r2 {
					out = append(out, e.id)
				}
			}
		}
	}
	return out
}
