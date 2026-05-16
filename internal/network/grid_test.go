package network

import "testing"

func TestSpatialGrid_InsertAndQuery(t *testing.T) {
	g := NewSpatialGrid(BoundingBox{0, 0, 100, 100}, 10) // 10m cells
	g.Insert(EdgeID(1), Point{5, 5})
	g.Insert(EdgeID(2), Point{55, 55})
	g.Insert(EdgeID(3), Point{6, 6})

	near := g.Query(Point{5, 5}, 5) // 5m radius
	got := map[EdgeID]bool{}
	for _, id := range near {
		got[id] = true
	}
	if !got[1] || !got[3] {
		t.Errorf("want edges 1 and 3 in result, got %v", got)
	}
	if got[2] {
		t.Errorf("edge 2 is far away (55,55) and should not be in result, got %v", got)
	}
}

func TestSpatialGrid_InsertOutOfBoundsDropped(t *testing.T) {
	g := NewSpatialGrid(BoundingBox{0, 0, 100, 100}, 10)
	g.Insert(EdgeID(99), Point{500, 500}) // far out of bounds
	g.Insert(EdgeID(1), Point{10, 10})    // inside

	// Query the entire box; the in-bounds edge should be present, the
	// out-of-bounds one should not.
	near := g.Query(Point{50, 50}, 200)
	found := map[EdgeID]bool{}
	for _, id := range near {
		found[id] = true
	}
	if !found[1] {
		t.Errorf("in-bounds edge 1 should be queryable, got %v", found)
	}
	if found[99] {
		t.Errorf("out-of-bounds edge 99 should have been silently dropped, got %v", found)
	}
}

// TestSpatialGrid_QueryRadiusIsExact verifies that Query honors the documented
// `radius` semantics rather than returning every edge in the candidate cells.
// With cellSize=10 and radius=5, a point at (5,5) lands in cell (0,0) and the
// query span covers cells (-1..1, -1..1). Edge at (15,15) sits in cell (1,1) and
// would be returned by a coarse cell-only query, but its true distance is
// sqrt(200) ≈ 14.14 m, well outside the 5 m radius.
func TestSpatialGrid_QueryRadiusIsExact(t *testing.T) {
	g := NewSpatialGrid(BoundingBox{0, 0, 100, 100}, 10)
	g.Insert(EdgeID(1), Point{5, 5})   // at query point
	g.Insert(EdgeID(2), Point{15, 15}) // adjacent cell, 14.14 m away
	g.Insert(EdgeID(3), Point{8, 5})   // 3 m away

	near := g.Query(Point{5, 5}, 5)
	got := map[EdgeID]bool{}
	for _, id := range near {
		got[id] = true
	}
	if !got[1] || !got[3] {
		t.Errorf("want edges 1 and 3 (within 5m), got %v", got)
	}
	if got[2] {
		t.Errorf("edge 2 is 14.14m away — outside 5m radius — but was returned, got %v", got)
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
