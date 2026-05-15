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
