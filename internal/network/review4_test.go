package network

import (
	"math"
	"testing"
)

// Regression tests for the fourth full code review.

// TestHeadings_SkipZeroLengthSegments: an edge whose last (or first) two
// points coincide (two OSM nodes at one position, or a node listed twice) got
// heading atan2(0, 0) = 0, due east, whichever way the road ran, so every
// turn at its junction was misclassified.
func TestHeadings_SkipZeroLengthSegments(t *testing.T) {
	a, b := Point{X: 0, Y: 0}, Point{X: 0, Y: 100} // a road running north
	c := Point{X: 0, Y: 200}
	net := &Network{Edges: []Edge{
		{ID: 0, Geometry: []Point{a, b, b}}, // arrives at b with a duplicate end point
		{ID: 1, Geometry: []Point{b, b, c}}, // leaves b with a duplicate start point
	}}
	if h := ArrivalHeading(net, 0); math.Abs(h-math.Pi/2) > 1e-9 {
		t.Errorf("ArrivalHeading = %.3f rad, want π/2 (north)", h)
	}
	if h := DepartureHeading(net, 1); math.Abs(h-math.Pi/2) > 1e-9 {
		t.Errorf("DepartureHeading = %.3f rad, want π/2 (north)", h)
	}
	if got := ClassifyTurn(net, 0, 1); got != TurnStraight {
		t.Errorf("straight-on through the duplicate points classified as %v, want straight", got)
	}
}
