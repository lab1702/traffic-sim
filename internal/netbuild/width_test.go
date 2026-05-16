package netbuild

import (
	"math"
	"testing"

	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/paulmach/osm"
)

func TestParseWidthMeters(t *testing.T) {
	cases := []struct {
		in      string
		want    float64
		wantOK  bool
		tolM    float64
		comment string
	}{
		{"7.5", 7.5, true, 1e-9, "bare number is meters"},
		{"7.5 m", 7.5, true, 1e-9, "meters suffix with space"},
		{"7.5m", 7.5, true, 1e-9, "meters suffix no space"},
		{"3 meters", 3, true, 1e-9, "spelled-out meters"},
		{"3 metres", 3, true, 1e-9, "British spelling"},
		{"24 ft", 24 * 0.3048, true, 1e-9, "feet suffix"},
		{"24feet", 24 * 0.3048, true, 1e-9, "feet word no space"},
		{"24'", 24 * 12 * 0.0254, true, 1e-9, "feet symbol only"},
		{"24'6\"", (24*12 + 6) * 0.0254, true, 1e-9, "feet and inches"},
		{"", 0, false, 0, "empty"},
		{"  ", 0, false, 0, "whitespace only"},
		{"wide", 0, false, 0, "non-numeric"},
		{"-3", 0, false, 0, "negative rejected"},
		{"0", 0, false, 0, "zero rejected"},
	}
	for _, c := range cases {
		got, ok := parseWidthMeters(c.in)
		if ok != c.wantOK {
			t.Errorf("parseWidthMeters(%q) ok=%v, want %v (%s)", c.in, ok, c.wantOK, c.comment)
			continue
		}
		if !ok {
			continue
		}
		if math.Abs(got-c.want) > c.tolM {
			t.Errorf("parseWidthMeters(%q) = %.6f m, want %.6f (%s)", c.in, got, c.want, c.comment)
		}
	}
}

// TestBuild_UsesExplicitWidthTag: when the OSM way carries `width=*`, the
// resulting Edge.Width must come from that tag rather than the lane-count
// fallback. Both directions of a two-way share the value.
func TestBuild_UsesExplicitWidthTag(t *testing.T) {
	feat := mkPair()
	w := mkWay(100, "residential", false, 1, 2)
	w.Tags = append(w.Tags, osm.Tag{Key: "width", Value: "10"})
	feat.Ways = []*osm.Way{w}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Edges) != 2 {
		t.Fatalf("want 2 edges (fwd+rev), got %d", len(net.Edges))
	}
	for i, e := range net.Edges {
		if math.Abs(e.Width-10) > 1e-9 {
			t.Errorf("edge %d Width = %.3f, want 10 (from width=10 tag)", i, e.Width)
		}
	}
}

// TestBuild_EstimatesWidthFromLanes: with no width tag, the Edge.Width
// should fall back to total-lanes × the shared LaneWidthMeters constant.
// A two-way residential has 1 lane per direction by default, so total
// width = 2 × 3.6 = 7.2 m.
func TestBuild_EstimatesWidthFromLanes(t *testing.T) {
	feat := mkPair()
	feat.Ways = []*osm.Way{mkWay(100, "residential", false, 1, 2)}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	const want = 2 * 3.6
	for i, e := range net.Edges {
		if math.Abs(e.Width-want) > 1e-9 {
			t.Errorf("edge %d Width = %.3f, want %.3f (2 lanes × LaneWidth)", i, e.Width, want)
		}
	}
}

// TestBuild_WidthForOneway: a one-way single-lane edge's total width
// matches lanes × LaneWidth (no doubling).
func TestBuild_WidthForOneway(t *testing.T) {
	feat := mkPair()
	w := mkWay(100, "primary", true, 1, 2)
	w.Tags = append(w.Tags, osm.Tag{Key: "lanes", Value: "3"})
	feat.Ways = []*osm.Way{w}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Edges) != 1 {
		t.Fatalf("oneway should produce 1 edge, got %d", len(net.Edges))
	}
	const want = 3 * 3.6
	if math.Abs(net.Edges[0].Width-want) > 1e-9 {
		t.Errorf("oneway edge Width = %.3f, want %.3f (3 lanes × LaneWidth)",
			net.Edges[0].Width, want)
	}
}

// mkPair is the minimal node fixture used by width tests — two nodes a
// few meters apart so Build doesn't reject the segment as degenerate.
func mkPair() *osmload.Features {
	return &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0),
		2: mkNode(2, 40.0001, -74.0),
	}}
}
