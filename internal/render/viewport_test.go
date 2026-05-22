package render

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
)

func TestSegDist2(t *testing.T) {
	// Segment (0,0)-(10,0). Point (5,3) is 3 away -> d2=9. Point (-5,0) is
	// off the end, nearest is (0,0) -> d2=25.
	if got := segDist2(5, 3, 0, 0, 10, 0); got != 9 {
		t.Fatalf("segDist2 perpendicular = %v, want 9", got)
	}
	if got := segDist2(-5, 0, 0, 0, 10, 0); got != 25 {
		t.Fatalf("segDist2 past-end = %v, want 25", got)
	}
}

func TestNextSeverity_Cycles(t *testing.T) {
	got := []uint8{
		nextSeverity(snapshot.SevNone),
		nextSeverity(snapshot.SevSlowdown),
		nextSeverity(snapshot.SevLaneClose),
		nextSeverity(snapshot.SevFullClose),
	}
	want := []uint8{
		snapshot.SevSlowdown, snapshot.SevLaneClose,
		snapshot.SevFullClose, snapshot.SevNone,
	}
	for i := range want {
		if got[i] != want[i] {
			t.Fatalf("nextSeverity step %d = %d, want %d", i, got[i], want[i])
		}
	}
}

func TestSeverityName(t *testing.T) {
	cases := map[uint8]string{
		snapshot.SevNone:      "none",
		snapshot.SevSlowdown:  "slowdown",
		snapshot.SevLaneClose: "lane closed",
		snapshot.SevFullClose: "fully closed",
		uint8(99):             "none", // unknown falls back to none
	}
	for sev, want := range cases {
		if got := severityName(sev); got != want {
			t.Fatalf("severityName(%d) = %q, want %q", sev, got, want)
		}
	}
}

func TestHitTestEdge(t *testing.T) {
	net := &network.Network{
		Nodes: []network.Node{
			{ID: 0, Pos: network.Point{X: 0, Y: 0}},
			{ID: 1, Pos: network.Point{X: 100, Y: 0}},
		},
		Edges: []network.Edge{
			{ID: 0, From: 0, To: 1, Length: 100, SpeedLimit: 10,
				Lanes:    []network.Lane{{Index: 0}},
				Geometry: []network.Point{{X: 0, Y: 0}, {X: 100, Y: 0}}},
		},
		Bounds: network.BoundingBox{MinX: 0, MinY: -50, MaxX: 100, MaxY: 50},
	}
	vp := NewViewport(net, snapshot.New(), 800, 600)

	sx, sy := vp.toScreen(50, 0) // screen point over the edge midpoint
	if eid, ok := vp.hitTestEdge(int(sx), int(sy)); !ok || eid != 0 {
		t.Fatalf("hitTestEdge over edge = (%d,%v), want (0,true)", eid, ok)
	}
	fx, fy := vp.toScreen(50, 40) // 40m off the edge, beyond the 30m radius
	if _, ok := vp.hitTestEdge(int(fx), int(fy)); ok {
		t.Fatal("hitTestEdge far from any edge should miss")
	}
}
