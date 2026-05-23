package sim

import (
	"math"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

func TestCornerSpeedCap_Anchors(t *testing.T) {
	cases := []struct {
		degrees float64
		want    float64
		isInf   bool
	}{
		{0, 0, true},  // straight: no cap
		{10, 0, true}, // below cutoff: no cap
		{16, 30, false}, // just past the 15° cutoff: near upper anchor
		{90, 5, false},
		{180, 2.5, false},
	}
	for _, c := range cases {
		got := cornerSpeedCap(c.degrees * math.Pi / 180)
		if c.isInf {
			if !math.IsInf(got, 1) {
				t.Errorf("cornerSpeedCap(%.0f°): want +Inf, got %.2f", c.degrees, got)
			}
			continue
		}
		if math.Abs(got-c.want) > 0.5 {
			t.Errorf("cornerSpeedCap(%.0f°): want ~%.1f m/s, got %.2f", c.degrees, c.want, got)
		}
	}
	// Monotonic decrease check: 30° > 60° > 90° > 120° > 180°.
	prev := math.Inf(1)
	for _, deg := range []float64{30, 60, 90, 120, 180} {
		v := cornerSpeedCap(deg * math.Pi / 180)
		if v > prev {
			t.Errorf("cornerSpeedCap should monotonically decrease with angle; at %.0f° got %.2f > prev %.2f", deg, v, prev)
		}
		prev = v
	}
}

// TestWorld_BrakesForSharpTurn: a vehicle approaching a 90° turn at speed
// should decelerate before reaching the corner, and end up close to the
// corner cap speed by the time it crosses.
func TestWorld_BrakesForSharpTurn(t *testing.T) {
	// Build an L-shaped path: a long approach edge ending in a 90° right
	// turn onto a short exit edge.
	//
	//   start (0,0) --- 200m --- corner (200,0)
	//                                |
	//                                | 90° right turn (heading 0 -> -π/2)
	//                                v
	//                                end (200, -50)
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		{ID: 2, Pos: network.Point{X: 200, Y: -50}},
	}
	edges := []network.Edge{
		{
			ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 15,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{{X: 0, Y: 0}, {X: 200, Y: 0}},
		},
		{
			ID: 1, From: 1, To: 2, Length: 50, SpeedLimit: 15,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{{X: 200, Y: 0}, {X: 200, Y: -50}},
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, S: 0, V: 15}, // cruising at speed limit
	}
	w.nextID = 2

	// Verify the corner cap actually applies for this 90° turn.
	cap := cornerSpeedCap(math.Pi / 2)
	if cap >= 15 {
		t.Fatalf("test prerequisite: cornerSpeedCap(90°) = %.2f, want < edge speed 15", cap)
	}

	// Run long enough for the vehicle to approach and traverse the corner.
	// 200m at 15 m/s = 13.3s nominal; corner braking adds time. Step 500x = 25s.
	for i := 0; i < 500; i++ {
		w.Step()
		if len(w.Vehicles) == 0 {
			break // despawned
		}
	}

	// Either: the vehicle despawned (made it through the corner and off
	// the end of edge 1), OR it's on edge 1 (past the corner). Either way,
	// some time during the run the vehicle must have slowed to near the
	// corner cap. We can sample the speed AS it crosses by watching for
	// a tick where it's on edge 1 at small S.
	//
	// Simpler check: run a second time, this time recording min V.
	w2 := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w2.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, S: 0, V: 15},
	}
	w2.nextID = 2
	minV := math.Inf(1)
	for i := 0; i < 500; i++ {
		w2.Step()
		if len(w2.Vehicles) == 0 {
			break
		}
		v := &w2.Vehicles[0]
		// Only measure during the approach/turn, not during the long
		// cruise before the corner. "Approach" = on edge 0 within 30m
		// of the end, OR on edge 1.
		onApproach := v.Edge == 0 && (200-v.S) < 30
		onExit := v.Edge == 1
		if onApproach || onExit {
			if v.V < minV {
				minV = v.V
			}
		}
	}
	// minV should be close to the corner cap (within tolerance).
	if minV > cap+2.0 {
		t.Errorf("vehicle did not slow enough for the 90° corner: minV=%.2f, cap=%.2f", minV, cap)
	}
	// And it shouldn't slam to a complete stop either — gentle slowdown.
	if minV < 1.0 {
		t.Errorf("vehicle braked too hard: minV=%.2f (cap=%.2f)", minV, cap)
	}
}

func TestCircumradius(t *testing.T) {
	// Collinear points -> +Inf (a straight road has no curvature constraint).
	got := circumradius(
		network.Point{X: 0, Y: 0},
		network.Point{X: 10, Y: 0},
		network.Point{X: 20, Y: 0})
	if !math.IsInf(got, 1) {
		t.Errorf("collinear: want +Inf, got %.3f", got)
	}
	// Right isosceles triangle, right angle at the apex, legs 15: for a right
	// triangle the circumradius is half the hypotenuse = 15*sqrt(2)/2 (~10.6).
	got = circumradius(
		network.Point{X: -15, Y: 0},
		network.Point{X: 0, Y: 0},
		network.Point{X: 0, Y: -15})
	want := 15 * math.Sqrt2 / 2
	if math.Abs(got-want) > 0.1 {
		t.Errorf("right-angle circumradius: want %.3f, got %.3f", want, got)
	}
}

func TestPolylineWalk(t *testing.T) {
	// Single long segment: interpolate within it.
	geom := []network.Point{{X: 0, Y: 0}, {X: 100, Y: 0}}
	if p := pointBackFromEnd(geom, 15); math.Abs(p.X-85) > 1e-9 || math.Abs(p.Y) > 1e-9 {
		t.Errorf("pointBackFromEnd 15m: got (%.3f,%.3f) want (85,0)", p.X, p.Y)
	}
	if p := pointForwardFromStart(geom, 15); math.Abs(p.X-15) > 1e-9 || math.Abs(p.Y) > 1e-9 {
		t.Errorf("pointForwardFromStart 15m: got (%.3f,%.3f) want (15,0)", p.X, p.Y)
	}
	// Shorter than dist: clamp to the far endpoint.
	short := []network.Point{{X: 0, Y: 0}, {X: 5, Y: 0}}
	if p := pointBackFromEnd(short, 15); math.Abs(p.X) > 1e-9 {
		t.Errorf("pointBackFromEnd short edge: got X=%.3f want 0 (clamp to start)", p.X)
	}
	if p := pointForwardFromStart(short, 15); math.Abs(p.X-5) > 1e-9 {
		t.Errorf("pointForwardFromStart short edge: got X=%.3f want 5 (clamp to end)", p.X)
	}
	// Multi-segment: walk across a vertex.
	multi := []network.Point{{X: 0, Y: 0}, {X: 10, Y: 0}, {X: 20, Y: 0}}
	if p := pointBackFromEnd(multi, 15); math.Abs(p.X-5) > 1e-9 {
		t.Errorf("pointBackFromEnd across vertex: got X=%.3f want 5", p.X)
	}
	if p := pointForwardFromStart(multi, 15); math.Abs(p.X-15) > 1e-9 {
		t.Errorf("pointForwardFromStart across vertex: got X=%.3f want 15", p.X)
	}
}

// TestWorld_DoesNotBrakeForStraight: same path layout but no real turn
// at the junction (edges arranged collinearly). Vehicle should cruise at
// speed limit throughout.
func TestWorld_DoesNotBrakeForStraight(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		{ID: 2, Pos: network.Point{X: 400, Y: 0}},
	}
	edges := []network.Edge{
		{
			ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 15,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{{X: 0, Y: 0}, {X: 200, Y: 0}},
		},
		{
			ID: 1, From: 1, To: 2, Length: 200, SpeedLimit: 15,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{{X: 200, Y: 0}, {X: 400, Y: 0}},
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, S: 100, V: 15},
	}
	w.nextID = 2

	for i := 0; i < 50; i++ {
		w.Step()
		if len(w.Vehicles) == 0 {
			break
		}
	}
	// Vehicle should still be at ~speed limit (no braking happened).
	if len(w.Vehicles) > 0 {
		if w.Vehicles[0].V < 14.0 {
			t.Errorf("vehicle braked on a straight path: V=%.2f (expected ~15)", w.Vehicles[0].V)
		}
	}
}
