package sim

import (
	"math"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

// TestWorld_BrakesForSharpTurn: a car at the speed limit approaching a 90° turn
// should slow to about the radius-based corner speed by the time it crosses.
func TestWorld_BrakesForSharpTurn(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		{ID: 2, Pos: network.Point{X: 200, Y: -50}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 15,
			Lanes: []network.Lane{{Index: 0}}, Geometry: []network.Point{{X: 0, Y: 0}, {X: 200, Y: 0}}},
		{ID: 1, From: 1, To: 2, Length: 50, SpeedLimit: 15,
			Lanes: []network.Lane{{Index: 0}}, Geometry: []network.Point{{X: 200, Y: 0}, {X: 200, Y: -50}}},
	}
	net := &network.Network{Nodes: nodes, Edges: edges}

	expected := cornerSpeed(turnRadius(net, 0, 1))
	if expected >= 15 {
		t.Fatalf("test prerequisite: corner speed %.2f should be < edge speed 15", expected)
	}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, S: 0, V: 15}}

	minV := math.Inf(1)
	for i := 0; i < 500; i++ {
		w.Step()
		if len(w.Vehicles) == 0 {
			break
		}
		v := &w.Vehicles[0]
		onApproach := v.Edge == 0 && (200-v.S) < 30
		onExit := v.Edge == 1
		if (onApproach || onExit) && v.V < minV {
			minV = v.V
		}
	}
	if minV > expected+2.0 {
		t.Errorf("did not slow enough for the 90° corner: minV=%.2f, corner speed=%.2f", minV, expected)
	}
	if minV < 1.0 {
		t.Errorf("braked to a near stop: minV=%.2f, corner speed=%.2f", minV, expected)
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

func TestCornerSpeed(t *testing.T) {
	if v := cornerSpeed(math.Inf(1)); !math.IsInf(v, 1) {
		t.Errorf("infinite radius: want +Inf, got %.3f", v)
	}
	if v := cornerSpeed(0.001); v != minCornerSpeed {
		t.Errorf("tiny radius: want floor %.2f, got %.3f", minCornerSpeed, v)
	}
	want := math.Sqrt(cornerLatAccel * 12)
	if v := cornerSpeed(12); math.Abs(v-want) > 1e-9 {
		t.Errorf("R=12: want %.3f, got %.3f", want, v)
	}
}

func TestTurnRadius(t *testing.T) {
	// Straight two-edge path -> +Inf.
	straight := &network.Network{Edges: []network.Edge{
		{ID: 0, Geometry: []network.Point{{X: 0, Y: 0}, {X: 100, Y: 0}}},
		{ID: 1, Geometry: []network.Point{{X: 100, Y: 0}, {X: 200, Y: 0}}},
	}}
	if r := turnRadius(straight, 0, 1); !math.IsInf(r, 1) {
		t.Errorf("straight path: want +Inf radius, got %.3f", r)
	}

	// 90° elbow with 15m sample arms -> right-triangle circumradius ~10.6m.
	elbow := &network.Network{Edges: []network.Edge{
		{ID: 0, Geometry: []network.Point{{X: 0, Y: 0}, {X: 100, Y: 0}}},
		{ID: 1, Geometry: []network.Point{{X: 100, Y: 0}, {X: 100, Y: -100}}},
	}}
	// Right angle at node, legs both 15 m -> circumradius = hypotenuse/2 = 15√2/2.
	want := 15 * math.Sqrt2 / 2
	if r := turnRadius(elbow, 0, 1); math.Abs(r-want) > 0.5 {
		t.Errorf("90° elbow: want ~%.2f, got %.2f", want, r)
	}

	// Jagged-but-straight: a short angled end stub on edge 0, but the road is
	// straight over the 15m sample. The corner speed must exceed a 40km/h
	// (11.2 m/s) limit so no false slowdown happens (artifact regression).
	jagged := &network.Network{Edges: []network.Edge{
		{ID: 0, Geometry: []network.Point{{X: 0, Y: 0}, {X: 98, Y: 0}, {X: 100, Y: 1.5}}},
		{ID: 1, Geometry: []network.Point{{X: 100, Y: 1.5}, {X: 200, Y: 1.5}}},
	}}
	if vs := cornerSpeed(turnRadius(jagged, 0, 1)); vs < 11.2 {
		t.Errorf("jagged-but-straight: corner speed %.2f should exceed 40km/h (no false slowdown)", vs)
	}

	// Edge with insufficient geometry -> +Inf (no constraint).
	bare := &network.Network{Edges: []network.Edge{
		{ID: 0, Geometry: []network.Point{{X: 0, Y: 0}}}, // single point
		{ID: 1, Geometry: []network.Point{{X: 0, Y: 0}, {X: 100, Y: 0}}},
	}}
	if r := turnRadius(bare, 0, 1); !math.IsInf(r, 1) {
		t.Errorf("single-point fromEdge: want +Inf, got %.3f", r)
	}
}

func TestTurnRadius_SweepingVsTight(t *testing.T) {
	// Shallow bend: large radius -> high corner speed (above a 40km/h limit).
	shallow := &network.Network{Edges: []network.Edge{
		{ID: 0, Geometry: []network.Point{{X: 0, Y: 0}, {X: 100, Y: 0}}},
		{ID: 1, Geometry: []network.Point{{X: 100, Y: 0}, {X: 200, Y: -20}}},
	}}
	// Tight 90° corner: small radius -> low corner speed.
	tight := &network.Network{Edges: []network.Edge{
		{ID: 0, Geometry: []network.Point{{X: 0, Y: 0}, {X: 100, Y: 0}}},
		{ID: 1, Geometry: []network.Point{{X: 100, Y: 0}, {X: 100, Y: -100}}},
	}}
	sV := cornerSpeed(turnRadius(shallow, 0, 1))
	tV := cornerSpeed(turnRadius(tight, 0, 1))
	if sV <= tV {
		t.Errorf("sweeping (%.2f) should allow a higher speed than tight (%.2f)", sV, tV)
	}
	if sV < 11.2 {
		t.Errorf("shallow bend corner speed %.2f should exceed 40km/h (barely slows)", sV)
	}
	if tV >= 11.2 {
		t.Errorf("tight 90° corner speed %.2f should be below 40km/h (clearly slows)", tV)
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

// TestWorld_CornerBrakingIsGentle: easing into a 90° turn must not hit the
// panic-brake clamp — the deceleration should stay well above -MaxBraking.
func TestWorld_CornerBrakingIsGentle(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		{ID: 2, Pos: network.Point{X: 200, Y: -100}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 15,
			Lanes: []network.Lane{{Index: 0}}, Geometry: []network.Point{{X: 0, Y: 0}, {X: 200, Y: 0}}},
		{ID: 1, From: 1, To: 2, Length: 100, SpeedLimit: 15,
			Lanes: []network.Lane{{Index: 0}}, Geometry: []network.Point{{X: 200, Y: 0}, {X: 200, Y: -100}}},
	}
	net := &network.Network{Nodes: nodes, Edges: edges}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, S: 0, V: 15}}

	minA := 0.0
	for i := 0; i < 600; i++ {
		w.Step()
		if len(w.Vehicles) == 0 {
			break
		}
		if a := w.Vehicles[0].A; a < minA {
			minA = a
		}
	}
	if minA < -4.0 {
		t.Errorf("corner braking too hard: peak decel %.2f m/s² (want > -4.0; MaxBraking is -%.1f)", minA, MaxBraking)
	}
}
