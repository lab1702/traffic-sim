package sim

import (
	"bytes"
	"log/slog"
	"strings"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

// 4-way intersection at node 2. Edges:
//
//	0: 1->2 incoming from west (heading east)
//	1: 2->3 right turn (south)
//	2: 2->4 straight (east)
//	3: 2->5 left turn (north)
//
// Outgoing edges have configurable lane counts.
func makeCarryoverNet(outNumLanes int) *network.Network {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}}, // unused placeholder
		{ID: 1, Pos: network.Point{X: -100, Y: 0}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 0, Y: -100}},
		{ID: 4, Pos: network.Point{X: 100, Y: 0}},
		{ID: 5, Pos: network.Point{X: 0, Y: 100}},
	}
	outLanes := make([]network.Lane, outNumLanes)
	for i := range outLanes {
		outLanes[i] = network.Lane{Index: uint8(i)}
	}
	edges := []network.Edge{
		{ID: 0, From: 1, To: 2, Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}, {Index: 1}, {Index: 2}},
			Geometry: []network.Point{nodes[1].Pos, nodes[2].Pos}},
		{ID: 1, From: 2, To: 3, Length: 100, SpeedLimit: 10,
			Lanes:    append([]network.Lane(nil), outLanes...),
			Geometry: []network.Point{nodes[2].Pos, nodes[3].Pos}},
		{ID: 2, From: 2, To: 4, Length: 100, SpeedLimit: 10,
			Lanes:    append([]network.Lane(nil), outLanes...),
			Geometry: []network.Point{nodes[2].Pos, nodes[4].Pos}},
		{ID: 3, From: 2, To: 5, Length: 100, SpeedLimit: 10,
			Lanes:    append([]network.Lane(nil), outLanes...),
			Geometry: []network.Point{nodes[2].Pos, nodes[5].Pos}},
	}
	return &network.Network{Nodes: nodes, Edges: edges}
}

func TestStepIDM_LaneCarryOver_RightTurn_SnapsToLane0(t *testing.T) {
	net := makeCarryoverNet(3)
	v := &Vehicle{
		ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, Lane: 2,
		S: 99, V: 10,
	}
	stepIDM(v, 10, 0, 0, false, net, DefaultIDM(), 1.0)
	if v.Edge != 1 {
		t.Fatalf("expected edge 1 after crossing, got %d", v.Edge)
	}
	if v.Lane != 0 {
		t.Errorf("right turn should snap to lane 0, got %d", v.Lane)
	}
}

func TestStepIDM_LaneCarryOver_LeftTurn_SnapsToLastLane(t *testing.T) {
	net := makeCarryoverNet(3)
	v := &Vehicle{
		ID: 1, Route: []network.EdgeID{0, 3}, Edge: 0, Lane: 0,
		S: 99, V: 10,
	}
	stepIDM(v, 10, 0, 0, false, net, DefaultIDM(), 1.0)
	if v.Edge != 3 {
		t.Fatalf("expected edge 3 after crossing, got %d", v.Edge)
	}
	if int(v.Lane) != 2 {
		t.Errorf("left turn should snap to lane N-1 (=2), got %d", v.Lane)
	}
}

func TestStepIDM_LaneCarryOver_Straight_PreservesLane(t *testing.T) {
	net := makeCarryoverNet(3)
	v := &Vehicle{
		ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, Lane: 1,
		S: 99, V: 10,
	}
	stepIDM(v, 10, 0, 0, false, net, DefaultIDM(), 1.0)
	if v.Edge != 2 {
		t.Fatalf("expected edge 2 after crossing, got %d", v.Edge)
	}
	if v.Lane != 1 {
		t.Errorf("straight should preserve lane, got %d", v.Lane)
	}
}

func TestStepIDM_LaneCarryOver_Straight_ClampsWhenNarrowing(t *testing.T) {
	net := makeCarryoverNet(1)
	v := &Vehicle{
		ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, Lane: 2,
		S: 99, V: 10,
	}
	stepIDM(v, 10, 0, 0, false, net, DefaultIDM(), 1.0)
	if v.Lane != 0 {
		t.Errorf("straight onto 1-lane should clamp to 0, got %d", v.Lane)
	}
}

func TestStepIDM_LaneCarryOver_EmitsSnapWarning(t *testing.T) {
	net := makeCarryoverNet(3)
	// Populate AllowedTurns so lane 0 is right-only, lane 2 is left-only.
	net.Edges[0].Lanes[0].AllowedTurns = []network.EdgeID{1}
	net.Edges[0].Lanes[1].AllowedTurns = []network.EdgeID{2}
	net.Edges[0].Lanes[2].AllowedTurns = []network.EdgeID{3}

	var buf bytes.Buffer
	prev := slog.Default()
	slog.SetDefault(slog.New(slog.NewTextHandler(&buf, &slog.HandlerOptions{Level: slog.LevelDebug})))
	t.Cleanup(func() { slog.SetDefault(prev) })

	// Vehicle in lane 0 (right-only) trying to go left (edge 3).
	v := &Vehicle{
		ID: 42, Route: []network.EdgeID{0, 3}, Edge: 0, Lane: 0,
		S: 99, V: 10,
	}
	stepIDM(v, 10, 0, 0, false, net, DefaultIDM(), 1.0)

	if !strings.Contains(buf.String(), "turn-lane snap fallback") {
		t.Errorf("expected snap-fallback warning; log was: %s", buf.String())
	}
	if !strings.Contains(buf.String(), "vehicle_id=42") {
		t.Errorf("expected vehicle_id=42 in warning; log was: %s", buf.String())
	}
}

func TestStepIDM_LaneCarryOver_NoWarningWhenBiasSucceeded(t *testing.T) {
	net := makeCarryoverNet(3)
	net.Edges[0].Lanes[0].AllowedTurns = []network.EdgeID{1}
	net.Edges[0].Lanes[1].AllowedTurns = []network.EdgeID{2}
	net.Edges[0].Lanes[2].AllowedTurns = []network.EdgeID{3}

	var buf bytes.Buffer
	prev := slog.Default()
	slog.SetDefault(slog.New(slog.NewTextHandler(&buf, &slog.HandlerOptions{Level: slog.LevelDebug})))
	t.Cleanup(func() { slog.SetDefault(prev) })

	// Vehicle in lane 2 (left-compatible) taking the left turn (edge 3).
	v := &Vehicle{
		ID: 43, Route: []network.EdgeID{0, 3}, Edge: 0, Lane: 2,
		S: 99, V: 10,
	}
	stepIDM(v, 10, 0, 0, false, net, DefaultIDM(), 1.0)

	if strings.Contains(buf.String(), "turn-lane snap fallback") {
		t.Errorf("expected NO snap-fallback warning when bias succeeded; log: %s", buf.String())
	}
}
