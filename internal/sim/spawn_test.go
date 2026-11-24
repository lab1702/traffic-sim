package sim

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

// floodSpawner requests one vehicle per tick from Origin to Dest — far more
// than the origin edge can absorb, so the World must refuse most of them.
type floodSpawner struct{ Origin, Dest network.NodeID }

func (f floodSpawner) Tick(float64, float64) []SpawnRequest {
	return []SpawnRequest{{OriginNode: f.Origin, DestNode: f.Dest}}
}

// minSameLaneGap returns the smallest bumper gap between consecutive
// vehicles sharing an (edge, lane), or +Inf if no lane holds two vehicles.
func minSameLaneGap(w *World) float64 {
	best := 1e18
	by := map[edgeLane][]float64{}
	for i := range w.Vehicles {
		v := &w.Vehicles[i]
		if v.Despawned {
			continue
		}
		k := edgeLane{v.Edge, v.Lane}
		by[k] = append(by[k], v.S)
	}
	for _, ss := range by {
		for i := range ss {
			for j := range ss {
				if ss[j] > ss[i] || (ss[j] == ss[i] && j > i) {
					if g := ss[j] - ss[i] - VehicleLength; g < best {
						best = g
					}
				}
			}
		}
	}
	return best
}

// TestSpawn_NeverOverlapsUnderFlood: requesting a spawn every tick must not
// stack vehicles on the origin edge. Before the fix every request spawned at
// S=0 at full speed, so vehicles appeared inside one another.
func TestSpawn_NeverOverlapsUnderFlood(t *testing.T) {
	net := buildLineGraph()
	w := NewWorld(net, floodSpawner{Origin: 0, Dest: 3}, nil)
	for i := 0; i < 600; i++ { // 30 s
		w.Step()
		if g := minSameLaneGap(w); g < 0 {
			t.Fatalf("tick %d: vehicles overlap (min bumper gap %.2f m)", w.Tick, g)
		}
	}
	if w.nextID < 5 {
		t.Fatalf("only %d vehicles spawned in 30 s; spawning should still proceed when there is room", w.nextID)
	}
}

// TestSpawn_BlockedByQueueTail: a stopped vehicle sitting at the very start
// of the origin edge blocks the spawn outright.
func TestSpawn_BlockedByQueueTail(t *testing.T) {
	net := buildLineGraph()
	w := NewWorld(net, nil, nil)
	w.Vehicles = []Vehicle{{ID: 100, Route: []network.EdgeID{0, 1}, Edge: 0, S: 2, V: 0}}
	w.trySpawn(SpawnRequest{OriginNode: 0, DestNode: 3}, w.buildSpawnOccupancy())
	if len(w.Vehicles) != 1 {
		t.Fatalf("spawned on top of a queue tail at S=2: %d vehicles", len(w.Vehicles))
	}
}

// TestSpawn_SlowsBehindStoppedVehicle: with room but a stopped vehicle ahead,
// the newcomer enters slowly enough to stop behind it comfortably instead of
// at full cruising speed.
func TestSpawn_SlowsBehindStoppedVehicle(t *testing.T) {
	net := buildLineGraph()
	w := NewWorld(net, nil, nil)
	w.Vehicles = []Vehicle{{ID: 100, Route: []network.EdgeID{0, 1}, Edge: 0, S: 30, V: 0}}
	w.trySpawn(SpawnRequest{OriginNode: 0, DestNode: 3}, w.buildSpawnOccupancy())
	if len(w.Vehicles) != 2 {
		t.Fatalf("expected a spawn with 25 m of room, got %d vehicles", len(w.Vehicles))
	}
	nv := w.Vehicles[1]
	p := DefaultIDM()
	gap := 30.0 - VehicleLength
	if nv.V*nv.V/(2*p.B) > gap-p.S0+1e-9 {
		t.Fatalf("entry speed %.2f m/s cannot stop comfortably in %.1f m", nv.V, gap-p.S0)
	}
}

// TestSpawn_BlockedByApproachingUpstreamVehicle: a vehicle about to cross
// the origin node onto the spawn edge at speed must not have a car appear
// just in front of it.
func TestSpawn_BlockedByApproachingUpstreamVehicle(t *testing.T) {
	net := buildLineGraph()
	w := NewWorld(net, nil, nil)
	// On edge 0 (node 0 -> 1), 3 m from node 1, heading onto edge 1 at 10 m/s.
	w.Vehicles = []Vehicle{{ID: 100, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 97, V: 10}}
	w.trySpawn(SpawnRequest{OriginNode: 1, DestNode: 3}, w.buildSpawnOccupancy())
	if len(w.Vehicles) != 1 {
		t.Fatalf("spawned directly in front of a vehicle crossing the origin node")
	}
}

// noOccupancy is an empty spawn index: trySpawn with it skips the room
// checks. For tests and benchmarks that seed many vehicles without stepping
// and care about sampling, not spawn physics.
func noOccupancy() *spawnOccupancy {
	return &spawnOccupancy{rear: map[edgeLane]int{}, onEdge: map[network.EdgeID][]int{}}
}
