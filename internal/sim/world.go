package sim

import (
	"log/slog"

	"github.com/lab1702/traffic-sim/internal/network"
)

// World owns mutable simulation state. Only the sim goroutine touches it.
type World struct {
	Net     *network.Network
	Router  *Router
	Spawner Spawner
	Vehicles []Vehicle
	Tick    uint64
	SimTime float64
	dt      float64

	nextID   VehicleID
	maxRetry int // spawn retries per tick before giving up
}

const DefaultDt = 0.05 // 50 ms == 20 Hz

func NewWorld(net *network.Network, spawner Spawner) *World {
	return &World{
		Net:      net,
		Router:   NewRouter(net),
		Spawner:  spawner,
		dt:       DefaultDt,
		maxRetry: 4,
	}
}

// Step advances the sim by one tick (DefaultDt seconds).
func (w *World) Step() {
	// 1. Demand.
	reqs := w.Spawner.Tick(w.SimTime, w.dt)
	for _, r := range reqs {
		w.trySpawn(r)
	}

	// 2. Bucket vehicles by edge for leader lookup, sorted by S ascending.
	byEdge := make(map[network.EdgeID][]int, 1024)
	for i := range w.Vehicles {
		if w.Vehicles[i].Despawned {
			continue
		}
		eid := w.Vehicles[i].Edge
		byEdge[eid] = append(byEdge[eid], i)
	}
	for _, idxs := range byEdge {
		sortVehicleIdxByS(w.Vehicles, idxs)
	}

	// 3. Step each vehicle, finding its leader as the next vehicle ahead
	//    on the same edge (or the first vehicle on the next route edge
	//    if no same-edge leader exists and gap to end-of-edge is small).
	//    Iterate vehicles in a stable index order to preserve determinism.
	for i := range w.Vehicles {
		if w.Vehicles[i].Despawned {
			continue
		}
		v := &w.Vehicles[i]
		idxs := byEdge[v.Edge]
		// Find this vehicle's position within the sorted bucket.
		pos := -1
		for k, vi := range idxs {
			if vi == i {
				pos = k
				break
			}
		}

		var lS, lV float64
		has := false
		if pos >= 0 && pos+1 < len(idxs) {
			ld := &w.Vehicles[idxs[pos+1]]
			lS, lV, has = ld.S, ld.V, true
		} else if v.RouteIdx+1 < len(v.Route) {
			// Lookahead to next edge's first vehicle.
			nextE := v.Route[v.RouteIdx+1]
			if nidxs, ok := byEdge[nextE]; ok && len(nidxs) > 0 {
				nv := &w.Vehicles[nidxs[0]]
				edge := &w.Net.Edges[v.Edge]
				lS = edge.Length + nv.S
				lV = nv.V
				has = true
			}
		}
		stepIDM(v, lS, lV, has, w.Net, DefaultIDM(), w.dt)
	}

	// 4. Compact and advance time.
	w.compact()
	w.Tick++
	w.SimTime += w.dt
}

// sortVehicleIdxByS sorts idxs ascending by Vehicles[i].S (insertion sort;
// fine for small per-edge counts).
func sortVehicleIdxByS(vs []Vehicle, idxs []int) {
	for i := 1; i < len(idxs); i++ {
		for j := i; j > 0 && vs[idxs[j-1]].S > vs[idxs[j]].S; j-- {
			idxs[j-1], idxs[j] = idxs[j], idxs[j-1]
		}
	}
}

func (w *World) trySpawn(r SpawnRequest) {
	route, err := w.Router.Route(r.OriginNode, r.DestNode)
	if err != nil || len(route) == 0 {
		return
	}
	// Spawn at the edge speed limit so vehicles enter at cruising speed.
	// IDM will regulate from there (following, braking) as needed.
	v := Vehicle{
		ID:    w.nextID,
		Route: route,
		Edge:  route[0],
		Lane:  0,
		S:     0,
		V:     w.Net.Edges[route[0]].SpeedLimit,
	}
	w.nextID++
	w.Vehicles = append(w.Vehicles, v)
}

func (w *World) compact() {
	dst := 0
	for _, v := range w.Vehicles {
		if v.Despawned {
			continue
		}
		w.Vehicles[dst] = v
		dst++
	}
	w.Vehicles = w.Vehicles[:dst]
}

// Run advances the sim for the given number of simulated seconds (headless).
// Logs basic progress every 1s of sim time.
func (w *World) Run(durationSec float64) {
	lastLog := w.SimTime
	target := w.SimTime + durationSec
	for w.SimTime < target {
		w.Step()
		if w.SimTime-lastLog >= 1.0 {
			slog.Info("sim progress",
				"sim_time", w.SimTime,
				"vehicles", len(w.Vehicles),
				"tick", w.Tick,
			)
			lastLog = w.SimTime
		}
	}
}
