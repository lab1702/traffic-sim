package sim

import (
	"log/slog"
	"math"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
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

	// SignalStates is indexed by IntersectionID; nil entries mean no signal.
	SignalStates []*SignalState

	// xByNodeID is a NodeID -> Intersection index for O(1) lookup during tick.
	xByNodeID map[network.NodeID]*network.Intersection

	// SnapshotBuf is read by the renderer; published once per tick.
	SnapshotBuf *snapshot.Buffer
}

const DefaultDt = 0.05 // 50 ms == 20 Hz

func NewWorld(net *network.Network, spawner Spawner, overrides map[network.IntersectionID]SignalConfig) *World {
	sigs := make([]*SignalState, len(net.Intersections))
	xByNode := make(map[network.NodeID]*network.Intersection, len(net.Intersections))
	for i := range net.Intersections {
		x := &net.Intersections[i]
		xByNode[x.NodeID] = x
		if x.HasSignal {
			if cfg, ok := overrides[x.ID]; ok {
				sigs[x.ID] = NewSignalState(cfg)
			} else {
				sigs[x.ID] = NewSignalState(DefaultSignalConfig(x.Incoming))
			}
		}
	}
	return &World{
		Net:          net,
		Router:       NewRouter(net),
		Spawner:      spawner,
		dt:           DefaultDt,
		maxRetry:     4,
		SignalStates: sigs,
		xByNodeID:    xByNode,
		SnapshotBuf:  snapshot.New(),
	}
}

// stopDistanceForRed returns (distance to stop line, true) if the vehicle
// is on an incoming edge to a red-signalled intersection and the vehicle
// is approaching it. Returns (0, false) otherwise.
func (w *World) stopDistanceForRed(v *Vehicle) (float64, bool) {
	edge := &w.Net.Edges[v.Edge]
	x, ok := w.xByNodeID[edge.To]
	if !ok {
		return 0, false
	}
	if !x.HasSignal {
		return 0, false
	}
	st := w.SignalStates[x.ID]
	if st == nil {
		return 0, false
	}
	pos := IncomingPos(x, v.Edge)
	if pos < 0 {
		return 0, false
	}
	if st.GreenFor(pos) {
		return 0, false
	}
	// Red: stop line is at the end of this edge.
	dist := edge.Length - v.S
	if dist < 0 {
		dist = 0
	}
	return dist, true
}

const gapThresholdSec = 3.0

// stopDistanceForYield returns (distance to stop line, true) if the
// vehicle's current edge ends at an UNSIGNALIZED intersection AND a
// higher-priority incoming edge has a vehicle approaching within
// gapThresholdSec seconds. "Higher priority" is defined here as a lower
// Incoming index (i.e., x.Incoming[0] is the priority road).
func (w *World) stopDistanceForYield(v *Vehicle, byEdge map[network.EdgeID][]int) (float64, bool) {
	edge := &w.Net.Edges[v.Edge]
	x, ok := w.xByNodeID[edge.To]
	if !ok || x.HasSignal {
		return 0, false
	}
	myPos := IncomingPos(x, v.Edge)
	if myPos <= 0 {
		// No higher-priority edge; we're the priority road (or unknown).
		return 0, false
	}
	myDist := edge.Length - v.S
	for i := 0; i < myPos; i++ {
		otherEdgeID := x.Incoming[i]
		others := byEdge[otherEdgeID]
		if len(others) == 0 {
			continue
		}
		// Find the closest-to-stop-line vehicle on the other approach.
		otherEdge := &w.Net.Edges[otherEdgeID]
		bestETA := 1e9
		for _, oi := range others {
			ov := &w.Vehicles[oi]
			d := otherEdge.Length - ov.S
			ovV := ov.V
			if ovV < 0.5 {
				ovV = 0.5
			}
			eta := d / ovV
			if eta < bestETA {
				bestETA = eta
			}
		}
		if bestETA < gapThresholdSec {
			return myDist, true
		}
	}
	return myDist, false
}

// Step advances the sim by one tick (DefaultDt seconds).
func (w *World) Step() {
	// 0. Advance all signal phases.
	for _, s := range w.SignalStates {
		if s != nil {
			s.Advance(w.dt)
		}
	}

	// 1. Demand.
	reqs := w.Spawner.Tick(w.SimTime, w.dt)
	for _, r := range reqs {
		w.trySpawn(r)
	}

	// 2. Bucket vehicles by edge and by (edge, lane) for leader lookup,
	//    sorted by S ascending within each bucket.
	byEdge := make(map[network.EdgeID][]int, 1024)
	byEdgeLane := make(map[network.EdgeID]map[uint8][]int, 1024)
	for i := range w.Vehicles {
		if w.Vehicles[i].Despawned {
			continue
		}
		eid := w.Vehicles[i].Edge
		byEdge[eid] = append(byEdge[eid], i)
		if _, ok := byEdgeLane[eid]; !ok {
			byEdgeLane[eid] = make(map[uint8][]int)
		}
		byEdgeLane[eid][w.Vehicles[i].Lane] = append(byEdgeLane[eid][w.Vehicles[i].Lane], i)
	}
	for _, idxs := range byEdge {
		sortVehicleIdxByS(w.Vehicles, idxs)
	}
	for _, lanes := range byEdgeLane {
		for _, idxs := range lanes {
			sortVehicleIdxByS(w.Vehicles, idxs)
		}
	}

	// 3. Pre-compute leaders per vehicle index using per-lane buckets.
	//    This is done in a separate pass to avoid order-dependence during stepping.
	type leaderInfo struct {
		lS, lV float64
		has    bool
	}
	leaders := make(map[int]leaderInfo, len(w.Vehicles))
	for eid, lanes := range byEdgeLane {
		edge := &w.Net.Edges[eid]
		for ln, idxs := range lanes {
			for pos, vi := range idxs {
				info := leaderInfo{}
				if pos+1 < len(idxs) {
					ld := &w.Vehicles[idxs[pos+1]]
					info.lS, info.lV, info.has = ld.S, ld.V, true
				} else {
					v := &w.Vehicles[vi]
					if v.RouteIdx+1 < len(v.Route) {
						nextE := v.Route[v.RouteIdx+1]
						if nlanes, ok := byEdgeLane[nextE]; ok {
							// Use first vehicle in the same lane index on the next edge if exists.
							if ne := &w.Net.Edges[nextE]; uint8(ln) < uint8(len(ne.Lanes)) {
								if nidxs, ok2 := nlanes[ln]; ok2 && len(nidxs) > 0 {
									nv := &w.Vehicles[nidxs[0]]
									info.lS = edge.Length + nv.S
									info.lV = nv.V
									info.has = true
								}
							}
						}
					}
				}
				leaders[vi] = info
			}
		}
	}

	// 4. Step each vehicle, applying signal/yield virtual leaders.
	//    Iterate vehicles in stable index order to preserve determinism.
	for i := range w.Vehicles {
		if w.Vehicles[i].Despawned {
			continue
		}
		v := &w.Vehicles[i]
		info := leaders[i]
		lS, lV, has := info.lS, info.lV, info.has

		// Apply red-light virtual leader if closer.
		if d, isRed := w.stopDistanceForRed(v); isRed {
			virtualS := v.S + d
			if !has || virtualS < lS {
				lS, lV, has = virtualS, 0, true
			}
		}
		// Apply unsignalized-yield virtual leader if closer.
		if d, mustYield := w.stopDistanceForYield(v, byEdge); mustYield {
			virtualS := v.S + d
			if !has || virtualS < lS {
				lS, lV, has = virtualS, 0, true
			}
		}

		stepIDM(v, lS, lV, has, w.Net, DefaultIDM(), w.dt)

		// Decrement lane-change cooldown.
		if v.LaneChangeCooldown > 0 {
			v.LaneChangeCooldown -= w.dt
			if v.LaneChangeCooldown < 0 {
				v.LaneChangeCooldown = 0
			}
		}
		// Try lane change after stepping (byEdgeLane is a tick-old snapshot —
		// consistent and avoids order-dependence).
		if lanes, ok := byEdgeLane[v.Edge]; ok {
			tryLaneChange(v, i, lanes, w.Vehicles, w.Net)
		}
	}

	// 5. Publish snapshot for renderer (before compact so live vehicles are included).
	w.publishSnapshot()

	// 6. Compact and advance time.
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

func (w *World) publishSnapshot() {
	if w.SnapshotBuf == nil {
		return
	}
	views := make([]snapshot.VehicleView, 0, len(w.Vehicles))
	for i := range w.Vehicles {
		v := &w.Vehicles[i]
		if v.Despawned {
			continue
		}
		x, y, hd := positionOnEdge(w.Net, v.Edge, v.S)
		views = append(views, snapshot.VehicleView{
			ID: uint32(v.ID), X: x, Y: y, Heading: hd, Speed: v.V,
		})
	}
	sigs := make([]snapshot.SignalView, 0, len(w.SignalStates))
	for i, st := range w.SignalStates {
		if st == nil {
			continue
		}
		x := &w.Net.Intersections[i]
		node := w.Net.Nodes[x.NodeID]
		// "Is red" for visualization: red if no incoming edge is currently green.
		isRed := true
		for j := range x.Incoming {
			if st.GreenFor(j) {
				isRed = false
				break
			}
		}
		sigs = append(sigs, snapshot.SignalView{
			IntersectionID: uint32(x.ID),
			X:              node.Pos.X, Y: node.Pos.Y,
			IsRed: isRed, IsYellow: st.IsYellow,
		})
	}
	w.SnapshotBuf.Publish(snapshot.Snapshot{
		Tick: w.Tick, SimTime: w.SimTime,
		Vehicles: views, Signals: sigs, Bounds: w.Net.Bounds,
	})
}

// positionOnEdge returns (x, y, heading) for the point S meters along
// edge's polyline geometry. Linear interpolation between vertices.
func positionOnEdge(net *network.Network, eid network.EdgeID, s float64) (float64, float64, float64) {
	e := &net.Edges[eid]
	g := e.Geometry
	if len(g) < 2 {
		return 0, 0, 0
	}
	remaining := s
	for i := 1; i < len(g); i++ {
		dx := g[i].X - g[i-1].X
		dy := g[i].Y - g[i-1].Y
		segLen := math.Sqrt(dx*dx + dy*dy)
		if remaining <= segLen || i == len(g)-1 {
			t := 0.0
			if segLen > 0 {
				t = remaining / segLen
			}
			if t > 1 {
				t = 1
			}
			x := g[i-1].X + dx*t
			y := g[i-1].Y + dy*t
			heading := math.Atan2(dy, dx)
			return x, y, heading
		}
		remaining -= segLen
	}
	return g[len(g)-1].X, g[len(g)-1].Y, 0
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
