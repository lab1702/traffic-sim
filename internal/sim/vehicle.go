package sim

import "github.com/lab1702/traffic-sim/internal/network"

type VehicleID uint32

// Vehicle is the simulated agent. In Phase 3 it moves at constant speed
// equal to the current edge's speed limit; IDM comes in Phase 4.
type Vehicle struct {
	ID       VehicleID
	Route    []network.EdgeID
	RouteIdx int            // index into Route of the current edge
	Edge     network.EdgeID
	Lane     uint8
	S        float64 // meters along edge
	V        float64 // m/s
	A        float64 // m/s^2 (unused in Phase 3)

	Despawned bool
}

// stepConstantVelocity advances a vehicle by dt seconds along its route at
// the current edge's speed limit. When it reaches the end of the route it
// is marked despawned; intermediate edge transitions roll S over.
func stepConstantVelocity(v *Vehicle, net *network.Network, dt float64) {
	if v.Despawned {
		return
	}
	edge := &net.Edges[v.Edge]
	v.V = edge.SpeedLimit
	v.S += v.V * dt
	for v.S >= edge.Length {
		v.S -= edge.Length
		v.RouteIdx++
		if v.RouteIdx >= len(v.Route) {
			v.Despawned = true
			v.S = 0
			return
		}
		v.Edge = v.Route[v.RouteIdx]
		edge = &net.Edges[v.Edge]
	}
}
