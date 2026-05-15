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
	// 1. Handle demand.
	reqs := w.Spawner.Tick(w.SimTime, w.dt)
	for _, r := range reqs {
		w.trySpawn(r)
	}

	// 2. Step vehicles.
	for i := range w.Vehicles {
		stepConstantVelocity(&w.Vehicles[i], w.Net, w.dt)
	}

	// 3. Garbage-collect despawned vehicles (compact in place).
	w.compact()

	w.Tick++
	w.SimTime += w.dt
}

func (w *World) trySpawn(r SpawnRequest) {
	route, err := w.Router.Route(r.OriginNode, r.DestNode)
	if err != nil || len(route) == 0 {
		return
	}
	v := Vehicle{
		ID:    w.nextID,
		Route: route,
		Edge:  route[0],
		Lane:  0,
		S:     0,
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
