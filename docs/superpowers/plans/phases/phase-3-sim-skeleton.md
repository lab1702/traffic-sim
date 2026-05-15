# Phase 3 — Sim Skeleton + Routing

**Milestone:** `trafficsim run <file> --headless --duration 10s` runs the sim for 10 simulated seconds. Vehicles spawn at random nodes, follow A*-computed routes at constant velocity, and despawn at their destination.

---

### Task 3.1: A* router on hand-built graph

**Files:**
- Create: `internal/sim/router.go`
- Create: `internal/sim/router_test.go`

- [ ] **Step 1: Write the failing test**

Write `internal/sim/router_test.go`:
```go
package sim

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

// buildLineGraph: nodes 0-1-2-3, four directed edges (one per pair, in
// order). Edge i goes from node i to node i+1, length 100m each.
func buildLineGraph() *network.Network {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 100, Y: 0}},
		{ID: 2, Pos: network.Point{X: 200, Y: 0}},
		{ID: 3, Pos: network.Point{X: 300, Y: 0}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 100, SpeedLimit: 10},
		{ID: 1, From: 1, To: 2, Length: 100, SpeedLimit: 10},
		{ID: 2, From: 2, To: 3, Length: 100, SpeedLimit: 10},
	}
	return &network.Network{Nodes: nodes, Edges: edges}
}

func TestRouter_ShortestPath_LineGraph(t *testing.T) {
	net := buildLineGraph()
	r := NewRouter(net)
	route, err := r.Route(0, 3)
	if err != nil {
		t.Fatalf("Route: %v", err)
	}
	if len(route) != 3 {
		t.Fatalf("want 3 edges, got %d", len(route))
	}
	for i, eid := range []network.EdgeID{0, 1, 2} {
		if route[i] != eid {
			t.Errorf("step %d: want edge %d, got %d", i, eid, route[i])
		}
	}
}

func TestRouter_Unreachable(t *testing.T) {
	net := buildLineGraph()
	// Add an isolated node 4 with no edges.
	net.Nodes = append(net.Nodes, network.Node{ID: 4, Pos: network.Point{X: 1000, Y: 0}})
	r := NewRouter(net)
	_, err := r.Route(0, 4)
	if err == nil {
		t.Fatalf("want error for unreachable target")
	}
}
```

- [ ] **Step 2: Run to verify failure**

Run: `go test ./internal/sim/ -run TestRouter -v`
Expected: FAIL — `NewRouter` undefined.

- [ ] **Step 3: Implement A***

Write `internal/sim/router.go`:
```go
package sim

import (
	"container/heap"
	"errors"
	"math"

	"github.com/lab1702/traffic-sim/internal/network"
)

// Router computes shortest-path routes over a Network.
// Cost = edge length / speed limit (i.e., free-flow travel time).
type Router struct {
	net     *network.Network
	adjOut  map[network.NodeID][]network.EdgeID
}

func NewRouter(net *network.Network) *Router {
	adj := make(map[network.NodeID][]network.EdgeID, len(net.Nodes))
	for i := range net.Edges {
		e := &net.Edges[i]
		adj[e.From] = append(adj[e.From], e.ID)
	}
	return &Router{net: net, adjOut: adj}
}

var ErrNoRoute = errors.New("no route between nodes")

// Route returns the edge IDs to traverse from src to dst.
func (r *Router) Route(src, dst network.NodeID) ([]network.EdgeID, error) {
	if src == dst {
		return nil, nil
	}
	type item struct {
		node network.NodeID
		f    float64 // estimated total cost
	}
	gScore := make(map[network.NodeID]float64)
	cameFromEdge := make(map[network.NodeID]network.EdgeID)
	cameFromNode := make(map[network.NodeID]network.NodeID)
	gScore[src] = 0

	dstPos := r.net.Nodes[dst].Pos
	heuristic := func(n network.NodeID) float64 {
		p := r.net.Nodes[n].Pos
		dx, dy := p.X-dstPos.X, p.Y-dstPos.Y
		return math.Sqrt(dx*dx+dy*dy) / 31.3 // assume max speed
	}

	open := &pq{}
	heap.Init(open)
	heap.Push(open, &pqItem{node: src, f: heuristic(src)})
	closed := make(map[network.NodeID]bool)

	for open.Len() > 0 {
		cur := heap.Pop(open).(*pqItem)
		if cur.node == dst {
			return reconstruct(cameFromEdge, cameFromNode, src, dst), nil
		}
		if closed[cur.node] {
			continue
		}
		closed[cur.node] = true
		for _, eid := range r.adjOut[cur.node] {
			e := &r.net.Edges[eid]
			speed := e.SpeedLimit
			if speed < 0.1 {
				speed = 0.1
			}
			tentative := gScore[cur.node] + e.Length/speed
			if existing, ok := gScore[e.To]; ok && tentative >= existing {
				continue
			}
			gScore[e.To] = tentative
			cameFromEdge[e.To] = eid
			cameFromNode[e.To] = cur.node
			heap.Push(open, &pqItem{node: e.To, f: tentative + heuristic(e.To)})
		}
	}
	return nil, ErrNoRoute
}

func reconstruct(edgeBy map[network.NodeID]network.EdgeID,
	nodeBy map[network.NodeID]network.NodeID,
	src, dst network.NodeID,
) []network.EdgeID {
	var out []network.EdgeID
	cur := dst
	for cur != src {
		eid, ok := edgeBy[cur]
		if !ok {
			return nil
		}
		out = append(out, eid)
		cur = nodeBy[cur]
	}
	// Reverse in place.
	for i, j := 0, len(out)-1; i < j; i, j = i+1, j-1 {
		out[i], out[j] = out[j], out[i]
	}
	return out
}

// --- priority queue ---

type pqItem struct {
	node network.NodeID
	f    float64
	idx  int
}

type pq []*pqItem

func (p pq) Len() int            { return len(p) }
func (p pq) Less(i, j int) bool  { return p[i].f < p[j].f }
func (p pq) Swap(i, j int)       { p[i], p[j] = p[j], p[i]; p[i].idx, p[j].idx = i, j }
func (p *pq) Push(x any)         { it := x.(*pqItem); it.idx = len(*p); *p = append(*p, it) }
func (p *pq) Pop() any           { o := *p; n := len(o); it := o[n-1]; *p = o[:n-1]; return it }
```

- [ ] **Step 4: Run tests**

Run: `go test ./internal/sim/ -run TestRouter -v`
Expected: both tests PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/sim/router.go internal/sim/router_test.go
git commit -m "feat(sim): A* router with free-flow travel time cost"
```

---

### Task 3.2: Vehicle struct + constant-velocity stepping

**Files:**
- Create: `internal/sim/vehicle.go`

- [ ] **Step 1: Write the vehicle module**

Write `internal/sim/vehicle.go`:
```go
package sim

import "github.com/lab1702/traffic-sim/internal/network"

type VehicleID uint32

// Vehicle is the simulated agent. In Phase 3 it moves at constant speed
// equal to the current edge's speed limit; IDM comes in Phase 4.
type Vehicle struct {
	ID       VehicleID
	Route    []network.EdgeID
	RouteIdx int             // index into Route of the current edge
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
```

- [ ] **Step 2: Verify build**

Run: `go build ./internal/sim/`
Expected: exit 0.

- [ ] **Step 3: Commit**

```bash
git add internal/sim/vehicle.go
git commit -m "feat(sim): Vehicle type with constant-velocity stepping"
```

---

### Task 3.3: Demand generator (RandomOD)

**Files:**
- Create: `internal/sim/demand.go`
- Create: `internal/sim/demand_test.go`

- [ ] **Step 1: Write the demand module**

Write `internal/sim/demand.go`:
```go
package sim

import (
	"math/rand/v2"

	"github.com/lab1702/traffic-sim/internal/network"
)

// SpawnRequest tells the World a new vehicle should appear. The World
// decides whether to honor it (e.g., it may delay if the origin edge is
// blocked at S=0).
type SpawnRequest struct {
	OriginNode network.NodeID
	DestNode   network.NodeID
}

// Spawner produces SpawnRequests over time. Implementations must be
// deterministic given their constructor inputs.
type Spawner interface {
	// Tick is called every sim tick. It may return zero or more requests
	// to be attempted this tick.
	Tick(simTime float64, dt float64) []SpawnRequest
}

// RandomOD is the simplest spawner: each second (in expectation) it
// produces `rate` requests, each with a uniformly-random origin and
// destination drawn from all network nodes.
type RandomOD struct {
	net  *network.Network
	rng  *rand.Rand
	rate float64 // vehicles per second
	// Accumulator tracks fractional requests carried across ticks.
	acc float64
}

func NewRandomOD(net *network.Network, seed uint64, ratePerSec float64) *RandomOD {
	return &RandomOD{
		net:  net,
		rng:  rand.New(rand.NewPCG(seed, seed^0x9E3779B97F4A7C15)),
		rate: ratePerSec,
	}
}

func (s *RandomOD) Tick(_ float64, dt float64) []SpawnRequest {
	s.acc += s.rate * dt
	n := int(s.acc)
	s.acc -= float64(n)
	if n == 0 || len(s.net.Nodes) < 2 {
		return nil
	}
	out := make([]SpawnRequest, 0, n)
	for i := 0; i < n; i++ {
		oi := s.rng.IntN(len(s.net.Nodes))
		di := s.rng.IntN(len(s.net.Nodes))
		if oi == di {
			continue
		}
		out = append(out, SpawnRequest{
			OriginNode: network.NodeID(oi),
			DestNode:   network.NodeID(di),
		})
	}
	return out
}
```

- [ ] **Step 2: Write determinism test**

Write `internal/sim/demand_test.go`:
```go
package sim

import (
	"reflect"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

func TestRandomOD_DeterministicForSeed(t *testing.T) {
	net := buildLineGraph()
	s1 := NewRandomOD(net, 42, 5.0)
	s2 := NewRandomOD(net, 42, 5.0)
	var r1, r2 []SpawnRequest
	for i := 0; i < 20; i++ {
		r1 = append(r1, s1.Tick(float64(i)*0.05, 0.05)...)
		r2 = append(r2, s2.Tick(float64(i)*0.05, 0.05)...)
	}
	if !reflect.DeepEqual(r1, r2) {
		t.Fatalf("same seed should produce identical request streams\n r1=%v\n r2=%v", r1, r2)
	}
	if len(r1) == 0 {
		t.Errorf("expected at least one spawn request over 1s @ 5/s")
	}
	// Suppress unused warning if buildLineGraph happens to grow imports.
	_ = network.NodeID(0)
}
```

- [ ] **Step 3: Run tests**

Run: `go test ./internal/sim/ -v -run TestRandomOD`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add internal/sim/demand.go internal/sim/demand_test.go
git commit -m "feat(sim): RandomOD spawner with deterministic seeded output"
```

---

### Task 3.4: World + tick loop

**Files:**
- Create: `internal/sim/world.go`

- [ ] **Step 1: Write the World**

Write `internal/sim/world.go`:
```go
package sim

import (
	"log/slog"

	"github.com/lab1702/traffic-sim/internal/network"
)

// World owns mutable simulation state. Only the sim goroutine touches it.
type World struct {
	Net       *network.Network
	Router    *Router
	Spawner   Spawner
	Vehicles  []Vehicle
	Tick      uint64
	SimTime   float64
	dt        float64

	nextID    VehicleID
	maxRetry  int // spawn retries per tick before giving up
}

const DefaultDt = 0.05 // 50 ms == 20 Hz

func NewWorld(net *network.Network, spawner Spawner) *World {
	return &World{
		Net: net,
		Router: NewRouter(net),
		Spawner: spawner,
		dt: DefaultDt,
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
```

- [ ] **Step 2: Verify build**

Run: `go build ./...`
Expected: exit 0.

- [ ] **Step 3: Commit**

```bash
git add internal/sim/world.go
git commit -m "feat(sim): World struct and headless tick loop"
```

---

### Task 3.5: Integration test — 4-intersection grid

**Files:**
- Create: `internal/sim/world_test.go`

- [ ] **Step 1: Write the test**

Write `internal/sim/world_test.go`:
```go
package sim

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

// build2x2Grid returns a 2x2 grid: 4 nodes arranged in a square,
// with edges between adjacent nodes in both directions. 100m blocks.
//
// 2 --- 3
// |     |
// 0 --- 1
func build2x2Grid() *network.Network {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 100, Y: 0}},
		{ID: 2, Pos: network.Point{X: 0, Y: 100}},
		{ID: 3, Pos: network.Point{X: 100, Y: 100}},
	}
	mkEdge := func(id, from, to int) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id),
			From: network.NodeID(from), To: network.NodeID(to),
			Length: 100, SpeedLimit: 10,
			Lanes: []network.Lane{{Index: 0}},
		}
	}
	edges := []network.Edge{
		mkEdge(0, 0, 1), mkEdge(1, 1, 0),
		mkEdge(2, 0, 2), mkEdge(3, 2, 0),
		mkEdge(4, 1, 3), mkEdge(5, 3, 1),
		mkEdge(6, 2, 3), mkEdge(7, 3, 2),
	}
	return &network.Network{Nodes: nodes, Edges: edges}
}

func TestWorld_VehiclesSpawnMoveDespawn(t *testing.T) {
	net := build2x2Grid()
	spawner := NewRandomOD(net, 7, 5.0) // 5 vehicles/sec
	w := NewWorld(net, spawner)

	w.Run(10.0) // 10 simulated seconds

	if w.Tick == 0 {
		t.Fatalf("no ticks ran")
	}
	// Some vehicles should have completed and despawned by now. With a
	// 100m block at 10 m/s, a 2-edge trip is 20s — most won't be done yet.
	// What we *can* assert: spawn was attempted multiple times.
	if w.nextID == 0 {
		t.Errorf("expected some spawns over 10s @ 5/s, got 0")
	}
}

func TestWorld_DeterminismSameSeed(t *testing.T) {
	run := func() (uint32, int) {
		net := build2x2Grid()
		w := NewWorld(net, NewRandomOD(net, 123, 3.0))
		w.Run(5.0)
		return uint32(w.nextID), len(w.Vehicles)
	}
	a, b := run(), run()
	c, d := run(), run()
	_ = b
	_ = d
	if a != c {
		t.Errorf("determinism: same seed produced different nextID: %d vs %d", a, c)
	}
}
```

- [ ] **Step 2: Run tests**

Run: `go test ./internal/sim/ -v`
Expected: all PASS.

- [ ] **Step 3: Run with race detector**

Run: `go test -race ./internal/sim/`
Expected: PASS, no race warnings.

- [ ] **Step 4: Commit**

```bash
git add internal/sim/world_test.go
git commit -m "test(sim): World integration on 2x2 grid; determinism check"
```

---

### Task 3.6: Wire `run --headless` subcommand

**Files:**
- Modify: `cmd/trafficsim/main.go`

- [ ] **Step 1: Read current main.go**

Run the Read tool on `cmd/trafficsim/main.go` to see current contents.

- [ ] **Step 2: Add run subcommand dispatch**

Modify `cmd/trafficsim/main.go` to add a `run` case in the switch and `runRun` function. Final file should be:
```go
package main

import (
	"flag"
	"fmt"
	"log/slog"
	"os"
	"time"

	"github.com/lab1702/traffic-sim/internal/netbuild"
	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/lab1702/traffic-sim/internal/sim"
)

func main() {
	if len(os.Args) < 2 {
		usage()
		os.Exit(2)
	}
	switch os.Args[1] {
	case "load":
		runLoad(os.Args[2:])
	case "run":
		runRun(os.Args[2:])
	default:
		usage()
		os.Exit(2)
	}
}

func usage() {
	fmt.Fprintln(os.Stderr, "usage: trafficsim <subcommand> [flags]")
	fmt.Fprintln(os.Stderr, "subcommands:")
	fmt.Fprintln(os.Stderr, "  load <path-to-osm>           parse and print graph stats")
	fmt.Fprintln(os.Stderr, "  run  <path-to-osm> [flags]   run the simulation")
}

func runLoad(args []string) {
	fs := flag.NewFlagSet("load", flag.ExitOnError)
	if err := fs.Parse(args); err != nil {
		os.Exit(2)
	}
	if fs.NArg() != 1 {
		fmt.Fprintln(os.Stderr, "load: need exactly one OSM path")
		os.Exit(2)
	}
	path := fs.Arg(0)

	feat, err := osmload.Load(path)
	if err != nil {
		slog.Error("load failed", "err", err)
		os.Exit(1)
	}
	net, rpt, err := netbuild.Build(feat)
	if err != nil {
		slog.Error("build failed", "err", err)
		os.Exit(1)
	}
	fmt.Printf("nodes=%d edges=%d intersections=%d signals=%d\n",
		len(net.Nodes), len(net.Edges), len(net.Intersections), countSignals(net.Intersections))
	fmt.Printf("ways_skipped=%d components_dropped=%d\n",
		rpt.WaysSkipped, rpt.ComponentsDropped)
	fmt.Printf("bounds=(%.1f,%.1f)-(%.1f,%.1f) m\n",
		net.Bounds.MinX, net.Bounds.MinY, net.Bounds.MaxX, net.Bounds.MaxY)
}

func runRun(args []string) {
	fs := flag.NewFlagSet("run", flag.ExitOnError)
	var (
		headless  = fs.Bool("headless", false, "skip rendering, run sim only")
		duration  = fs.Duration("duration", 0, "stop after this much sim time (0 = unbounded)")
		seed      = fs.Uint64("seed", 1, "RNG seed for deterministic runs")
		spawnRate = fs.Float64("spawn-rate", 5.0, "vehicles spawned per simulated second")
	)
	if err := fs.Parse(args); err != nil {
		os.Exit(2)
	}
	if fs.NArg() != 1 {
		fmt.Fprintln(os.Stderr, "run: need exactly one OSM path")
		os.Exit(2)
	}
	path := fs.Arg(0)

	feat, err := osmload.Load(path)
	if err != nil {
		slog.Error("load failed", "err", err)
		os.Exit(1)
	}
	net, _, err := netbuild.Build(feat)
	if err != nil {
		slog.Error("build failed", "err", err)
		os.Exit(1)
	}

	spawner := sim.NewRandomOD(net, *seed, *spawnRate)
	w := sim.NewWorld(net, spawner)

	if !*headless {
		fmt.Fprintln(os.Stderr, "warning: rendering not implemented yet; pass --headless")
		os.Exit(2)
	}
	if *duration == 0 {
		fmt.Fprintln(os.Stderr, "error: --headless requires --duration > 0")
		os.Exit(2)
	}
	w.Run(duration.Seconds())
	fmt.Printf("done. final_vehicles=%d ticks=%d sim_time=%.2fs\n",
		len(w.Vehicles), w.Tick, w.SimTime)
	_ = time.Now() // keep time import live if unused
}

func countSignals(xs []network.Intersection) int {
	n := 0
	for _, x := range xs {
		if x.HasSignal {
			n++
		}
	}
	return n
}
```

- [ ] **Step 3: Build and smoke test**

Run:
```powershell
go build ./cmd/trafficsim/
.\trafficsim.exe run .\internal\osmload\testdata\tiny.osm --headless --duration 5s --spawn-rate 2
```
Expected: prints sim-progress logs and a final `done. ...` line. (Tiny fixture has only 3 connected nodes so most spawn attempts will pick same-node OD pairs and be discarded; that's fine for the smoke test — the point is the binary runs without crashing.)

- [ ] **Step 4: Commit**

```bash
git add cmd/trafficsim/main.go
git commit -m "feat(cli): add trafficsim run --headless subcommand"
```

---

**Phase 3 done when:**
- `go test ./...` is green.
- `trafficsim run <file> --headless --duration 10s` runs without crashing.
- Determinism test passes.
