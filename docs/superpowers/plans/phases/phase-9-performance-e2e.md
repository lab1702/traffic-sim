# Phase 9 — Performance + E2E

**Milestone:** Benchmarks at 1k/5k/10k vehicles report per-tick cost. A small real OSM extract loads, runs headless for 60 sim-seconds, and emits a well-formed trace. CI runs the suite under `-race`.

---

### Task 9.1: Per-tick benchmark

**Files:**
- Create: `internal/sim/bench_test.go`

- [ ] **Step 1: Write benchmark**

Write `internal/sim/bench_test.go`:
```go
package sim

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

// buildGridN builds an N x N grid of intersections with 100m blocks.
// Resulting graph has N*N nodes, ~4*N*(N-1) directed edges.
func buildGridN(n int) *network.Network {
	idx := func(i, j int) network.NodeID { return network.NodeID(i*n + j) }
	var nodes []network.Node
	for i := 0; i < n; i++ {
		for j := 0; j < n; j++ {
			nodes = append(nodes, network.Node{
				ID:  idx(i, j),
				Pos: network.Point{X: float64(j) * 100, Y: float64(i) * 100},
			})
		}
	}
	var edges []network.Edge
	mkEdge := func(from, to network.NodeID) {
		fromPos := nodes[from].Pos
		toPos := nodes[to].Pos
		edges = append(edges, network.Edge{
			ID: network.EdgeID(len(edges)), From: from, To: to, Length: 100,
			SpeedLimit: 13.4,
			Lanes:      []network.Lane{{Index: 0}, {Index: 1}},
			Geometry:   []network.Point{fromPos, toPos},
		})
	}
	for i := 0; i < n; i++ {
		for j := 0; j < n; j++ {
			if j+1 < n {
				mkEdge(idx(i, j), idx(i, j+1))
				mkEdge(idx(i, j+1), idx(i, j))
			}
			if i+1 < n {
				mkEdge(idx(i, j), idx(i+1, j))
				mkEdge(idx(i+1, j), idx(i, j))
			}
		}
	}
	return &network.Network{
		Nodes: nodes, Edges: edges,
		Bounds: network.BoundingBox{
			MinX: 0, MinY: 0, MaxX: float64(n) * 100, MaxY: float64(n) * 100,
		},
	}
}

func benchmarkTickN(b *testing.B, vehicleCount int) {
	net := buildGridN(40) // 1600 intersections
	w := NewWorld(net, NewRandomOD(net, 1, 0), nil)
	// Pre-seed N vehicles by issuing spawn requests until we have enough.
	for len(w.Vehicles) < vehicleCount {
		w.trySpawn(SpawnRequest{
			OriginNode: network.NodeID(len(w.Vehicles) % len(net.Nodes)),
			DestNode:   network.NodeID((len(w.Vehicles)*7 + 3) % len(net.Nodes)),
		})
	}
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		w.Step()
	}
}

func BenchmarkTick_1k(b *testing.B)  { benchmarkTickN(b, 1000) }
func BenchmarkTick_5k(b *testing.B)  { benchmarkTickN(b, 5000) }
func BenchmarkTick_10k(b *testing.B) { benchmarkTickN(b, 10000) }
```

- [ ] **Step 2: Run benchmarks**

Run: `go test ./internal/sim/ -bench=. -benchtime=2s -run=^$`
Expected: prints `BenchmarkTick_1k`, `_5k`, `_10k` with `ns/op` values. Note them for the README.

Tick budget at 20 Hz is 50 ms = 50,000,000 ns. The 10k benchmark `ns/op` should be under that for the sim to keep up in real time. If it's not, that's a finding — see Task 9.2.

- [ ] **Step 3: Commit**

```bash
git add internal/sim/bench_test.go
git commit -m "test(sim): per-tick benchmarks at 1k/5k/10k vehicles"
```

---

### Task 9.2: Optimize hot path if needed

If 10k benchmark is over budget (≥ 50ms/tick), investigate. Likely first suspects:
1. **Map allocations per tick** in `byEdge` / `byEdgeLane` — switch to reusable slices keyed by edge index.
2. **Insertion sort over large per-edge slices** — switch to `sort.Slice` once N per edge > ~16.
3. **Allocations in `publishSnapshot`** — preallocate buffers, reuse them.

If it's already under budget, skip optimization and document the result in the README.

- [ ] **Step 1: Profile if over budget**

Run: `go test ./internal/sim/ -bench=BenchmarkTick_10k -benchtime=2s -run=^$ -cpuprofile cpu.prof`
Then: `go tool pprof -top cpu.prof | head -20`
Expected: top functions in the sim. Apply targeted fixes only to functions in the top 5.

- [ ] **Step 2: Re-benchmark and commit any optimizations**

If you made changes:
```bash
go test ./internal/sim/ -bench=BenchmarkTick_10k -benchtime=2s -run=^$
git add internal/sim/
git commit -m "perf(sim): <specific optimization>"
```

If no changes needed:
- Update README with measured numbers, commit that instead.

---

### Task 9.3: Small real-OSM E2E test (build-tag gated)

**Files:**
- Create: `internal/e2e/e2e_test.go`
- Create: `internal/e2e/testdata/.gitkeep` (or actual fixture if licensing allows)

- [ ] **Step 1: Write the test**

Write `internal/e2e/e2e_test.go`:
```go
//go:build e2e

// Package e2e runs the full pipeline against a small real OSM extract.
// Gate this behind the `e2e` build tag because it requires a fixture file
// not bundled in the repo; download instructions are in this directory's
// .gitkeep.
package e2e

import (
	"bytes"
	"os"
	"testing"

	"github.com/lab1702/traffic-sim/internal/netbuild"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/lab1702/traffic-sim/internal/sim"
	"github.com/lab1702/traffic-sim/internal/trace"
)

func TestE2E_RealOSM_HeadlessRun(t *testing.T) {
	path := os.Getenv("TRAFFIC_SIM_E2E_OSM")
	if path == "" {
		t.Skip("TRAFFIC_SIM_E2E_OSM not set; skipping E2E")
	}
	feat, err := osmload.Load(path)
	if err != nil { t.Fatal(err) }
	net, rpt, err := netbuild.Build(feat)
	if err != nil { t.Fatal(err) }
	if len(net.Nodes) < 100 {
		t.Fatalf("expected non-trivial graph, got %d nodes", len(net.Nodes))
	}
	t.Logf("graph: nodes=%d edges=%d intersections=%d ways_skipped=%d components_dropped=%d",
		len(net.Nodes), len(net.Edges), len(net.Intersections),
		rpt.WaysSkipped, rpt.ComponentsDropped)

	spawner := sim.NewRandomOD(net, 42, 10.0)
	w := sim.NewWorld(net, spawner, nil)
	var buf bytes.Buffer
	tw := trace.NewWriter(&buf)
	w.EmitTrace = func(tick uint64, simTime float64, e trace.Event) {
		_ = tw.Write(tick, simTime, e)
	}
	w.Run(60.0) // 60 sim-seconds

	if w.Tick == 0 {
		t.Fatalf("no ticks executed")
	}
	if buf.Len() == 0 {
		t.Errorf("no trace bytes written")
	}
	// Read back, count spawn/despawn events.
	r := trace.NewReader(&buf)
	spawns, despawns := 0, 0
	for {
		_, ev, err := r.Next()
		if err != nil { break }
		switch ev.(type) {
		case *trace.VehicleSpawn:   spawns++
		case *trace.VehicleDespawn: despawns++
		}
	}
	if spawns == 0 {
		t.Errorf("no spawns recorded")
	}
	t.Logf("trace: %d spawns, %d despawns", spawns, despawns)
}
```

- [ ] **Step 2: Write the README for testdata**

Write `internal/e2e/testdata/README.md`:
```markdown
# E2E test data

Run E2E tests with:

```
TRAFFIC_SIM_E2E_OSM=path/to/extract.osm.pbf go test -tags e2e ./internal/e2e/
```

Recommended small extract: a single neighborhood from
https://extract.bbbike.org (5-20 MB .osm.pbf).
```

- [ ] **Step 3: Verify build tag works**

Run: `go build ./...`
Expected: succeeds (e2e file is excluded without the tag).

Run: `go test -tags e2e ./internal/e2e/`
Expected: SKIP because `TRAFFIC_SIM_E2E_OSM` is not set (test is wired correctly).

- [ ] **Step 4: Commit**

```bash
git add internal/e2e/
git commit -m "test(e2e): build-tag-gated test against real OSM extract"
```

---

### Task 9.4: README finalization

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Replace README with full usage doc**

Write `README.md`:
```markdown
# traffic-sim

Go-based traffic simulator that reads OpenStreetMap files and runs a
city-scale microsimulation. Vehicles use the Intelligent Driver Model
(IDM) for car-following, change lanes when beneficial, and obey
auto-generated fixed-time traffic signals (overridable via YAML).

Output: a live Ebitengine viewer (pan/zoom, vehicles, signals, HUD)
and an optional binary trace file for replay or offline analysis.

## Build

```
go build ./cmd/trafficsim/
go build ./cmd/tracereplay/
```

## Run

Load and inspect a graph:
```
trafficsim load extract.osm.pbf
```

Run with the viewer:
```
trafficsim run extract.osm.pbf --spawn-rate 20 --trace run.trace
```

Run headless (research mode):
```
trafficsim run extract.osm.pbf --headless --duration 5m --spawn-rate 20 --trace run.trace
```

Replay a trace:
```
tracereplay -osm extract.osm.pbf -trace run.trace
```

Signal overrides (per intersection):
```
trafficsim run extract.osm.pbf --signals configs/signals.example.yaml
```

## Determinism

Same `--seed` + same OSM + same `--spawn-rate` → byte-identical trace.
This is verified by `TestWorld_TraceDeterminism`.

## Performance

Tick budget is 50 ms at 20 Hz. Current per-tick benchmarks on a 40x40 grid:

| Vehicles | Per-tick |
|----------|----------|
| 1,000    | (fill in from `go test -bench=BenchmarkTick_1k`) |
| 5,000    | (fill in) |
| 10,000   | (fill in) |

## Design + plan

- Spec: `docs/superpowers/specs/2026-05-15-traffic-sim-design.md`
- Plan: `docs/superpowers/plans/2026-05-15-traffic-sim.md`
```

- [ ] **Step 2: Fill in the benchmark numbers**

Run: `go test ./internal/sim/ -bench=. -benchtime=2s -run=^$ 2>&1 | tee /tmp/bench.txt`
Then update the README table with the actual numbers.

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "docs: README with usage, determinism note, and benchmark results"
```

---

**Phase 9 done when:**
- All benchmarks run and numbers are recorded.
- E2E test is gated behind `-tags e2e` and skipped without the env var.
- README documents build/run/replay/overrides/determinism.
- `go test ./...` is green; `go test -race ./...` is green.
