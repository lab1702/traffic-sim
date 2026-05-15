# Traffic Simulator — Design

**Date:** 2026-05-15
**Status:** Approved (brainstorming phase)

## Goal

Build a Go-based traffic simulator that reads street layouts from OpenStreetMap (OSM) files and is suitable for both **visualization** and **research**.

Target scale: a full city — tens of thousands of intersections, 10k+ concurrent vehicles.

## Major decisions (from brainstorming)

| Axis | Decision |
|---|---|
| Goals | Visualization + research/analysis |
| Scale | Full city (10k+ vehicles, tens of thousands of intersections) |
| Vehicle model | Full microsimulation: IDM car-following + lane changing + intersection negotiation + traffic signals |
| Output | Live desktop view **and** trace files for replay/analysis |
| Demand model | Random OD pairs initially; demand interface designed so a zone-based OD matrix can be plugged in later |
| OSM input | Both `.osm` XML and `.osm.pbf`, via `paulmach/osm` |
| Traffic signals | Auto-generated fixed-time defaults at signalized intersections + optional per-intersection config overrides |
| Architecture | Single binary; sim and renderer decoupled internally (sim on its own goroutine at fixed timestep, renderer reads double-buffered snapshots) |

## High-level architecture

```
                +------------------------+
                |    OSM file (.osm /    |
                |       .osm.pbf)        |
                +-----------+------------+
                            |
                            v
                +------------------------+
                |   OSM Loader & Graph   |  one-time, on startup
                |        Builder         |
                +-----------+------------+
                            |
                  network graph (nodes,
                  edges, lanes, signals)
                            |
                            v
            +-----------------------------------+
            |          Simulation Core          |
            |  (own goroutine, fixed 20 Hz tick)|
            |                                   |
            |  - Vehicle agents (IDM + lane     |
            |    change + intersection nego.)   |
            |  - Signal controllers             |
            |  - Demand generator (spawner)     |
            |  - Spatial index (grid)           |
            +---------+-------------+-----------+
                      |             |
            snapshot  |             |  event stream
            (double-  |             |
            buffered) |             |
                      v             v
            +-----------------+   +------------------+
            |    Renderer     |   |   Trace Writer   |
            |  (Ebitengine,   |   |   (goroutine,    |
            |   60 FPS,       |   |   binary file)   |
            |   pan/zoom)     |   +------------------+
            +-----------------+
```

Three top-level boundaries:
- **OSM Loader / Graph Builder** — runs once at startup, produces an immutable network graph.
- **Sim Core** — owns all mutable simulation state, advances at a fixed 20 Hz timestep on its own goroutine.
- **Renderer + Trace Writer** — consumers of sim state; never mutate it.

The sim is the only writer. The renderer reads from a double-buffered snapshot (sim writes back-buffer, renderer reads front-buffer, swap atomically on tick boundary). The trace writer consumes events from a buffered channel.

A `--headless` flag skips the renderer entirely so research runs go as fast as CPU allows.

## Package layout

```
traffic-sim/
  cmd/
    trafficsim/         # main: live sim + viewer
      main.go
    tracereplay/        # main: replays a trace file in the viewer
      main.go
  internal/
    osmload/            # parse .osm / .osm.pbf -> raw OSM features
    netbuild/           # build the routable network graph from OSM
                        #   (filter highway tags, split ways at
                        #    intersections, infer lanes, find signal nodes)
    network/            # immutable graph types: Node, Edge, Lane, Intersection
                        # + spatial index (uniform grid)
    sim/                # simulation core
      vehicle.go        #   IDM + lane-change state
      intersection.go   #   signal phase + gap-acceptance logic
      signal.go         #   signal controller (auto + override)
      demand.go         #   spawner interface + RandomOD impl
      router.go         #   A* over the graph
      world.go          #   tick loop, snapshot publisher
    snapshot/           # double-buffered render snapshot type
    trace/              # event types + binary writer/reader
    render/             # Ebitengine viewport, pan/zoom, HUD
    config/             # YAML config + signal override loading
  configs/
    signals.example.yaml
  docs/
    superpowers/specs/
```

Two binaries:
- `trafficsim` — live sim with viewer (or `--headless`)
- `tracereplay` — viewer-only, reads a trace file

Two design rules:
- `network` types are immutable after build — sim and renderer can read freely without locks.
- `sim` is the only package that mutates simulation state. Everything else reads via snapshots or events.

## Data types

**Network graph (built once, immutable):**

```go
type Network struct {
    Nodes         []Node
    Edges         []Edge          // directed; one-way streets => 1 edge, two-way => 2
    Intersections []Intersection
    Grid          *SpatialGrid    // edges bucketed by uniform cell
}

type Edge struct {
    ID         EdgeID
    From, To   NodeID
    Lanes      []Lane     // per-lane geometry + allowed turns
    Length     float64    // meters
    SpeedLimit float64    // m/s (from maxspeed tag, defaults by highway type)
    Geometry   []Point    // polyline for rendering
}

type Intersection struct {
    NodeID       NodeID
    Incoming     []EdgeID
    Outgoing     []EdgeID
    HasSignal    bool       // from highway=traffic_signals
    SignalConfig *SignalConfig // nil unless overridden
}
```

**Sim state (mutable, owned by sim goroutine):**

```go
type World struct {
    Net      *Network        // read-only
    Vehicles []Vehicle       // struct-of-fields slice (DOD-friendly)
    Signals  []SignalState   // phase + timer per signalized intersection
    Rng      *rand.Rand
    Tick     uint64
}

type Vehicle struct {
    ID         VehicleID
    Route      []EdgeID
    RouteIdx   int
    Edge       EdgeID
    Lane       uint8
    S          float64   // meters along edge
    V          float64   // m/s
    A          float64   // m/s²
    LaneChange laneChangeState
}
```

**Snapshot (what the renderer sees):**

```go
type Snapshot struct {
    Tick     uint64
    SimTime  float64
    Vehicles []VehicleView // x, y, heading, speed, color
    Signals  []SignalView  // intersectionID + current phase
}
```

The sim builds the back-buffer snapshot on each tick, then atomically swaps it for the renderer to read.

**Trace events (binary stream):**

```
EventHeader { tick uint64, simTime float64, kind uint8, len uint16 }
followed by kind-specific payload:
  - VehicleSpawn  { id, route, type }
  - VehicleDespawn{ id }
  - SignalPhase   { intersectionID, phase, durationS }
  - MetricsTick   { totalVehicles, avgSpeed, congestionIdx } (every N ticks)
```

Vehicle *positions* are NOT in the trace by default — they're reconstructable from spawn + route + sim-deterministic playback. A periodic state snapshot is recorded every ~N seconds so replay can seek without re-simulating from t=0.

## Tick loop (20 Hz, dt = 50 ms)

1. Demand generator may spawn new vehicles.
2. For each vehicle: IDM accel from leader → integrate position → check lane change → check route advancement.
3. For each signal: advance phase timer.
4. For each intersection without signal: resolve gap-acceptance for waiting vehicles.
5. Emit events to trace channel (non-blocking; drops are logged, not silent).
6. Build snapshot back-buffer and swap (skipped in `--headless`).

**Determinism:** sim uses a single seeded RNG; identical seed + same OSM + same demand config → identical trace. This is the contract that makes replay reliable and research repeatable.

**Routing:** A* per vehicle at spawn. For city scale, contraction hierarchies may be added later if A* becomes a bottleneck; start with plain A* and measure.

## Error handling

**OSM parsing & graph build (startup):**
- Malformed/missing file → exit with clear error, non-zero status.
- Ways with unknown `highway=` tags → skip; log count of skipped ways at end.
- Disconnected graph components → keep the largest, log the rest.
- Missing `maxspeed` → fall back to defaults per highway type (motorway 31 m/s, residential 13 m/s, etc.) defined in `netbuild`.
- Missing `lanes` → infer from `highway` type + `oneway` tag.
- Zero-length or degenerate edges → drop.

**Sim runtime:**
- Vehicle reaches end of route → despawn (emit event).
- Routing fails (no path) → discard spawn attempt, retry next tick. Cap retries per tick to avoid spin.
- Vehicle stuck (zero velocity for N ticks beyond signal/queue norms) → treat as a sim bug, log with full state, and despawn so the run continues.
- IDM numerical issues (negative speed from integration step) → clamp to zero.

**Trace writer:**
- Events go through a buffered channel. If the writer falls behind and the buffer fills, the sim drops events and increments a `trace_drop` counter (logged + emitted as a metric). Sim never blocks on the trace writer.
- On clean shutdown (SIGINT), sim emits a final `SimEnd` event and waits up to 2s for the writer to drain.

**Renderer:**
- If sim ticks faster than renderer reads, intermediate snapshots are overwritten — renderer always sees the latest.
- If sim ticks slower than renderer (heavy CPU), renderer interpolates positions between the two most recent snapshots using `v * dt`. Keeps motion smooth even when sim dips below 20 Hz.
- Pan/zoom out of bounds → clamp to network bounding box.

**Config file (signals.yaml):**
- File missing → fine, use auto-generated defaults.
- Override references unknown intersection ID → log warning, skip that override, continue.
- Malformed YAML → fail at startup with line number.

**Principles:**
- Sim is *resilient*: bad input data shouldn't crash a 10-minute run — drop the bad thing, log, continue.
- Sim is *honest*: dropped events, stuck vehicles, disconnected components, and signal-override misses all surface in logs/metrics. Silent degradation is a research-result killer.

## Testing strategy

**Unit tests (fast, deterministic):**
- `osmload` — tiny hand-built `.osm` XML and `.osm.pbf` fixtures; both formats produce identical feature sets.
- `netbuild` — fixture OSM: ways split at intersections, one-way handling, lane inference, signal detection, disconnected-component pruning.
- `network` — spatial-grid lookups (point-in-cell, edges-within-radius).
- `sim/router` — A* on a hand-built 5-node graph: shortest path correctness, unreachable-target handling.
- `sim/vehicle` — IDM: free flow reaches speed limit, with leader maintains safe gap, zero gap clamps to non-negative speed.
- `sim/signal` — phase advancement timing; config override applied correctly.
- `sim/intersection` — gap-acceptance: yields when gap < threshold, proceeds when gap ≥ threshold.
- `trace` — round-trip write → read equality.
- `snapshot` — double-buffer swap is race-free under `-race`.

**Integration tests (small, realistic):**
- 4-intersection grid (hand-built `Network`, no OSM): spawn 20 vehicles with random OD pairs, run 5 sim-minutes headless. Assert all vehicles despawned or made forward progress; no stuck-vehicle events; total distance > 0.
- Same scenario twice with same seed → identical trace bytes (determinism test).
- Same scenario, signal override applied → measurable change in throughput at that intersection.

**End-to-end test (slow, opt-in via build tag):**
- Use a small real OSM extract checked into `testdata/` (license permitting; otherwise downloaded by a script). Load, run headless for 1 sim-minute, assert no crash, vehicles spawn/despawn, trace file is well-formed.

**Replay test:**
- Run sim headless → produces trace file. Open trace with `tracereplay`, advance through all events, assert event count and final vehicle count match sim's own counters.

**Manual / visual checks (not automated):**
- Open a real OSM file in `trafficsim`, pan/zoom, watch vehicles. Spot-check that they follow roads, stop at signals, don't drive through buildings.

**Performance benchmarks (tracked, not pass/fail):**
- `BenchmarkTick_1k`, `_5k`, `_10k` vehicles on a fixed network → tick duration. Goal: 10k vehicles in <50 ms (one tick budget at 20 Hz). Regressions show up in CI output.

**Ground rules:**
- Unit + integration tests must run in under ~10 s total so they run on every save.
- Determinism (same seed → same trace) is load-bearing. If it ever breaks, that's a P0 — replay and reproducibility depend on it.

## Out of scope (explicitly deferred)

- Multi-modal traffic (pedestrians, bikes, transit) — vehicles only for now.
- Zone-based OD matrix demand — interface designed for it; implementation deferred.
- Adaptive (actuated) signal control — fixed-time only for now; the controller interface is the extension point.
- Contraction hierarchies / advanced routing — plain A* until measurements demand otherwise.
- Multiplayer / interactive control of signals or vehicles — observation-only UI for now.
