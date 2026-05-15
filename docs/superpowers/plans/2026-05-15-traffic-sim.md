# Traffic Simulator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Go-based traffic simulator that reads OSM files, simulates city-scale microscopic traffic (IDM + lane changes + signal-controlled intersections), renders live in a desktop window, and writes trace files for replay/analysis.

**Architecture:** Single binary with three internal boundaries: an immutable network graph built once from OSM input, a sim core running on its own goroutine at a fixed 20 Hz timestep, and consumers (renderer + trace writer) that read sim state via a double-buffered snapshot and a buffered event channel. Determinism (same seed + same input → same trace) is a load-bearing property.

**Tech Stack:** Go 1.22+, `github.com/paulmach/osm` (OSM parsing), `github.com/hajimehoshi/ebiten/v2` (rendering), `gopkg.in/yaml.v3` (config), stdlib for everything else.

---

## File Structure

```
traffic-sim/
  go.mod
  cmd/
    trafficsim/main.go      # live sim + viewer binary
    tracereplay/main.go     # replay-only viewer binary
  internal/
    osmload/                # parse .osm / .osm.pbf -> raw features
      osmload.go
      osmload_test.go
    netbuild/               # build routable graph from OSM features
      netbuild.go
      defaults.go           # speed/lane defaults per highway type
      netbuild_test.go
    network/                # immutable graph types + spatial index
      types.go
      grid.go
      grid_test.go
    sim/                    # simulation core
      world.go              # World struct, tick loop
      vehicle.go            # Vehicle + IDM + lane-change
      vehicle_test.go
      router.go             # A*
      router_test.go
      signal.go             # signal controller
      signal_test.go
      intersection.go       # gap-acceptance + signal application
      intersection_test.go
      demand.go             # spawner interface + RandomOD
      demand_test.go
    snapshot/               # double-buffered render snapshot
      snapshot.go
      snapshot_test.go
    trace/                  # binary event format
      events.go
      writer.go
      reader.go
      trace_test.go
    render/                 # Ebitengine viewport, HUD
      viewport.go
      hud.go
    config/                 # YAML config + signal overrides
      config.go
      config_test.go
  configs/
    signals.example.yaml
  testdata/
    tiny.osm                # 4-node hand-built fixture
    fourway.osm             # signalized 4-way intersection fixture
```

## Phases

Each phase ends with a working milestone you can run. Each phase is a separate file with the full bite-sized task breakdown.

| Phase | File | Milestone |
|---|---|---|
| 1 | [phase-1-foundations.md](phases/phase-1-foundations.md) | `go test ./...` green for network + grid |
| 2 | [phase-2-osm-to-graph.md](phases/phase-2-osm-to-graph.md) | `trafficsim load <file>` prints graph stats |
| 3 | [phase-3-sim-skeleton.md](phases/phase-3-sim-skeleton.md) | Headless sim runs constant-velocity vehicles |
| 4 | [phase-4-idm.md](phases/phase-4-idm.md) | IDM car-following with leader lookahead |
| 5 | [phase-5-signals-intersections.md](phases/phase-5-signals-intersections.md) | Stop at red, yield at unsignalized, YAML overrides |
| 6 | [phase-6-lane-changing.md](phases/phase-6-lane-changing.md) | Threshold-based overtaking on multi-lane edges |
| 7 | [phase-7-snapshot-renderer.md](phases/phase-7-snapshot-renderer.md) | Live Ebitengine viewer with pan/zoom + HUD |
| 8 | [phase-8-trace-replay.md](phases/phase-8-trace-replay.md) | Binary trace + `tracereplay` + determinism test |
| 9 | [phase-9-performance-e2e.md](phases/phase-9-performance-e2e.md) | Benchmarks at 1k/5k/10k + small real-OSM E2E |

## Execution order

Work the phases in order. Each phase ends with a working binary or green test suite, so it's safe to pause between phases for review.

Within a phase, tasks are also ordered — early tasks set up types used by later tasks. Steps within a task are bite-sized (write test → run failing → implement → run passing → commit). Subagent-driven execution should treat each *task* as a unit of work (one fresh subagent per task).

## Determinism is a P0 invariant

Phase 8's `TestWorld_TraceDeterminism` asserts byte-identical trace output for the same seed. **If this test ever fails, stop and find the source of nondeterminism before continuing.** Replay, reproducible research runs, and CI all rely on it. Common offenders to check first: map iteration order, time-based randomness, goroutine scheduling racing with sim state.

## Known follow-ups (out of scope for v1)

These are spec items I intentionally deferred. Add them as small follow-up tasks once the v1 plan is complete:

1. **Stuck-vehicle despawn.** If a vehicle has `V < 0.1 m/s` for >60 sim-seconds *and* `stopDistanceForRed`/`stopDistanceForYield` both return false, log full state and despawn. Defensive against unforeseen sim bugs in long research runs.
2. **Snapshot interpolation in the renderer.** At 60 FPS reading a 20 Hz snapshot, motion appears in 50 ms steps (~3 frames). Add linear interpolation by storing two recent snapshots and lerping by `wallTime since last publish / dt`. Visual polish only — doesn't affect correctness.
3. **Periodic vehicle state snapshots in the trace.** Phase 8 reconstructs positions from spawn + route + sim-deterministic playback; this works but is fragile if replay logic drifts from sim logic. Recording a full state snapshot every ~5 sim-seconds would make replay robust against future sim changes.
