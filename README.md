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

Tick budget is 50 ms at 20 Hz. Current per-tick benchmarks on a 40x40 grid
(1,600 intersections, ~6,240 directed edges), measured on Intel Core Ultra 9 285K:

| Vehicles | ns/op     | ms/tick |
|----------|-----------|---------|
| 1,000    | 460,402   | 0.46    |
| 5,000    | 1,966,688 | 1.97    |
| 10,000   | 3,023,438 | 3.02    |

All three are well under the 50 ms budget, leaving headroom for real-world
network complexity and additional features.

Run benchmarks yourself:
```
go test ./internal/sim/ -bench=. -benchtime=2s -run=^$
```

## E2E testing

To run the full pipeline against a real OSM extract:
```
TRAFFIC_SIM_E2E_OSM=path/to/extract.osm.pbf go test -tags e2e ./internal/e2e/
```

Recommended small extract: a single neighborhood from https://extract.bbbike.org (5–20 MB .osm.pbf).

## Design + plan

- Spec: `docs/superpowers/specs/2026-05-15-traffic-sim-design.md`
- Plan: `docs/superpowers/plans/2026-05-15-traffic-sim.md`
