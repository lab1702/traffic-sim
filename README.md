# traffic-sim

![traffic-sim screenshot](trafficsim.png)

Go-based traffic simulator that reads OpenStreetMap files and runs a
city-scale microsimulation. Vehicles use the Intelligent Driver Model
(IDM) for car-following, change lanes when beneficial, and obey
auto-generated fixed-time traffic signals (overridable via YAML).

Output: a live Ebitengine viewer (pan/zoom, vehicles, signals, HUD)
and an optional binary trace file for replay or offline analysis.

## Install

Recommended — install the binaries directly from GitHub with `go install`:

```
go install github.com/lab1702/traffic-sim/cmd/trafficsim@latest
go install github.com/lab1702/traffic-sim/cmd/tracereplay@latest
```

This drops `trafficsim` and `tracereplay` into `$(go env GOBIN)` (or
`$(go env GOPATH)/bin` if `GOBIN` is unset). Add that directory to your
`PATH` to invoke them by name.

Pin a specific version by replacing `@latest` with a tag (e.g. `@v0.1.0`)
or commit SHA.

## Build from source

If you've cloned the repo and want to build locally:

```
go build ./cmd/trafficsim/
go build ./cmd/tracereplay/
```

This drops `trafficsim` and `tracereplay` (`.exe` on Windows) into the
current directory. Either prefix with `./` to run them, or `go install
./...` to put them on `PATH`. Examples below assume one of those.

## Run

Load and inspect a graph:
```
./trafficsim load extract.osm.pbf
```

Run with the viewer:
```
./trafficsim run --spawn-rate 20 --trace run.trace extract.osm.pbf
```

> Flags must appear BEFORE the OSM path — the Go flag parser stops at
> the first non-flag argument.

Run headless (research mode):
```
./trafficsim run --headless --duration 5m --spawn-rate 20 --trace run.trace extract.osm.pbf
```

> Ctrl+C (SIGINT/SIGTERM) triggers an orderly shutdown: the trace is
> flushed with a final `SimEnd` event before the process exits.

Replay a trace:
```
./tracereplay -osm extract.osm.pbf -trace run.trace
./tracereplay -speed 4 -osm extract.osm.pbf -trace run.trace   # 4x playback
```

Signal overrides (per intersection):
```
./trafficsim run --signals configs/signals.example.yaml extract.osm.pbf
```

The YAML schema is documented inline in `configs/signals.example.yaml`.
A missing or invalid signals file causes `trafficsim` to exit with a
clear error rather than silently producing zero overrides.

### GPS rerouting

By default every vehicle has GPS and re-routes around congestion. Each edge
tracks the smoothed average speed of the vehicles on it, which feeds a
travel-time routing cost; when a vehicle enters a new edge it re-evaluates the
remaining path to its destination and switches to a meaningfully faster route
if one exists. Tune the share of GPS-equipped vehicles with `--gps-share`
(0..1, default 1.0):

```
./trafficsim run --gps-share 0.5 --spawn-rate 20 extract.osm.pbf   # half the fleet
./trafficsim run --gps-share 0 --spawn-rate 20 extract.osm.pbf     # static routing
```

Reroutes are recorded in the trace as `VehicleReroute` events, so `tracereplay`
follows the path each vehicle actually took.

### Notes for Windows

- Built binaries end in `.exe`. The commands above work in PowerShell
  exactly as shown (`./trafficsim.exe` is also accepted).
- `go install ./...` places binaries in `%USERPROFILE%\go\bin` by default.
- Paths with spaces must be double-quoted.

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

> These figures predate GPS rerouting (now on by default), which adds a
> per-tick congestion update and periodic per-vehicle A* reroutes. With it
> enabled, live per-tick cost is higher and more workload-dependent (it scales
> with congestion, not just vehicle count), but still comfortably within the
> 50 ms budget. Re-run the benchmark for current numbers, or use `--gps-share 0`
> to measure static routing.

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
