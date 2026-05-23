# Realistic Radius-Based Cornering — Design

**Date:** 2026-05-23
**Status:** Approved (brainstorming phase)

## Goal

Make vehicles slow for turns realistically: brake **gently** to a
**geometry-appropriate** speed and reach that speed **at the corner** — not a
panic stop far up a straight road. Eliminate the user-visible symptom of cars
"slowing down where there is no visible curve."

## Background — why the current cornering looks wrong

The current corner cap lives in `internal/sim/cornering.go`
(`computeDesiredSpeed` → `cornerSpeedCap` / `shouldApplyCornerCap`). Three
properties combine to produce the symptom:

1. **It is angle-based, measured from single OSM end-segments.** `TurnAngle`
   uses only the last segment of the incoming edge and the first segment of the
   outgoing edge (`ArrivalHeading` / `DepartureHeading` in
   `internal/network/geometry.go`). On real OSM geometry those end-segments are
   often short, jagged stubs, so heading is noisy.

2. **It fires on ~1 in 5 "go-straight" movements.** OSM ways are split into
   separate edges at every node shared by ≥2 ways (`splitAtIntersections`), so
   the cap is evaluated at way-boundaries and road elbows mid-block, not only at
   junctions. Measured on real extracts (jackson/oscoda/grasslake/lars2),
   ~19–26% of straightest-continuation movements get capped, and ~21–26% of
   degree-2 pass-throughs (no turn choice exists).

3. **The braking is a panic stop, applied too early.** When the cap engages,
   `v0` drops abruptly to the cap (e.g. 5 m/s for 90°) and IDM's free-flow term
   saturates (`v ≫ v0`) to the `MaxBraking` clamp. Reproduced: a lone car at
   40 km/h into a 90° elbow brakes at **−8.00 m/s²** starting **42 m** before
   the bend and is below 22 km/h while still **14 m** short of it. The
   deceleration zone sits on the straight approach — hence "no visible curve."

Evidence note: only ~1% of capped moves are straight over a 30 m baseline, so
the bends are mostly geometrically real; the dominant defects are (3) the
panic-brake/early-braking and (1) the short-stub artifacts (~2–6% of capped
moves involve a sub-5 m end stub).

## Major decisions (from brainstorming)

- **Keep cornering slowdown**, but make it physically realistic: gentle braking
  to a sane speed reached at the corner.
- **Radius + lateral-acceleration model** for the target speed
  (`v = √(a_lat·R)`), not an angle→speed table. Sweeping bends stay fast, tight
  corners slow, straight roads are unaffected.
- **Scope: junction turns only** (the immediate next route edge), as today. No
  within-road sweeping-curve handling (deferred).
- All-new tuning values are named constants.

## Model

### 1. Turn radius from local geometry

Estimate the radius of the upcoming turn by fitting a circle through three
points sampled along the actual driven path around the junction node
`n = Edges[fromEdge].To = Edges[toEdge].From`:

- `P_before` — walk back `sampleDist` metres along `fromEdge`'s polyline from `n`
- `P_node`   — `n` itself
- `P_after`  — walk forward `sampleDist` metres along `toEdge`'s polyline from `n`

```
R = circumradius(P_before, P_node, P_after)
```

`circumradius(p1,p2,p3) = (|p1p2|·|p2p3|·|p3p1|) / (4·area)` where `area` is the
triangle area from the cross product. When the three points are collinear
(`area ≈ 0`, a straight road) return `+Inf`. The `sampleDist` arms are what
remove the short-stub artifacts: a 1 m jagged end-segment no longer dominates
the heading because the sample point is `sampleDist` back along the real road
direction.

Polyline walk: accumulate segment lengths from the relevant end until the
running total reaches `sampleDist`, returning the point reached. If the edge is
shorter than `sampleDist`, return its far endpoint (use what geometry exists).
Edges with <2 geometry points yield no radius (treat as straight → `+Inf`).

### 2. Safe corner speed

```
v_safe = max(minCornerSpeed, √(a_lat · R))     (v_safe = +Inf when R = +Inf)
```

`a_lat` is comfortable lateral acceleration; `minCornerSpeed` floors hairpins so
they crawl rather than stop. A gentle bend (large `R`) gives `v_safe ≥` the
speed limit, so it has no effect.

### 3. Smooth approach profile (the key fix)

Replace the binary `shouldApplyCornerCap` + hard `v0 = cap` with a
distance-dependent desired speed. With `d` = distance from the front bumper to
the node (`edge.Length − v.S`):

```
v0_corner(d) = √(v_safe² + 2·a_brake·d)
desired      = min(edgeLimit·driverFactor, incidentCap, v0_corner(d))
```

This is the standard kinematic "max speed from which you can still decelerate at
`a_brake` to reach `v_safe` in distance `d`." Far from the node the √ term is
large → no slowdown. As `d → 0` it eases to `v_safe` exactly at the corner.
Because the desired speed stays close to the actual speed, IDM's free-flow term
brakes gently (≈ `a_brake`) and **never reaches the `MaxBraking` clamp** — that
clamp remains only for genuine emergencies (a sudden real leader). When
`v_safe = +Inf`, `v0_corner = +Inf` and the term drops out entirely.

## Architecture & data flow

`computeDesiredSpeed(v)` is the **only** consumer of the cornering logic, called
once per vehicle per tick in `World.Step` (`internal/sim/world.go`) to produce
the `v0` passed to `stepIDM`. All changes are contained to
`internal/sim/cornering.go` and its test. No changes to `idm.go`, `world.go`,
the lane-change logic, the snapshot/renderer, or the trace format.

`computeDesiredSpeed` after the change:

1. `v0 := edge.SpeedLimit · driverFactor` (unchanged).
2. Apply the `Slowdown` incident cap if present (unchanged).
3. If there is no next route edge, return `v0` (unchanged).
4. `R, deflection := turnGeometry(net, v.Edge, nextEdge)` — radius and deflection
   angle from the same three sample points.
5. **Gate:** if `deflection < minCornerAngle` (~40°), return `v0`. Drivers don't
   lift for slight bends; without this gate the radius model slows for
   deflections as gentle as ~21° on a 40 km/h road, which look straight on
   screen. Genuine sharp turns (≥ ~40°) still slow.
6. `v_safe := cornerSpeed(R)`; if `v_safe` is `+Inf` or `≥ v0`, return `v0`.
7. `d := edge.Length − v.S`; return `min(v0, √(v_safe² + 2·a_brake·d))`.

## Components

### `circumradius(p1, p2, p3 network.Point) float64`
Pure geometry helper. Returns `+Inf` for (near-)collinear points.

### `pointBackFromEnd(geom, dist)` / `pointForwardFromStart(geom, dist)`
Polyline-walk helpers returning the `network.Point` reached `dist` metres from
the respective end (clamped to the far endpoint for short edges).

### `turnRadius(net, fromEdge, toEdge) float64`
Samples the three points and returns `circumradius`. `+Inf` when geometry is
insufficient or the path is straight.

### `cornerSpeed(R float64) float64`
`max(minCornerSpeed, √(a_lat·R))`, passing `+Inf` through.

### `computeDesiredSpeed` (rewritten)
As in the data-flow steps above.

### Removed
`cornerSpeedCap` and `shouldApplyCornerCap` (and the `cornerBrakingDecel` /
`cornerReactionBuf` constants they used, replaced by the new constants).
`TurnAngle` / `ClassifyTurn` are retained — lane-choice and post-turn-lane logic
still use them.

## Tuning defaults (named constants)

| Constant         | Value      | Meaning                                      |
|------------------|------------|----------------------------------------------|
| `cornerLatAccel` | 3.0 m/s²   | comfortable lateral acceleration (`a_lat`)   |
| `cornerBrakeDecel` | 1.0 m/s² | planning deceleration for the approach (`a_brake`) |
| `cornerSampleDist` | 15.0 m   | radius sampling arm length                    |
| `minCornerSpeed` | 2.5 m/s    | floor (~9 km/h) for hairpins                  |
| `minCornerAngle` | 40°        | gate — bends gentler than this keep full speed |

`cornerBrakeDecel` is intentionally low. IDM's leaderless (free-flow) term
brakes weakly — it only produces strong deceleration when `v ≫ v0`, which
saturates toward the panic clamp. A larger `a_brake` drops the kinematic
profile late and steeply, so IDM both spikes (hard catch-up) *and* fails to
reach the corner speed in time. `1.0` makes the profile ramp earlier and
gently; the vehicle eases down at ~1.5–1.6 m/s² peak and arrives at the corner
speed. Verified by `TestWorld_CornerBrakingIsGentle` (peak > −4 m/s²) and
`TestWorld_BrakesForSharpTurn` (reaches the corner speed). Because the model is
radius-based, this gentle anticipation only happens at genuine sharp corners —
straight roads and gentle bends get no slowdown at all.

Resulting feel on a 40 km/h (11.2 m/s) road: only turns with effective radius
< ~42 m slow at all; tight 90° (R≈11 m) → ~20 km/h; open turn (R≈20 m) →
~28 km/h; sweeping (R≥42 m) → no slowdown. (`a_brake` shapes only the approach
ramp, not these final corner speeds, which come from `cornerLatAccel`.)

## Testing

### New / replacement unit tests — `internal/sim/cornering_test.go`
- `circumradius`: collinear points → `+Inf`; right isoceles triangle → known R
  (within tolerance).
- `turnRadius`: straight two-edge path → `+Inf`; 90° elbow → expected R for
  `cornerSampleDist`; a jagged-but-straight road (short angled end stub, real
  direction straight over 15 m) → large R / no cap **(artifact regression)**.
- `cornerSpeed`: `+Inf` in → `+Inf` out; tiny R → floored at `minCornerSpeed`.

### Behavior tests — `internal/sim`
- Keep `TestWorld_DoesNotBrakeForStraight` (must still pass: straight → ~limit).
- Update `TestWorld_BrakesForSharpTurn` assertion to the new corner speed
  (~20 km/h band) instead of the old 5 m/s cap.
- New: **gentle peak deceleration** — drive a lone car through a 90° elbow and
  assert min accel stays well above `−MaxBraking` (e.g. `> −3.5 m/s²`)
  **(panic-brake regression)**.
- New: **sweeping vs tight** — a large-radius bend produces little/no slowdown
  while a tight corner on the same speed limit clearly slows.

### Existing tests / suite
- Remove `TestCornerSpeedCap_Anchors` (tests the deleted angle table).
- `go test ./...` green; `go vet ./...` clean.

## Out of scope (deferred)
- Slowing for within-road sweeping curves (curvature along an edge, not just at
  junctions).
- Per-(edge,nextEdge) caching of `v_safe` (geometry is static, so it could be
  precomputed; deferred unless profiling shows the per-tick walk matters).
- Radius constrained by lane/road width.

## Files changed
- `internal/sim/cornering.go` — rewrite.
- `internal/sim/cornering_test.go` — new/updated tests.
