# Turn-Aware Lane Choice — Design

**Date:** 2026-05-15
**Status:** Approved (brainstorming phase)

## Goal

Vehicles approaching an intersection where they need to turn should migrate into a lane from which that turn is legal — right-turning cars to the right, left-turning cars to the left, straight-through cars to a middle lane. Today, lane changes are purely speed-driven (`internal/sim/lanechange.go`), so a vehicle in the leftmost lane will happily attempt a right turn and vice versa.

The constraint is **hard with a fallback**: every turn departs from a legal lane, even if the gradual bias didn't get the vehicle there in time. The fallback is a small lateral teleport at the intersection.

## Major decisions (from brainstorming)

| Axis | Decision |
|---|---|
| Strictness | Hard requirement: every turn departs from a legal lane. Fallback is a lateral snap at the intersection. |
| Mapping source | OSM `turn:lanes=*` when present and parseable; geometric inference via `network.ClassifyTurn` otherwise. |
| Trigger range | Whole approach edge, effectively capped at 300 m (vehicles farther out don't bias yet). |
| Data ownership | Lane→turn mapping pre-computed at netbuild time into the existing `network.Lane.AllowedTurns` field. Network stays immutable. |
| Code placement | Bias logic in `tryLaneChange` (extends existing function). Snap in the existing route-advance block in `vehicle.go`. |

## Architecture

Three pieces along existing module boundaries:

1. **Build-time** (`internal/netbuild`): populate `network.Lane.AllowedTurns` for every edge whose downstream node is a multi-edge intersection. Output is baked into the immutable `Network`.
2. **Runtime bias** (`internal/sim/lanechange.go`): `tryLaneChange` gains a turn-aware branch that runs before the existing speed-driven logic. When the vehicle is within 300 m of an intersection it will turn at, and its current lane can't reach the next route edge, it biases one lane toward the nearest compatible lane (respecting safety gaps).
3. **Intersection snap** (`internal/sim/vehicle.go`): in the route-advance block (currently lines 71–81), after `RouteIdx++`, set `v.Lane` on the new edge based on the just-completed turn category. This is both the normal post-turn lane carry-over AND the snap fallback — same code path.

Data flows one-way: netbuild writes `AllowedTurns` once; sim reads it on every lane-change call and every intersection crossing. No mutation, no sync.

## Lane assignment (netbuild)

**Plumbing preconditions** (apply before the algorithm below):

- Per-direction lane slices. Today `netbuild.go:140-153` calls `makeLanes(lanesPerDir)` once per segment and assigns the same slice to both forward and reverse edges. Once `AllowedTurns` becomes per-direction state, the two edges need their own slices. Allocate `makeLanes(...)` once per edge instead of once per segment.
- OSM way back-reference. The existing `osmWayOfEdge []osm.WayID` parallel slice (built at netbuild.go:145,152) lets the lane-assignment pass look up the source OSM way per edge for `turn:lanes:forward / turn:lanes:backward / turn:lanes`. No new plumbing required.

For each `(incoming_edge, intersection)` pair where the intersection has ≥ 2 outgoing edges:

**Step 1 — Classify outgoing edges.** For every outgoing edge `e`, compute `network.ClassifyTurn(incoming, e)` → one of `TurnLeft / TurnStraight / TurnRight / TurnUTurn`. Skip U-turns and skip banned turns (consult `Intersection.BannedTurns`).

**Step 2 — Try OSM data first.** Look at the incoming edge's source OSM way for `turn:lanes=*` (or `turn:lanes:forward` / `turn:lanes:backward` on two-way ways). Parse the pipe-delimited per-lane spec like `left|through|through;right`. Each token can list multiple turn types via `;`. If the token count equals `len(Lanes)` for the incoming edge, use it directly: for each lane `i`, set `AllowedTurns` to the outgoing edges whose `ClassifyTurn` result matches a token in slot `i`. Mapping table:

| OSM token | Maps to |
|---|---|
| `left`, `sharp_left`, `slight_left`, `merge_to_left` | `TurnLeft` |
| `right`, `sharp_right`, `slight_right`, `merge_to_right` | `TurnRight` |
| `through`, `none`, empty | `TurnStraight` |
| `reverse` | (drop — U-turns not modeled) |
| anything else | (drop — unknown token) |

If parsing fails or the token count doesn't match the lane count, fall through to Step 3.

**Step 3 — Geometric fallback.** Given the set of turn categories present at this intersection and the lane count `N`:

- If only one category exists: every lane gets all outgoing edges in that category.
- If two categories exist (`{S, R}` or `{S, L}`): assign by side.
  - `{S, R}`, `N = 2`: lane 0 = `{R, S}`, lane 1 = `{S}`.
  - `{S, L}`, `N = 2`: lane 0 = `{S}`, lane 1 = `{L, S}`.
- If three categories exist (`{L, S, R}`):
  - `N = 1`: lane 0 = `{L, S, R}`.
  - `N = 2`: lane 0 = `{R, S}`, lane 1 = `{L, S}`.
  - `N = 3`: lane 0 = `{R}`, lane 1 = `{S}`, lane 2 = `{L}`.
  - `N ≥ 4`: lane 0 = `{R}`, lane N−1 = `{L}`, all middle lanes = `{S}`.
- One-lane edges always get all non-banned outgoing edges.

**Step 4 — Sanity check.** Verify every non-banned outgoing edge is reachable from at least one lane. If some category exists but no lane was assigned to it (shouldn't happen with the rules above, but defensive), add it to the closest-side lane. Guarantees no turn becomes unreachable.

**Output.** `Lane.AllowedTurns []EdgeID` populated for every lane on every edge ending at a multi-way intersection. Edges ending at degree-1 or degree-2 (pass-through) intersections leave `AllowedTurns` empty, which per the existing schema doc means "any outgoing edge" — vehicles on those edges are unconstrained.

## Runtime bias

`tryLaneChange` gains a new branch that runs **before** the existing speed-driven logic.

**Trigger check** (top of function, after the cooldown guard):
1. Upcoming-turn context exists: `v.RouteIdx + 1 < len(v.Route)`.
2. Distance to intersection `dToInt = edge.Length - v.S ≤ 300 m`.
3. `len(edge.Lanes) ≥ 2`.
4. Current lane's `AllowedTurns` is non-empty (empty = "any" = no bias needed).

**Compatibility check.** Let `nextE = v.Route[v.RouteIdx+1]` and `myAllowed = edge.Lanes[v.Lane].AllowedTurns`. The current lane is compatible iff `nextE ∈ myAllowed`.

**Behavior split:**
- **Compatible** → fall through to existing speed-driven LC, with one modification: when the trigger is active, the neighbor-lane loop rejects candidate lanes whose `AllowedTurns` doesn't include `nextE`. A speed-driven pass into an incompatible lane is suppressed.
- **Incompatible** → turn bias runs:
  1. Build the set of compatible lanes: `{i : nextE ∈ edge.Lanes[i].AllowedTurns}`.
  2. Find the closest to `v.Lane` by absolute index distance. Tie-break toward the rightmost (lane 0 side).
  3. Step exactly one lane in that direction (`dl = ±1`). Multi-lane jumps happen over multiple ticks.
  4. Apply existing front/rear safety-gap checks (`safetyGapFront = 20 m`, `safetyGapRear = 10 m`). If they pass, commit and start cooldown (`laneChangeCooldown = 3 s`). If blocked, do nothing this tick — retry next tick.
  5. **Skip** the speed-difference threshold (`vDiffThreshold = 5 m/s`) and the `laneChangeCheckGap = 50 m` early-out. Turn bias has a reason to change regardless of leader speed.

Safety gaps are never overridden by turn bias. If they stay blocked for the full 300 m approach, the intersection-snap fallback handles the rest.

## Intersection snap & lane carry-over

In `internal/sim/vehicle.go`, in the route-advance block (currently lines 71–81), after `v.Edge = v.Route[v.RouteIdx]`:

Classify the turn just taken via `network.ClassifyTurn(prevEdge, newEdge)` and set `v.Lane`:

| Turn category | New `v.Lane` |
|---|---|
| `TurnRight` | `0` (rightmost lane of `newEdge`) |
| `TurnLeft` | `len(newEdge.Lanes) - 1` (leftmost) |
| `TurnStraight` | `min(prev_v.Lane, len(newEdge.Lanes) - 1)` (preserve, clamp) |
| `TurnUTurn` | `0` (defensive; routes shouldn't include U-turns) |

This unifies the snap and the normal case: whether bias succeeded or failed, the vehicle ends up on the correct lane of `newEdge`. The teleport when bias failed is at most ~14 m of lateral motion on a 4-lane road — visible but acceptable per the chosen strictness.

**Diagnostics.** When `v.Lane` on the previous edge was *not* in `prevEdge.Lanes[v.Lane].AllowedTurns` for `newEdge` — i.e., bias failed and the snap was required — emit a `slog.Warn` with vehicle state. Format consistent with the existing stuck-vehicle WARN. Observational only; doesn't alter behavior.

## State additions

No new fields on `Vehicle`. The lane index already exists (`v.Lane uint8`) and the route already exists (`v.Route`). All new data lives in `network.Lane.AllowedTurns`, which is already declared in `internal/network/types.go:27`.

## Testing

**Layer 1 — Lane assignment unit tests (`internal/netbuild/lanes_test.go`, new file).**
- `turn:lanes=*` parsing: valid tokens, mixed tokens (`through;right`), malformed input, unknown tokens.
- Geometric fallback for representative intersection shapes:
  - 4-way, 1-lane incoming → lane 0 gets all of `{L, S, R}`.
  - 4-way, 2-lane incoming → lane 0 = `{R, S}`, lane 1 = `{L, S}`.
  - 4-way, 3-lane incoming → lane 0 = `{R}`, lane 1 = `{S}`, lane 2 = `{L}`.
  - T-intersection with only `{L, S}` or `{S, R}`.
  - Intersection with a banned turn — banned edges must not appear anywhere.
- Step-4 invariant: every non-banned outgoing edge appears in at least one lane's `AllowedTurns`.

**Layer 2 — Turn bias in `tryLaneChange` (`internal/sim/lanechange_test.go`, extended).**
- 2-lane edge, vehicle in lane 0, must turn left → migrates to lane 1 within trigger range when gaps are clear.
- Same setup with a blocker inside the safety gap on lane 1 → does NOT change lanes this tick; retries.
- Vehicle already in a compatible lane → bias does nothing; speed-driven LC still works but rejects neighbor lanes that would become incompatible.
- Vehicle > 300 m from intersection → bias does NOT fire; existing speed-driven behavior unchanged.
- Last edge of route → bias does NOT fire.

**Layer 3 — Intersection snap & lane carry-over (extend `internal/sim/world_test.go` or new file).**
- After `TurnRight`: `v.Lane == 0` on new edge regardless of previous lane.
- After `TurnLeft`: `v.Lane == N-1` on new edge.
- After `TurnStraight`: `v.Lane` preserved, clamped to `N-1` when new edge has fewer lanes.
- Snap-fallback diagnostic: previous `v.Lane` incompatible with the just-taken turn → WARN emitted (capture via the existing logger writer pattern).

**Layer 4 — End-to-end on the existing OSM fixture (`internal/e2e/e2e_test.go`).**
- Run the existing scenario for some number of ticks. Assert the total snap-fallback warning count stays below a small threshold relative to vehicles spawned (e.g., < 10% of route turns) — bias must succeed in the vast majority of cases. Catches tuning regressions.

**Manual verification.** Visual lateral migration is checked by running `trafficsim.exe` and watching a multi-lane intersection.

## Out of scope

- Lane-direction conventions other than right-hand driving.
- Dedicated turn pockets (additional lanes appearing only near intersections).
- Lane-changing across U-turn maneuvers (not modeled).
- Tagged turn restrictions beyond the existing `Intersection.BannedTurns`.
- Recording lane history in the trace format (replay continues to default vehicles to lane 0).
