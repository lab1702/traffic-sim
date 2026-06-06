# Roundabouts (Multi-Lane) — Design

**Date:** 2026-06-05
**Status:** Approved (brainstorming phase)

## Goal

Model roundabouts as real roundabouts: a one-way circulating ring where
entering traffic yields to circulating traffic, circulating traffic has
absolute priority, and — on multi-lane rings — vehicles choose and change
lanes by exit position, weaving out to the outer lane before exiting.

Today the simulator has **no** roundabout support. OSM ways tagged
`junction=roundabout` fall through to ordinary junction logic (their nodes
become priority-road or AllWayStop intersections), and — because
`onewayDirection` does not treat `junction=roundabout`/`circular` as
implicitly one-way — a ring tagged without an explicit `oneway` tag is built
as a **two-way** edge, i.e. wrong-way traffic circulating the ring. This is a
latent geometry bug the design also fixes.

The end goal is **full multi-lane** roundabouts. To bound risk, a single
implementation plan stages the work into two independently shippable phases:

- **Phase A — correct one-way single-lane:** geometry fix, ring identity,
  entry-yield, circulating priority. Reuses the existing per-approach
  `Control` + `yieldGapCheck` machinery with essentially no new sim logic.
- **Phase B — multi-lane lane discipline:** lane choice on entry (tags first,
  then convention heuristic) and weaving to the outer lane before exit,
  feeding the existing `lanechange.go` mechanics.

## Major decisions (from brainstorming)

| Axis | Decision |
|---|---|
| Fidelity | Full multi-lane (geometry, entry-yield, circulating priority, lane discipline/weaving). |
| Lane policy | Honor `turn:lanes`/`destination:lanes` tags when present; otherwise convention heuristic by exit position. |
| Staging | One spec; implementation plan stages Phase A (single-lane correct) then Phase B (multi-lane). Each phase independently shippable/testable. |
| Architecture | Approach 1: reuse `Control` + mark ring edges. Persist only the minimal ring identity Phase B's exit-ordering needs. No new `Control` enum value, no new sim controller. |
| Handedness | Right-hand traffic (confirmed from code: lane 0 = curb/right, right turns snap to lane 0, left turns to the highest lane). Rings circulate counterclockwise; entries yield to traffic from the left. |
| Mini-roundabouts | Out of scope. Node-tagged `highway=mini_roundabout` (no ring geometry) is not handled. |
| Entry critical gap | New `roundaboutGapSec ≈ 3.5` in the `sim` package (entry critical gaps run ~3–4 s), shrinkable by `effectiveGap`/impatience, mirroring `leftTurnGapSec`. Not overloaded onto the 3.0 s straight-crossing `gapThresholdSec`. |
| Missed exit | A vehicle that cannot merge to the outer lane before its exit stays on the ring and loops around to try again. No dedicated loop-detection; the existing stuck-vehicle despawn is the backstop. |
| Rendering | No required renderer changes — ring segments draw as ordinary one-way edges once the geometry fix lands. |

## Why this architecture

The sim's `yieldGapCheck` (`internal/sim/world.go:489`) already makes a
`ControlYield` approach yield, via ETA-based gap-acceptance, to **every**
`ControlNone` (priority) approach at the node. At a roundabout entry node the
incoming edges are just `[approach-road-in, ring-segment-in]`. So if netbuild
sets the circulating ring segment to `ControlNone` and the entering road to
`ControlYield`, entering vehicles yield to circulating traffic with **no new
sim logic** — the only priority approach at that node *is* the circulating
one. This is why Phase A is small and why we reuse `Control` rather than
introduce a first-class roundabout controller (the rejected Approach 2).

## Phase A — one-way ring, entry-yield, circulating priority

### Detection (`internal/osmload`, `internal/netbuild`)

Recognize `junction=roundabout` and `junction=circular` on ways. (Mini-
roundabouts tagged on nodes are out of scope.)

### Geometry fix (`internal/netbuild/netbuild.go`, `onewayDirection`)

A `junction=roundabout`/`circular` way is implicitly one-way **forward** even
with no `oneway` tag. Add this to `onewayDirection` so the ring produces a
single directed edge per segment instead of a wrong-way two-way pair. An
explicit `oneway` tag on the way still wins (unchanged precedence).

### Ring identity (`internal/netbuild`)

- Mark circulating edges with `Edge.Roundabout bool`.
- Group the consecutive ring edges of one OSM loop into an ordered
  `Roundabout{ Edges []EdgeID }` (circulation order). Two adjacent
  roundabouts are two separate rings.
- Build a `edge→ring` lookup for the runtime forward-scan Phase B needs.

This grouping is computed once at build time and stored on the network.

### Control assignment (`internal/netbuild/control.go`)

At every node on a ring, applied **before** `applyClassFallback` /
`applyStopAllOrMinor`:

- the **circulating** incoming approach (ring-segment-in) → `ControlNone`
  (priority, never stops);
- each **entering** road approach → `ControlYield`;
- the node is **not** demoted to `ControlAllWayStop` and does not get
  priority-by-road-class.

**Scoping guard (no regressions):** this special-casing is gated strictly on
`Edge.Roundabout` for the node's edges. Non-ring nodes take the existing
control path untouched, so the change cannot regress the recent
junction/AllWayStop fixes (`straight-road-phantom-stops`,
`bent-through-road-overstops`).

### Sim behavior (Phase A) — essentially no new code

Entering vehicles fall through the existing `ControlYield` →
`yieldGapCheck` path and yield to the circulating edge automatically.
Circulating vehicles (`ControlNone`) never stop. The one addition is a
roundabout-specific critical-gap constant `roundaboutGapSec ≈ 3.5` in the
`sim` package, used by the entry gap-acceptance in place of
`gapThresholdSec`, and shrinkable by `effectiveGap`/impatience exactly like
`leftTurnGapSec`.

### Phase A edge cases

- **Adjacent entries sharing a short ring segment:** the upstream segment is
  the conflict source, which `yieldGapCheck` already scans by ETA.
- **Exit-only ring node (no entering road):** no yield needed; falls out
  naturally (no `ControlYield` approach).
- **Routing and turn restrictions:** already work — the ring is just normal
  directed edges in the routable graph.

## Phase B — multi-lane lane discipline

Phase B activates only where ring segments have **≥2 lanes** (from OSM
`lanes`). Single-lane rings are fully covered by Phase A and Phase B is a
no-op there. Right-hand traffic: **lane 0 = outer/right**, higher index =
inner/left.

### Exit distance from the route (runtime, no precompute)

A vehicle's route already lists the ring segments it will traverse. Scanning
`RouteIdx` forward to the segment where it leaves the ring gives "how far
around" (segment count / angular sweep), classifying the maneuver as *early*
(≈first exit / right), *middle*, or *late* (past ~12 o'clock / U-turn). The
ring grouping from Phase A makes this a bounded forward scan.

### Entry lane choice — tags first, then heuristic

- If the entry/ring edges carry `turn:lanes`/`destination:lanes`, the
  existing `Lane.AllowedTurns` machinery already encodes which lane feeds
  which continuation — pick a lane whose `AllowedTurns` lead to the vehicle's
  exit. Reuses the existing parsing; no new tag logic.
- Otherwise heuristic: *early* exit → outer lane (0); *late*/U-turn → inner
  lane (highest); *middle* → outer by default.

### Circulating + weaving — a target-lane policy feeding existing lane-change code

The lateral-move mechanics in `internal/sim/lanechange.go` (gap-checking the
target lane, executing the shift) are reused unchanged. Phase B adds one
focused function, `roundaboutTargetLane(v, ring)`:

- while **more than `K` segments** from its exit: prefer the inner lane (keeps
  the outer lane free for entering/exiting traffic);
- when **within `K` segments** of its exit: target lane 0 to exit.

This produces the weave-out. Because ring segments are short (often
10–30 m), the policy computes remaining-ring distance **across multiple
segments**, not just the current edge, and begins the move to lane 0 a
segment or two early. This multi-segment lookahead is the trickiest part of
Phase B and the focus of testing. `K` is a single tunable constant (start
at 1 segment).

### Phase B behavioral decisions

- **Missed exit → go around.** If a vehicle cannot merge to lane 0 before its
  exit (blocked), it stays on the ring and loops to try again next time
  around. No dedicated loop-detection; the existing stuck-vehicle despawn is
  the backstop against an infinite loop under gridlock.
- **`K` (how early to weave)** is a single tunable, adjustable after viewer
  observation.

## Data model changes (`internal/network`)

Minimal additions; no `Control` enum churn (we reuse `ControlNone` /
`ControlYield`):

- `Edge.Roundabout bool` — marks a circulating segment.
- `Network.Roundabouts []Roundabout`, where
  `Roundabout{ Edges []EdgeID }` lists ring segments in circulation order,
  plus a build-time `edge→ring` lookup. This is the minimal ring identity
  Approach 1 promised — present only because Phase B's exit-distance scan
  needs ordering.
- **Hashing (`internal/network/hash.go`):** fold the new fields into the
  network hash so the determinism/trace guarantee and any cache keys stay
  correct. Control values already flow into the hash via `IncomingControl`,
  and the ring data is derived deterministically from OSM, so determinism
  holds — hashing it keeps the invariant explicit.

Sim tunables (`roundaboutGapSec`, weave lookahead `K`) live in the `sim`
package next to `leftTurnGapSec`, not in `network`.

## Rendering (`internal/render`)

No required changes. Ring segments are ordinary one-way edges and draw
correctly once the geometry fix makes them one-way. No HUD or overlay work.

## Testing strategy

### Phase A

- **Unit (netbuild):** `onewayDirection` returns forward for
  `junction=roundabout`/`circular` with no `oneway` tag; an explicit `oneway`
  tag is still honored. A synthetic small-ring fixture gives circulating
  approaches `ControlNone`, entries `ControlYield`, and is **not** demoted to
  AllWayStop. Ring grouping yields edges in circulation order and treats two
  adjacent roundabouts as separate rings.
- **Sim integration:** in a ring fixture, an entering vehicle yields to a
  circulating vehicle via gap-acceptance; the circulating vehicle never
  stops. `TestWorld_TraceDeterminism` must still pass.

### Phase B

- **Unit:** `roundaboutTargetLane` — early exit stays lane 0; late exit uses
  the inner lane then targets lane 0 within `K`; a `turn:lanes`-tagged entry
  picks the tagged lane.
- **Integration:** a 2-lane ring fixture — an inner-lane vehicle weaves out
  and exits; a vehicle that cannot merge loops around (no despawn unless the
  existing stuck-timeout fires); the merge to lane 0 begins a segment early
  (multi-segment lookahead).

### E2E (`-tags e2e`)

Run against a real OSM extract containing a multi-lane roundabout and assert
ring edges are one-way and entries are `ControlYield`.

### Viewer acceptance

Per the project's realism-priority rule, viewer observation is the final
word: on a real map with a multi-lane roundabout — entries yield, circulating
flows, weaving looks plausible, and there are no wrong-way or phantom-stop
artifacts.

## Open items / risks

1. **Test extract:** identify a real OSM extract with a clean multi-lane
   roundabout in `/home/lab/MEGA/OpenStreetMap` for E2E and viewer
   validation.
2. **Short-segment weaving:** the multi-segment weave lookahead on short ring
   segments is the main correctness risk in Phase B.
3. **Missed-exit-loops-around** relies on the existing despawn backstop (an
   approved decision, recorded here so the risk is explicit).

## Out of scope

- Mini-roundabouts (`highway=mini_roundabout`, node-tagged, no ring
  geometry).
- Roundabout metering signals (signalized entry).
- Turbo-roundabout spiral lane markings beyond the tags-first / convention
  heuristic.
- Any renderer/HUD treatment specific to roundabouts.
