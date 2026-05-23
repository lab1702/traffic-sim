# Right-of-way fallback: "terminating road yields"

**Date:** 2026-05-23
**Status:** Approved

## Problem

For unsigned intersections (no OSM `highway=traffic_signals`, `highway=stop`,
`highway=give_way`, `stop=all`, or `stop=minor`), `netbuild`'s
`applyClassFallback` assigns right-of-way by functional class only:

- Unequal classes → the lower-class approaches get a mandatory `Stop`.
- Equal classes → **every** approach gets `AllWayStop`.

The equal-class rule over-stops through-traffic. A residential road that
continues straight through a same-class T-junction, or past a same-class
one-way spur, is forced to all-way-stop even though no cross traffic
conflicts with it. The mandatory `Stop` on unsigned minor roads is likewise
heavier than real-world give-way behavior.

(The companion fix — way-join nodes wrongly promoted to all-way stops on
straight roads — is already shipped via `distinctNeighbors` in
`control.go`; this spec covers the remaining genuine-junction cases.)

## Model: terminating roads yield, through roads keep priority

At each unsigned junction (≥3 distinct neighbors — way-joins are already
left uncontrolled):

1. Find the highest-priority class present, `best`.
2. Classify each best-class approach using the `Opposing` array:
   - **through** — has an opposing partner that is also best-class (its road
     continues straight across the junction).
   - **terminating** — no such partner (a T-stem).
3. Assign controls:
   - Lower-class approaches → `Yield`.
   - Best-class **terminating** approaches → `Yield`.
   - Best-class **through** approaches → `None` (priority).
   - **Exception:** if best-class through approaches form ≥2 crossing axes (a
     genuine 4-way of equal class), or there are ≥2 best-class arms with no
     through road at all (ambiguous Y), → `AllWayStop` for every approach.
   - Degenerate single best-class approach → `None`.

Explicit OSM signage still wins: `applyStopAllOrMinor`, `applyNodeLevelSign`,
and `applyInteriorNodeSign` run after the fallback and override it, so tagged
`highway=stop` / `stop=minor` / `stop=all` produce real stops as before.

### Case table

| Junction (unsigned) | Before | After |
|---|---|---|
| Equal-class T (through + side stem) | all-way stop | through `None`, stem `Yield` |
| Equal-class one-way spur diverge | all-way stop | through `None` |
| Major/minor (T or 4-way) | minor `Stop` | minor `Yield`, major `None` |
| Equal-class true 4-way crossing | all-way stop | all-way stop (kept) |
| Ambiguous equal-class Y (no through pair) | all-way stop | all-way stop |
| Tagged stop=all / highway=stop / stop=minor | as tagged | unchanged |

## Mechanism

`internal/netbuild`, two changes:

1. **Reorder** in `Build`: run `resolveOpposing` (and its `partialNet`)
   *before* `resolveControls`, so the fallback can read `x.Opposing`. Both
   already run after `sortIncomingByPriority`; `resolveOpposing` only needs
   edge geometry and is independent of controls, so the move is safe.

2. **Rewrite** `applyClassFallback` per the model above. It already receives
   the intersection and a `classOfEdge` closure; it additionally reads
   `x.Opposing` (now populated) to find through vs terminating approaches.

No new control primitives; uses existing `None` / `Yield` / `Stop` /
`AllWayStop`.

## Testing

TDD. New `netbuild` tests:

- Equal-class T → through approaches `None`, stem `Yield`.
- Equal-class one-way spur diverge → through `None`.
- Major/minor T → minor `Yield`, major `None`.
- Equal-class true 4-way → still `AllWayStop` (regression guard).

Updated existing tests (old behavior was the bug being fixed):

- `TestNetbuild_Fallback_UnequalClass`: residential approach `Stop` → `Yield`.
- `TestNetbuild_InteriorNodeStop`: service approaches `Stop` → `Yield`.
- `TestNetbuild_InteriorNodeGiveWay`: service approaches `Stop` → `Yield`.

`TestNetbuild_Fallback_EqualClass` (a 4-way) stays green. Full suite +
headless run on a real map must pass with no new stuck-vehicle warnings.

## Out of scope

- Priority-to-the-right modeling (no primitive; equal 4-ways stay all-way-stop).
- Traffic-volume- or length-based tie-breaking between equal-class roads.
