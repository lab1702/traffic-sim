package sim

import "github.com/lab1702/traffic-sim/internal/network"

const (
	laneChangeCheckGap = 50.0 // m: only consider LC if leader within this
	vDiffThreshold     = 5.0  // m/s: leader must be this much slower
	safetyGapFront     = 20.0 // m: minimum gap to the new leader
	safetyGapRear      = 10.0 // m: minimum gap to the new follower
	laneChangeCooldown = 3.0  // s
)

// laneGapSafe reports whether ego (vehicle index vi) can move into the lane
// holding vehicles idxs without cutting anyone off. Both neighbours in the
// target lane must clear the fixed minimum gaps AND the speed-aware MOBIL
// safety criterion: neither ego (behind its new leader) nor the new follower
// (behind ego) may be forced to brake harder than SafeDecel. The fixed gaps
// alone ignore closing speed, so a slow car could pull in 10 m ahead of one
// doing 20 m/s.
func laneGapSafe(v *Vehicle, vi int, idxs []int, vs []Vehicle, edge *network.Edge) bool {
	ai, bi := laneNeighbors(idxs, vs, vi, v.Edge, v.S)
	p := DefaultIDM()
	if ai >= 0 {
		a := &vs[ai]
		gap := a.S - v.S - VehicleLength
		if gap < safetyGapFront || !safeToFollow(p, v.V, edge.SpeedLimit, gap, a.V) {
			return false
		}
	}
	if bi >= 0 {
		b := &vs[bi]
		gap := v.S - b.S - VehicleLength
		if gap < safetyGapRear || !safeToFollow(p, b.V, edge.SpeedLimit, gap, v.V) {
			return false
		}
	}
	return true
}

// changeLane moves v one lane in direction dl and starts the cooldown.
func changeLane(v *Vehicle, dl int8) {
	v.Lane = uint8(int(v.Lane) + int(dl))
	v.LaneChangeCooldown = laneChangeCooldown
	v.LastLCDir = dl
}

const turnBiasRange = 300.0 // meters before the intersection

// tryLaneChange mutates v.Lane if a beneficial change is available.
// laneVehicles[lane] is a sorted-by-S slice of vehicle indices on that
// lane of the current edge.
//
// Four modes:
//   - Incident vacate: when the vehicle is in an incident's closed lane,
//     move to an adjacent open lane as soon as a safe gap exists (takes
//     priority over the other modes; still respects safety gaps).
//   - Roundabout weave: on a multi-lane ring, migrate one step toward the
//     target lane (inner while circulating, outer within K of the exit).
//     Runs after incident-vacate and before turn bias; respects safety gaps.
//   - Turn bias: within turnBiasRange of an intersection where v will turn,
//     shift toward the nearest lane that suits the route ahead: its
//     AllowedTurns include the next route edge and, through short edges,
//     the turns after it (see routeLaneMask). Skips the speed-difference
//     threshold but keeps safety gaps.
//   - Speed-driven (existing): catch a faster gap on a neighbor lane.
//     When turn bias is active, this still runs only if the ego lane is
//     already compatible, AND candidate lanes that would become
//     incompatible are rejected.
func tryLaneChange(v *Vehicle, vi int, laneVehicles map[uint8][]int, vs []Vehicle, net *network.Network, closedLane int8) {
	if v.LaneChangeCooldown > 0 {
		return
	}
	edge := &net.Edges[v.Edge]
	numLanes := uint8(len(edge.Lanes))
	if numLanes < 2 {
		return
	}

	// Incident: ego is in a closed lane — vacate to an adjacent open lane as
	// soon as a safe gap exists (overrides the normal speed/turn incentive,
	// but never overrides the safety gaps).
	if closedLane >= 0 && v.Lane == uint8(closedLane) {
		for _, dl := range []int8{1, -1} {
			nl := int(v.Lane) + int(dl)
			if nl < 0 || nl >= int(numLanes) || nl == int(closedLane) { // last clause defensive: nl != closedLane here
				continue
			}
			if !laneGapSafe(v, vi, laneVehicles[uint8(nl)], vs, edge) {
				continue
			}
			changeLane(v, dl)
			return
		}
		return // blocked in the closed lane; don't fall through to normal LC
	}

	// Roundabout weave: on a multi-lane ring, migrate one step toward the
	// target lane (inner while circulating, outer within K of the exit).
	// Reuses the same safety-gap checks as the other modes.
	if target, ok := roundaboutTargetLane(v, net); ok && target != v.Lane {
		dl := int8(1)
		if target < v.Lane {
			dl = -1
		}
		nl := int(v.Lane) + int(dl)
		if nl >= 0 && nl < int(numLanes) && nl != int(closedLane) &&
			laneGapSafe(v, vi, laneVehicles[uint8(nl)], vs, edge) {
			changeLane(v, dl)
		}
		return // on a ring, the weave policy owns lane choice this tick
	}

	// --- Turn-bias context ---
	// fit has bit l set when lane l suits the route ahead (routeLaneMask);
	// turnContext means some lane does not, so lane choice matters.
	var fit uint64
	turnContext := false
	if v.RouteIdx+1 < len(v.Route) && edge.Length-v.S <= turnBiasRange && numLanes <= 64 {
		fit = routeLaneMask(v, net, numLanes)
		turnContext = fit != 1<<numLanes-1
	}
	myCompatible := !turnContext || fit&(1<<v.Lane) != 0

	// --- Turn bias branch: ego in incompatible lane, must migrate ---
	if turnContext && !myCompatible {
		dl, ok := nearestLaneIn(fit, v.Lane, numLanes)
		if !ok {
			return
		}
		nl := int(v.Lane) + int(dl)
		if nl < 0 || nl >= int(numLanes) {
			return
		}
		// Never steer into a lane an incident has closed, even when it is the
		// only lane allowing the next turn: the vehicle would just have to
		// vacate it again at the obstacle. It takes the turn from where it is
		// (the post-turn snap) instead.
		if nl == int(closedLane) {
			return
		}
		if !laneGapSafe(v, vi, laneVehicles[uint8(nl)], vs, edge) {
			return
		}
		changeLane(v, dl)
		return
	}

	// --- Speed-driven (existing logic, with turn-aware suppression) ---
	myLane := v.Lane
	same := laneVehicles[myLane]
	var myPos int = -1
	for i, idx := range same {
		if idx == vi {
			myPos = i
			break
		}
	}
	if myPos < 0 {
		return
	}
	// Find the next live, same-edge leader ahead in this lane. The
	// laneVehicles bucket was built at start-of-tick; by now an earlier
	// vehicle may have despawned or crossed onto the next edge (its S
	// reset to near 0), which would otherwise show as a wildly negative
	// gap and force a spurious lane change.
	var leaderV float64 = edge.SpeedLimit
	var leaderS float64 = edge.Length + 1e6
	for j := myPos + 1; j < len(same); j++ {
		ld := &vs[same[j]]
		if ld.Despawned || ld.Edge != v.Edge {
			continue
		}
		leaderV, leaderS = ld.V, ld.S
		break
	}
	leaderGap := leaderS - v.S - VehicleLength
	if leaderGap > laneChangeCheckGap || edge.SpeedLimit-leaderV < vDiffThreshold {
		return
	}

	for _, dl := range []int8{-1, 1} {
		nl := int(myLane) + int(dl)
		if nl < 0 || nl >= int(numLanes) {
			continue
		}
		if nl == int(closedLane) {
			continue // an empty closed lane looks fast, but it ends at the obstacle
		}
		if turnContext && fit&(1<<uint(nl)) == 0 {
			continue
		}
		if !laneGapSafe(v, vi, laneVehicles[uint8(nl)], vs, edge) {
			continue
		}
		changeLane(v, dl)
		return
	}
}

// laneChangeRoom is the shortest edge on which a vehicle can still be counted
// on to change lanes before its end. netbuild splits roads at every shared
// node, so the only lane that reaches a turn lane may have to be chosen on
// the edge before a short one; turn bias plans the lane through edges
// shorter than this (see laneFitsRoute).
const laneChangeRoom = 50.0 // m

// routeLaneMask returns a bit mask of the lanes of v's current edge that suit
// its route ahead (bit l: lane l), for edges of at most 64 lanes. A lane
// suits it when the vehicle can stay in it, and in the lanes it carries into
// through each node, through every turn up to and including the end of the
// first edge long enough to change lanes on (laneFitsRoute). When no lane
// manages that, it falls back to the lanes whose AllowedTurns include the
// next route edge, the one-edge rule.
func routeLaneMask(v *Vehicle, net *network.Network, numLanes uint8) uint64 {
	lanes := net.Edges[v.Edge].Lanes
	nextE := v.Route[v.RouteIdx+1]
	var planned, next uint64
	for l := uint8(0); l < numLanes; l++ {
		if !laneAllows(lanes[l].AllowedTurns, nextE) {
			continue
		}
		next |= 1 << l
		if laneFitsRoute(v, net, l) {
			planned |= 1 << l
		}
	}
	if planned != 0 {
		return planned
	}
	return next
}

// laneFitsRoute reports whether a vehicle staying in `lane` of its current
// edge, and in the lane postTurnLane carries it into at each node, can make
// every turn ahead without changing lanes, up to the end of the first edge
// at least laneChangeRoom long (where it can change lanes if it must) or
// turnBiasRange metres, whichever comes first.
func laneFitsRoute(v *Vehicle, net *network.Network, lane uint8) bool {
	e := v.Edge
	d := net.Edges[e].Length - v.S
	for j := v.RouteIdx + 1; j < len(v.Route) && d <= turnBiasRange; j++ {
		next := v.Route[j]
		if lanes := net.Edges[e].Lanes; int(lane) < len(lanes) && !laneAllows(lanes[lane].AllowedTurns, next) {
			return false
		}
		if int(next) >= len(net.Edges) || net.Edges[next].Length >= laneChangeRoom {
			return true // room to change lanes there (or nothing known to plan for)
		}
		lane = postTurnLane(lane, network.ClassifyTurn(net, e, next), uint8(len(net.Edges[next].Lanes)))
		e = next
		d += net.Edges[e].Length
	}
	return true
}

// nearestLaneIn returns the direction step (±1) from lane `from` toward the
// nearest lane in mask (bit l: lane l), ties toward the lower index, and
// false when mask holds no lane below numLanes.
func nearestLaneIn(mask uint64, from, numLanes uint8) (int8, bool) {
	best, bestDist := -1, 1<<30
	for l := 0; l < int(numLanes); l++ {
		if mask&(1<<uint(l)) == 0 {
			continue
		}
		d := l - int(from)
		if d < 0 {
			d = -d
		}
		if d < bestDist {
			best, bestDist = l, d
		}
	}
	if best < 0 {
		return 0, false
	}
	if best < int(from) {
		return -1, true
	}
	return 1, true
}

// laneAllows reports whether `nextE` is in the lane's AllowedTurns list.
// Empty list means "any outgoing edge" (per the network.Lane schema doc).
func laneAllows(allowed []network.EdgeID, nextE network.EdgeID) bool {
	if len(allowed) == 0 {
		return true
	}
	for _, e := range allowed {
		if e == nextE {
			return true
		}
	}
	return false
}

// nearestCompatibleLane returns the index, direction step (±1), and ok
// flag for the lane closest to `fromLane` whose AllowedTurns includes
// `nextE`. Tie-breaks toward lower index (rightmost side).
func nearestCompatibleLane(lanes []network.Lane, fromLane uint8, nextE network.EdgeID) (uint8, int8, bool) {
	bestIdx := uint8(0)
	bestDist := 1 << 30
	found := false
	for i, l := range lanes {
		if !laneAllows(l.AllowedTurns, nextE) {
			continue
		}
		d := int(i) - int(fromLane)
		ad := d
		if ad < 0 {
			ad = -ad
		}
		if ad < bestDist || (ad == bestDist && uint8(i) < bestIdx) {
			bestDist = ad
			bestIdx = uint8(i)
			found = true
		}
	}
	if !found {
		return 0, 0, false
	}
	dl := int8(1)
	if int(bestIdx) < int(fromLane) {
		dl = -1
	}
	return bestIdx, dl, true
}

// laneNeighbors returns the indices of the closest live vehicles in a lane
// (on ego's edge) ahead of and behind egoS, or -1 when there is none. A
// vehicle exactly level with ego counts as ahead, so a car alongside always
// blocks the change. Ego itself (index vi) is skipped. Despawned vehicles and
// vehicles that have moved onto the next edge earlier this tick are skipped
// too — both would otherwise produce stale S values that could trigger
// spurious lane changes.
func laneNeighbors(idxs []int, vs []Vehicle, vi int, egoEdge network.EdgeID, egoS float64) (ahead, behind int) {
	ahead, behind = -1, -1
	for _, i := range idxs {
		if i == vi {
			continue
		}
		v := &vs[i]
		if v.Despawned || v.Edge != egoEdge {
			continue
		}
		if v.S >= egoS {
			if ahead < 0 || v.S < vs[ahead].S {
				ahead = i
			}
		} else if behind < 0 || v.S > vs[behind].S {
			behind = i
		}
	}
	return ahead, behind
}

// roundaboutTargetLane returns the lane a circulating vehicle should occupy,
// and ok=false when the vehicle is not on a multi-lane ring (so callers fall
// back to normal lane-change logic). Policy:
//   - tags first: if the current ring edge's lanes constrain turns via
//     AllowedTurns, target the nearest lane that feeds the vehicle's next
//     route edge;
//   - otherwise heuristic by exit distance: within roundaboutWeaveLookahead
//     segments of the exit -> outer lane 0; farther -> inner lane.
func roundaboutTargetLane(v *Vehicle, net *network.Network) (uint8, bool) {
	if int(v.Edge) >= len(net.Edges) {
		return 0, false
	}
	edge := &net.Edges[v.Edge]
	if !edge.Roundabout || len(edge.Lanes) < 2 {
		return 0, false
	}
	nLanes := uint8(len(edge.Lanes))

	// Tags first: honor AllowedTurns toward the next route edge when some lane
	// actually constrains the turn.
	if v.RouteIdx+1 < len(v.Route) {
		anyConstrained := false
		for i := range edge.Lanes {
			if len(edge.Lanes[i].AllowedTurns) > 0 {
				anyConstrained = true
				break
			}
		}
		if anyConstrained {
			nextE := v.Route[v.RouteIdx+1]
			if lane, _, ok := nearestCompatibleLane(edge.Lanes, v.Lane, nextE); ok {
				return lane, true
			}
		}
	}

	// Heuristic by exit distance.
	if roundaboutSegmentsToExit(v, net) <= roundaboutWeaveLookahead {
		return 0, true // outer lane to exit
	}
	return nLanes - 1, true // inner lane while circulating
}

// roundaboutSegmentsToExit returns how many ring segments the vehicle will
// traverse, counting from its current edge, before leaving the roundabout.
// The exit is the first non-Roundabout edge in the remaining route. The count
// includes the current segment (1 == "the next edge is the exit"). Returns 0
// when the vehicle is not currently on a ring edge.
func roundaboutSegmentsToExit(v *Vehicle, net *network.Network) int {
	if int(v.Edge) >= len(net.Edges) || !net.Edges[v.Edge].Roundabout {
		return 0
	}
	count := 0
	for i := v.RouteIdx; i < len(v.Route); i++ {
		e := v.Route[i]
		if int(e) >= len(net.Edges) || !net.Edges[e].Roundabout {
			break
		}
		count++
	}
	return count
}

// moveLaneBucket keeps a per-lane bucket map current after vehicle vi
// changes lane from `from` to `to`: it leaves the old bucket and is inserted
// into the new one in S order. Without this, two vehicles approaching an
// empty lane from either side could both judge it free in the same tick and
// merge into the same spot.
func moveLaneBucket(lanes map[uint8][]int, vi int, from, to uint8, vs []Vehicle) {
	if old := lanes[from]; old != nil {
		for k, idx := range old {
			if idx == vi {
				lanes[from] = append(old[:k], old[k+1:]...)
				break
			}
		}
	}
	dst := lanes[to]
	k := len(dst)
	for k > 0 && vs[dst[k-1]].S > vs[vi].S {
		k--
	}
	dst = append(dst, 0)
	copy(dst[k+1:], dst[k:])
	dst[k] = vi
	lanes[to] = dst
}
