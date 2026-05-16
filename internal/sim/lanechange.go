package sim

import "github.com/lab1702/traffic-sim/internal/network"

const (
	laneChangeCheckGap = 50.0 // m: only consider LC if leader within this
	vDiffThreshold     = 5.0  // m/s: leader must be this much slower
	safetyGapFront     = 20.0 // m
	safetyGapRear      = 10.0 // m
	laneChangeCooldown = 3.0  // s
)

const turnBiasRange = 300.0 // meters before the intersection

// tryLaneChange mutates v.Lane if a beneficial change is available.
// laneVehicles[lane] is a sorted-by-S slice of vehicle indices on that
// lane of the current edge.
//
// Two modes:
//   - Turn bias: within turnBiasRange of an intersection where v will turn,
//     shift toward the nearest lane whose AllowedTurns includes the next
//     route edge. Skips the speed-difference threshold but keeps safety gaps.
//   - Speed-driven (existing): catch a faster gap on a neighbor lane.
//     When turn bias is active, this still runs only if the ego lane is
//     already compatible, AND candidate lanes that would become
//     incompatible are rejected.
func tryLaneChange(v *Vehicle, vi int, laneVehicles map[uint8][]int, vs []Vehicle, net *network.Network) {
	if v.LaneChangeCooldown > 0 {
		return
	}
	edge := &net.Edges[v.Edge]
	numLanes := uint8(len(edge.Lanes))
	if numLanes < 2 {
		return
	}

	// --- Turn-bias context ---
	var nextE network.EdgeID
	turnContext := false
	if v.RouteIdx+1 < len(v.Route) {
		nextE = v.Route[v.RouteIdx+1]
		dToInt := edge.Length - v.S
		if dToInt <= turnBiasRange && len(edge.Lanes[v.Lane].AllowedTurns) > 0 {
			turnContext = true
		}
	}
	myCompatible := !turnContext || laneAllows(edge.Lanes[v.Lane].AllowedTurns, nextE)

	// --- Turn bias branch: ego in incompatible lane, must migrate ---
	if turnContext && !myCompatible {
		_, dl, ok := nearestCompatibleLane(edge.Lanes, v.Lane, nextE)
		if !ok {
			return
		}
		nl := int(v.Lane) + int(dl)
		if nl < 0 || nl >= int(numLanes) {
			return
		}
		other := laneVehicles[uint8(nl)]
		frontS, hasFront := nextAheadS(other, vs, v.Edge, v.S)
		rearS, hasRear := nextBehindS(other, vs, v.Edge, v.S)
		if hasFront && frontS-v.S-VehicleLength < safetyGapFront {
			return
		}
		if hasRear && v.S-rearS-VehicleLength < safetyGapRear {
			return
		}
		v.Lane = uint8(nl)
		v.LaneChangeCooldown = laneChangeCooldown
		v.LastLCDir = dl
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
		if turnContext && !laneAllows(edge.Lanes[nl].AllowedTurns, nextE) {
			continue
		}
		other := laneVehicles[uint8(nl)]
		frontS, hasFront := nextAheadS(other, vs, v.Edge, v.S)
		rearS, hasRear := nextBehindS(other, vs, v.Edge, v.S)
		if hasFront && frontS-v.S-VehicleLength < safetyGapFront {
			continue
		}
		if hasRear && v.S-rearS-VehicleLength < safetyGapRear {
			continue
		}
		v.Lane = uint8(nl)
		v.LaneChangeCooldown = laneChangeCooldown
		v.LastLCDir = dl
		return
	}
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

// nextAheadS returns the S of the closest live vehicle on the lane (on
// the same edge as ego) with S > egoS. Despawned vehicles and vehicles
// that have transitioned onto the next edge earlier this tick are
// skipped — both would otherwise produce stale S values that could
// trigger spurious lane changes.
func nextAheadS(idxs []int, vs []Vehicle, egoEdge network.EdgeID, egoS float64) (float64, bool) {
	var best float64
	found := false
	for _, i := range idxs {
		v := &vs[i]
		if v.Despawned || v.Edge != egoEdge {
			continue
		}
		s := v.S
		if s <= egoS {
			continue
		}
		if !found || s < best {
			best, found = s, true
		}
	}
	return best, found
}

// nextBehindS returns the S of the closest live vehicle on the lane (on
// the same edge as ego) with S < egoS. Same filtering as nextAheadS.
func nextBehindS(idxs []int, vs []Vehicle, egoEdge network.EdgeID, egoS float64) (float64, bool) {
	var best float64
	found := false
	for _, i := range idxs {
		v := &vs[i]
		if v.Despawned || v.Edge != egoEdge {
			continue
		}
		s := v.S
		if s >= egoS {
			continue
		}
		if !found || s > best {
			best, found = s, true
		}
	}
	return best, found
}
