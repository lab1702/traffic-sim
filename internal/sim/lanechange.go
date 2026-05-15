package sim

import "github.com/lab1702/traffic-sim/internal/network"

const (
	laneChangeCheckGap = 50.0 // m: only consider LC if leader within this
	vDiffThreshold     = 5.0  // m/s: leader must be this much slower
	safetyGapFront     = 20.0 // m
	safetyGapRear      = 10.0 // m
	laneChangeCooldown = 3.0  // s
)

// tryLaneChange mutates v.Lane if a beneficial change is available.
// laneVehicles[lane] is a sorted-by-S slice of vehicle indices on that
// lane of the current edge.
func tryLaneChange(v *Vehicle, vi int, laneVehicles map[uint8][]int, vs []Vehicle, net *network.Network) {
	if v.LaneChangeCooldown > 0 {
		return
	}
	edge := &net.Edges[v.Edge]
	numLanes := uint8(len(edge.Lanes))
	if numLanes < 2 {
		return
	}

	// Find same-lane leader.
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
	var leaderV float64 = edge.SpeedLimit
	var leaderS float64 = edge.Length + 1e6 // effectively infinity
	if myPos+1 < len(same) {
		ld := &vs[same[myPos+1]]
		leaderV, leaderS = ld.V, ld.S
	}
	leaderGap := leaderS - v.S - VehicleLength
	if leaderGap > laneChangeCheckGap || edge.SpeedLimit-leaderV < vDiffThreshold {
		return // no reason to change
	}

	// Try each neighbor lane.
	for _, dl := range []int8{-1, 1} {
		nl := int(myLane) + int(dl)
		if nl < 0 || nl >= int(numLanes) {
			continue
		}
		other := laneVehicles[uint8(nl)]
		// Find the position where ego would sit and check front/rear gaps.
		frontS, hasFront := nextAheadS(other, vs, v.S)
		rearS, hasRear := nextBehindS(other, vs, v.S)
		if hasFront && frontS-v.S-VehicleLength < safetyGapFront {
			continue
		}
		if hasRear && v.S-rearS-VehicleLength < safetyGapRear {
			continue
		}
		// Commit the change.
		v.Lane = uint8(nl)
		v.LaneChangeCooldown = laneChangeCooldown
		return
	}
}

// nextAheadS returns the S of the closest vehicle on the lane with S > egoS.
func nextAheadS(idxs []int, vs []Vehicle, egoS float64) (float64, bool) {
	var best float64
	found := false
	for _, i := range idxs {
		s := vs[i].S
		if s <= egoS {
			continue
		}
		if !found || s < best {
			best, found = s, true
		}
	}
	return best, found
}

// nextBehindS returns the S of the closest vehicle on the lane with S < egoS.
func nextBehindS(idxs []int, vs []Vehicle, egoS float64) (float64, bool) {
	var best float64
	found := false
	for _, i := range idxs {
		s := vs[i].S
		if s >= egoS {
			continue
		}
		if !found || s > best {
			best, found = s, true
		}
	}
	return best, found
}
