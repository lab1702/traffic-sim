package network

import (
	"encoding/binary"
	"hash/fnv"
	"math"
	"sort"
)

// Hash returns a 64-bit fingerprint of the network. It covers topology
// AND the control configuration the sim relies on: edge endpoints,
// lengths, lane counts, speed limits, per-intersection structure, the
// per-approach right-of-way Control values, banned turns, and HasSignal.
//
// Two networks built from the same OSM input by the same builder produce
// the same hash. Any change to the topology, the lane geometry, the
// speed-limit field, or the resolved sign/signal/restriction state
// changes the hash. Replayers compare this value from SimStart against
// the loaded network and warn on mismatch.
//
// Floats are rounded before hashing so values that round-trip through
// the OSM lat/lon projection don't drift the hash spuriously:
//   - lengths to millimeters
//   - speed limits to mm/s
//
// Deterministic — same Network in, same hash out, regardless of run.
func Hash(net *Network) uint64 {
	h := fnv.New64a()
	var buf [8]byte
	putU := func(v uint64) {
		binary.LittleEndian.PutUint64(buf[:], v)
		_, _ = h.Write(buf[:])
	}
	putU32 := func(v uint32) {
		binary.LittleEndian.PutUint32(buf[:4], v)
		_, _ = h.Write(buf[:4])
	}
	putU8 := func(v uint8) {
		buf[0] = v
		_, _ = h.Write(buf[:1])
	}

	putU(uint64(len(net.Nodes)))
	putU(uint64(len(net.Edges)))
	putU(uint64(len(net.Intersections)))

	for i := range net.Edges {
		e := &net.Edges[i]
		putU32(uint32(e.From))
		putU32(uint32(e.To))
		putU(uint64(math.Round(e.Length * 1000)))
		putU(uint64(math.Round(e.SpeedLimit * 1000)))
		putU32(uint32(len(e.Lanes)))
	}
	for i := range net.Intersections {
		x := &net.Intersections[i]
		putU32(uint32(x.NodeID))
		if x.HasSignal {
			putU8(1)
		} else {
			putU8(0)
		}
		// Per-approach edges + controls. Incoming order is deterministic
		// (set by netbuild's sortIncomingByPriority).
		putU32(uint32(len(x.Incoming)))
		for j, eid := range x.Incoming {
			putU32(uint32(eid))
			var c Control
			if j < len(x.IncomingControl) {
				c = x.IncomingControl[j]
			}
			putU8(uint8(c))
		}
		putU32(uint32(len(x.Outgoing)))
		for _, eid := range x.Outgoing {
			putU32(uint32(eid))
		}
		// Banned turns: sort for stability — order of insertion into
		// BannedTurns is not part of the externally visible contract.
		bans := append([]TurnRestriction(nil), x.BannedTurns...)
		sort.Slice(bans, func(a, b int) bool {
			if bans[a].From != bans[b].From {
				return bans[a].From < bans[b].From
			}
			return bans[a].To < bans[b].To
		})
		putU32(uint32(len(bans)))
		for _, br := range bans {
			putU32(uint32(br.From))
			putU32(uint32(br.To))
		}
	}
	return h.Sum64()
}
