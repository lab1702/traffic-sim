package network

import (
	"encoding/binary"
	"hash/fnv"
	"math"
)

// Hash returns a 64-bit fingerprint of the network's topology. It covers
// node count, edge endpoints + length, and intersection structure — enough
// to detect that two networks were built from the same OSM input. Trace
// files record this hash in SimStart so a replayer can warn if the loaded
// OSM doesn't match the original.
//
// Edge lengths are rounded to millimeters before hashing so values that
// round-trip through OSM lat/lon projection don't drift the hash.
// Deterministic — same Network in, same hash out, regardless of run.
func Hash(net *Network) uint64 {
	h := fnv.New64a()
	var buf [8]byte

	binary.LittleEndian.PutUint64(buf[:], uint64(len(net.Nodes)))
	_, _ = h.Write(buf[:])
	binary.LittleEndian.PutUint64(buf[:], uint64(len(net.Edges)))
	_, _ = h.Write(buf[:])
	binary.LittleEndian.PutUint64(buf[:], uint64(len(net.Intersections)))
	_, _ = h.Write(buf[:])

	for i := range net.Edges {
		e := &net.Edges[i]
		binary.LittleEndian.PutUint32(buf[:4], uint32(e.From))
		_, _ = h.Write(buf[:4])
		binary.LittleEndian.PutUint32(buf[:4], uint32(e.To))
		_, _ = h.Write(buf[:4])
		binary.LittleEndian.PutUint64(buf[:], uint64(math.Round(e.Length*1000)))
		_, _ = h.Write(buf[:])
		binary.LittleEndian.PutUint32(buf[:4], uint32(len(e.Lanes)))
		_, _ = h.Write(buf[:4])
	}
	for i := range net.Intersections {
		x := &net.Intersections[i]
		binary.LittleEndian.PutUint32(buf[:4], uint32(x.NodeID))
		_, _ = h.Write(buf[:4])
		binary.LittleEndian.PutUint32(buf[:4], uint32(len(x.Incoming)))
		_, _ = h.Write(buf[:4])
		binary.LittleEndian.PutUint32(buf[:4], uint32(len(x.Outgoing)))
		_, _ = h.Write(buf[:4])
	}
	return h.Sum64()
}
