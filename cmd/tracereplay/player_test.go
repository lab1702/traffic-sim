package main

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

// TestNetHashMismatch covers the SimStart check that warns when the
// loaded OSM doesn't match the original run's network. Zero traceHash
// is the legacy-trace short-circuit; non-zero values are compared.
func TestNetHashMismatch(t *testing.T) {
	// Tiny network — Hash() returns a deterministic non-zero value.
	net := &network.Network{
		Nodes:         []network.Node{{ID: 0}, {ID: 1}},
		Edges:         []network.Edge{{From: 0, To: 1, Length: 100, Lanes: []network.Lane{{}}}},
		Intersections: []network.Intersection{{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0}}},
	}
	loaded := network.Hash(net)
	if loaded == 0 {
		t.Fatal("test setup: Hash returned 0 for a non-empty network")
	}

	t.Run("zero trace hash is silently skipped (legacy)", func(t *testing.T) {
		mismatch, _, _ := netHashMismatch(net, 0)
		if mismatch {
			t.Errorf("zero traceHash should NOT flag mismatch (legacy traces)")
		}
	})
	t.Run("matching hashes pass", func(t *testing.T) {
		mismatch, _, _ := netHashMismatch(net, loaded)
		if mismatch {
			t.Errorf("matching hashes should NOT flag mismatch")
		}
	})
	t.Run("differing hashes flag mismatch", func(t *testing.T) {
		mismatch, traceH, loadedH := netHashMismatch(net, loaded^0xDEADBEEF)
		if !mismatch {
			t.Errorf("differing hashes should flag mismatch")
		}
		if loadedH != loaded {
			t.Errorf("returned loadedHash = %x, want %x", loadedH, loaded)
		}
		if traceH != loaded^0xDEADBEEF {
			t.Errorf("returned traceHash = %x, want %x", traceH, loaded^0xDEADBEEF)
		}
	})
}
