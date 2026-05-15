package sim

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

func TestIncomingPos_FindsEdgePosition(t *testing.T) {
	x := network.Intersection{
		ID:       0,
		NodeID:   5,
		Incoming: []network.EdgeID{10, 20, 30},
	}
	if p := IncomingPos(&x, 20); p != 1 {
		t.Errorf("want 1, got %d", p)
	}
	if p := IncomingPos(&x, 99); p != -1 {
		t.Errorf("missing edge should return -1, got %d", p)
	}
}
