//go:build e2e

package e2e

import (
	"os"
	"testing"

	"github.com/lab1702/traffic-sim/internal/netbuild"
	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
)

// TestE2E_Roundabouts builds the network from a real OSM extract and checks
// the roundabout invariants Phase A guarantees: every circulating ring edge is
// one-way (no Roundabout reverse twin), and at every ring node the circulating
// approaches keep priority (ControlNone) while entering approaches yield
// (ControlYield) and none is demoted to an all-way stop.
//
// Skips when no extract is provided, or when the extract contains no
// roundabouts (so it never fails on a roundabout-free map). Use an extract
// known to contain roundabouts, e.g. jackson.osm.
func TestE2E_Roundabouts(t *testing.T) {
	path := os.Getenv("TRAFFIC_SIM_E2E_OSM")
	if path == "" {
		t.Skip("TRAFFIC_SIM_E2E_OSM not set; skipping E2E")
	}

	feat, err := osmload.Load(path)
	if err != nil {
		t.Fatal(err)
	}
	net, _, err := netbuild.Build(feat)
	if err != nil {
		t.Fatal(err)
	}

	// Collect ring edges and index them by (From,To) for reverse-twin lookup.
	type pair struct{ from, to network.NodeID }
	ringEdges := make(map[pair]bool)
	ringCount := 0
	for i := range net.Edges {
		e := &net.Edges[i]
		if e.Roundabout {
			ringCount++
			ringEdges[pair{e.From, e.To}] = true
		}
	}
	if ringCount == 0 {
		t.Skipf("extract %q has no roundabout edges; choose one that does (e.g. jackson.osm)", path)
	}
	t.Logf("found %d circulating ring edges", ringCount)

	// One-way: no ring edge may have a ring reverse twin.
	for p := range ringEdges {
		if ringEdges[pair{p.to, p.from}] {
			t.Errorf("ring edge %d->%d has a wrong-way ring twin %d->%d", p.from, p.to, p.to, p.from)
		}
	}

	// Control: inspect every ring node (an intersection with an incoming
	// roundabout edge).
	ringNodes := 0
	for i := range net.Intersections {
		x := &net.Intersections[i]
		onRing := false
		for _, eid := range x.Incoming {
			if net.Edges[eid].Roundabout {
				onRing = true
				break
			}
		}
		if !onRing {
			continue
		}
		ringNodes++
		for j, eid := range x.Incoming {
			c := x.IncomingControl[j]
			if c == network.ControlAllWayStop {
				t.Errorf("ring node (intersection %d): approach edge %d is AllWayStop", x.ID, eid)
			}
			if net.Edges[eid].Roundabout {
				if c != network.ControlNone {
					t.Errorf("circulating edge %d at intersection %d: got %v, want ControlNone", eid, x.ID, c)
				}
			} else if c != network.ControlYield {
				t.Errorf("entering edge %d at intersection %d: got %v, want ControlYield", eid, x.ID, c)
			}
		}
	}
	if ringNodes == 0 {
		t.Error("found ring edges but no ring nodes with an incoming ring edge — unexpected topology")
	}
	t.Logf("validated %d ring nodes", ringNodes)
}
