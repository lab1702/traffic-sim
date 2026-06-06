//go:build e2e

package e2e

import (
	"os"
	"testing"

	"github.com/lab1702/traffic-sim/internal/netbuild"
	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/lab1702/traffic-sim/internal/sim"
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

// TestE2E_RoundaboutWeave exercises Phase B (multi-lane lane discipline) on a
// real extract: it runs a headless World and confirms vehicles circulating a
// multi-lane ring actually use the inner lane and weave (change lanes on a ring
// edge), rather than staying glued to lane 0. Skips unless the extract has a
// multi-lane roundabout (e.g. bigi.osm) and the run actually routes vehicles
// through it, so it is a no-op on single-lane-only maps.
//
// Deterministic: fixed seed + spawn rate + tick count, so the observed counts
// are stable across runs.
func TestE2E_RoundaboutWeave(t *testing.T) {
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

	multiRing := make(map[network.EdgeID]bool)
	for i := range net.Edges {
		e := &net.Edges[i]
		if e.Roundabout && len(e.Lanes) >= 2 {
			multiRing[e.ID] = true
		}
	}
	if len(multiRing) == 0 {
		t.Skipf("extract %q has no multi-lane roundabout; choose one that does (e.g. bigi.osm)", path)
	}

	w := sim.NewWorld(net, sim.NewRandomOD(net, 7, 25.0), nil)

	lastLane := make(map[sim.VehicleID]uint8)    // last lane seen on a multi-ring edge
	ringVisitors := make(map[sim.VehicleID]bool) // vehicles seen on a multi-ring edge
	innerUsers := make(map[sim.VehicleID]bool)   // used the inner lane (>=1) on a multi-ring edge
	weaves := 0                                  // lane changes while on a multi-ring edge

	const ticks = 6000 // 300 sim-seconds at 20 Hz
	for tick := 0; tick < ticks; tick++ {
		w.Step()
		for i := range w.Vehicles {
			v := &w.Vehicles[i]
			if v.Despawned || !multiRing[v.Edge] {
				continue
			}
			ringVisitors[v.ID] = true
			if v.Lane >= 1 {
				innerUsers[v.ID] = true
			}
			if prev, ok := lastLane[v.ID]; ok && prev != v.Lane {
				weaves++
			}
			lastLane[v.ID] = v.Lane
		}
	}

	t.Logf("multi-lane ring edges=%d  visitors=%d  inner-lane users=%d  weaves=%d",
		len(multiRing), len(ringVisitors), len(innerUsers), weaves)

	if len(ringVisitors) == 0 {
		t.Skip("no vehicles routed through the multi-lane ring in this run; cannot assess weaving")
	}
	if len(innerUsers) == 0 {
		t.Error("vehicles circulated the multi-lane ring but none used the inner lane — Phase B weave inactive")
	}
	if weaves == 0 {
		t.Error("no lane changes observed on the multi-lane ring — vehicles never weaved")
	}
}
