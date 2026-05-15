//go:build e2e

// Package e2e runs the full pipeline against a small real OSM extract.
// Gate this behind the `e2e` build tag because it requires a fixture file
// not bundled in the repo; download instructions are in this directory's
// testdata/README.md.
package e2e

import (
	"bytes"
	"os"
	"testing"

	"github.com/lab1702/traffic-sim/internal/netbuild"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/lab1702/traffic-sim/internal/sim"
	"github.com/lab1702/traffic-sim/internal/trace"
)

func TestE2E_RealOSM_HeadlessRun(t *testing.T) {
	path := os.Getenv("TRAFFIC_SIM_E2E_OSM")
	if path == "" {
		t.Skip("TRAFFIC_SIM_E2E_OSM not set; skipping E2E")
	}
	feat, err := osmload.Load(path)
	if err != nil {
		t.Fatal(err)
	}
	net, rpt, err := netbuild.Build(feat)
	if err != nil {
		t.Fatal(err)
	}
	if len(net.Nodes) < 100 {
		t.Fatalf("expected non-trivial graph, got %d nodes", len(net.Nodes))
	}
	t.Logf("graph: nodes=%d edges=%d intersections=%d ways_skipped=%d components_dropped=%d",
		len(net.Nodes), len(net.Edges), len(net.Intersections),
		rpt.WaysSkipped, rpt.ComponentsDropped)

	spawner := sim.NewRandomOD(net, 42, 10.0)
	w := sim.NewWorld(net, spawner, nil)
	var buf bytes.Buffer
	tw := trace.NewWriter(&buf)
	w.EmitTrace = func(tick uint64, simTime float64, e trace.Event) {
		_ = tw.Write(tick, simTime, e)
	}
	w.Run(60.0) // 60 sim-seconds

	if w.Tick == 0 {
		t.Fatalf("no ticks executed")
	}
	if buf.Len() == 0 {
		t.Errorf("no trace bytes written")
	}
	// Read back, count spawn/despawn events.
	r := trace.NewReader(&buf)
	spawns, despawns := 0, 0
	for {
		_, ev, err := r.Next()
		if err != nil {
			break
		}
		switch ev.(type) {
		case *trace.VehicleSpawn:
			spawns++
		case *trace.VehicleDespawn:
			despawns++
		}
	}
	if spawns == 0 {
		t.Errorf("no spawns recorded")
	}
	t.Logf("trace: %d spawns, %d despawns", spawns, despawns)
}
