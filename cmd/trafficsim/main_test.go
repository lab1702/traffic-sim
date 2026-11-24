package main

import (
	"context"
	"io"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/sim"
	"github.com/lab1702/traffic-sim/internal/trace"
)

// crossOSM is a single signalled four-way junction: a primary N-S road
// crossing a residential E-W road. It builds to one intersection (ID 0)
// with four approaches.
const crossOSM = `<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6">
  <node id="1" lat="40.0000" lon="-74.0000"><tag k="highway" v="traffic_signals"/></node>
  <node id="2" lat="40.0020" lon="-74.0000"/>
  <node id="3" lat="39.9980" lon="-74.0000"/>
  <node id="4" lat="40.0000" lon="-73.9975"/>
  <node id="5" lat="40.0000" lon="-74.0025"/>
  <way id="10"><nd ref="2"/><nd ref="1"/><nd ref="3"/><tag k="highway" v="primary"/></way>
  <way id="11"><nd ref="4"/><nd ref="1"/><nd ref="5"/><tag k="highway" v="residential"/></way>
</osm>
`

func writeFile(t *testing.T, name, content string) string {
	t.Helper()
	path := filepath.Join(t.TempDir(), name)
	if err := os.WriteFile(path, []byte(content), 0o644); err != nil {
		t.Fatalf("write %s: %v", name, err)
	}
	return path
}

// readTrace returns every event in the trace file at path, failing the test
// on anything but a clean end of stream.
func readTrace(t *testing.T, path string) []trace.Event {
	t.Helper()
	f, err := os.Open(path)
	if err != nil {
		t.Fatalf("open trace: %v", err)
	}
	defer f.Close()
	r := trace.NewReader(f)
	var evs []trace.Event
	for {
		_, ev, err := r.Next()
		if err == io.EOF {
			return evs
		}
		if err != nil {
			t.Fatalf("read trace after %d events: %v", len(evs), err)
		}
		evs = append(evs, ev)
	}
}

// TestRunRun_RejectsBadFlags: flag values the sim can't use must be
// rejected up front. NaN used to slip past `< 0` style checks, and a
// negative headless --duration passed the `== 0` check and exited 0
// without running. The OSM path doesn't exist, so a flag that gets past
// validation shows up as a "load osm" error instead.
func TestRunRun_RejectsBadFlags(t *testing.T) {
	cases := []struct {
		args []string
		want string
	}{
		{[]string{"--spawn-rate", "NaN"}, "--spawn-rate"},
		{[]string{"--spawn-rate", "+Inf"}, "--spawn-rate"},
		{[]string{"--spawn-rate", "-1"}, "--spawn-rate"},
		{[]string{"--headless", "--duration", "-5s"}, "--duration"},
		{[]string{"--duration", "-5s"}, "--duration"},
		{[]string{"--headless"}, "--duration"},
		{[]string{"--gps-share", "NaN"}, "--gps-share"},
	}
	for _, c := range cases {
		args := append(append([]string(nil), c.args...), "does-not-exist.osm")
		err := runRun(args)
		if err == nil || !strings.Contains(err.Error(), c.want) {
			t.Errorf("runRun(%v) = %v, want an error about %s", c.args, err, c.want)
		}
	}
	// Valid edge values pass validation (and then fail on the missing file).
	for _, args := range [][]string{
		{"--spawn-rate", "0"},
		{"--headless", "--duration", "1s"},
		{"--gps-share", "0"},
	} {
		err := runRun(append(args, "does-not-exist.osm"))
		if err == nil || !strings.Contains(err.Error(), "load osm") {
			t.Errorf("runRun(%v) = %v, want it to pass validation and fail on load osm", args, err)
		}
	}
}

// TestRunRun_TraceFinalizeErrorFailsRun: a trace that can't be flushed
// or closed used to be logged while runRun returned nil, so the process
// exited 0 with a broken trace on disk.
func TestRunRun_TraceFinalizeErrorFailsRun(t *testing.T) {
	osm := writeFile(t, "cross.osm", crossOSM)
	tracePath := filepath.Join(t.TempDir(), "run.trace")

	orig := newTraceSinkFunc
	t.Cleanup(func() { newTraceSinkFunc = orig })
	newTraceSinkFunc = func(path string) (*traceSink, error) {
		ts, err := newTraceSink(path)
		if err == nil {
			ts.file.Close() // every flush/sync/close from here on fails
		}
		return ts, err
	}

	err := runRun([]string{"--headless", "--duration", "1s", "--trace", tracePath, osm})
	if err == nil || !strings.Contains(err.Error(), "trace finalize") {
		t.Fatalf("runRun = %v, want a trace finalize error", err)
	}
}

// TestRunRun_TraceRecordsPlansAndInitialState: an end-to-end trace starts
// with SimStart, then the signal's plan, then its initial phase at tick 0,
// and ends with SimEnd.
func TestRunRun_TraceRecordsPlansAndInitialState(t *testing.T) {
	osm := writeFile(t, "cross.osm", crossOSM)
	tracePath := filepath.Join(t.TempDir(), "run.trace")
	if err := runRun([]string{"--headless", "--duration", "1s", "--trace", tracePath, osm}); err != nil {
		t.Fatalf("runRun: %v", err)
	}
	evs := readTrace(t, tracePath)
	if len(evs) < 4 {
		t.Fatalf("trace has %d events, want at least SimStart, SignalPlan, SignalPhase, SimEnd", len(evs))
	}
	if _, ok := evs[0].(*trace.SimStart); !ok {
		t.Errorf("event 0 = %T, want *trace.SimStart", evs[0])
	}
	plan, ok := evs[1].(*trace.SignalPlan)
	if !ok {
		t.Fatalf("event 1 = %T, want *trace.SignalPlan", evs[1])
	}
	if plan.IntersectionID != 0 || len(plan.Phases) == 0 {
		t.Errorf("plan = %+v, want intersection 0 with phases", plan)
	}
	sawInitialPhase := false
	for _, ev := range evs {
		if p, ok := ev.(*trace.SignalPhase); ok && p.IntersectionID == 0 {
			sawInitialPhase = true
			break
		}
	}
	if !sawInitialPhase {
		t.Errorf("no SignalPhase recorded for the signal (initial state missing)")
	}
	if _, ok := evs[len(evs)-1].(*trace.SimEnd); !ok {
		t.Errorf("last event = %T, want *trace.SimEnd", evs[len(evs)-1])
	}
}

// TestTraceSink_SimEndAndDropMarker: with the queue full at shutdown,
// SimEnd must still be written (it used to go through the same
// non-blocking emit and could be dropped), and the TraceDropped marker
// must sit where the gap is, after the events queued before the drop. It
// used to be written ahead of everything still in the queue.
func TestTraceSink_SimEndAndDropMarker(t *testing.T) {
	path := filepath.Join(t.TempDir(), "sink.trace")
	f, err := os.Create(path)
	if err != nil {
		t.Fatal(err)
	}
	ts := newTraceSinkFile(f, 2)                          // drain not started: the queue fills up
	ts.emit(1, 0.05, &trace.VehicleDespawn{VehicleID: 1}) // queued
	ts.emit(2, 0.10, &trace.VehicleDespawn{VehicleID: 2}) // queued
	ts.emit(3, 0.15, &trace.VehicleDespawn{VehicleID: 3}) // dropped
	ts.emit(4, 0.20, &trace.VehicleDespawn{VehicleID: 4}) // dropped
	go func() {
		time.Sleep(50 * time.Millisecond) // finish starts against a full queue
		ts.drain()
	}()
	if err := ts.finish(5, 0.25); err != nil {
		t.Fatalf("finish: %v", err)
	}

	evs := readTrace(t, path)
	var got []string
	for _, ev := range evs {
		switch e := ev.(type) {
		case *trace.VehicleDespawn:
			got = append(got, "despawn"+string(rune('0'+e.VehicleID)))
		case *trace.TraceDropped:
			got = append(got, "dropped"+string(rune('0'+e.Count)))
		case *trace.SimEnd:
			got = append(got, "end")
		}
	}
	want := "despawn1 despawn2 dropped2 end"
	if strings.Join(got, " ") != want {
		t.Errorf("trace = %v, want %s", got, want)
	}
}

// unencodable is an event the trace writer has no encoding for.
type unencodable struct{}

func (unencodable) Kind() trace.Kind { return 200 }

// TestTraceSink_DrainSurvivesBadEvent: one event that fails to write (an
// oversize record used to be the common case) stopped the drain loop, so
// every later event, SimEnd included, was lost while the run still exited
// 0. Now the loop keeps going, marks the gap, and finish reports the error.
func TestTraceSink_DrainSurvivesBadEvent(t *testing.T) {
	path := filepath.Join(t.TempDir(), "sink.trace")
	ts, err := newTraceSink(path)
	if err != nil {
		t.Fatal(err)
	}
	ts.emit(1, 0.05, &trace.VehicleDespawn{VehicleID: 1})
	ts.emit(2, 0.10, unencodable{})
	ts.emit(3, 0.15, &trace.VehicleDespawn{VehicleID: 3})
	if err := ts.finish(4, 0.20); err == nil {
		t.Errorf("finish returned nil; a failed event write must be reported")
	}

	evs := readTrace(t, path)
	if len(evs) != 4 {
		t.Fatalf("trace has %d events (%v), want despawn, TraceDropped, despawn, SimEnd", len(evs), evs)
	}
	if d, ok := evs[1].(*trace.TraceDropped); !ok || d.Count != 1 {
		t.Errorf("event 1 = %#v, want TraceDropped{Count: 1} for the failed write", evs[1])
	}
	if d, ok := evs[2].(*trace.VehicleDespawn); !ok || d.VehicleID != 3 {
		t.Errorf("event 2 = %#v, want the despawn written after the failure", evs[2])
	}
	if _, ok := evs[3].(*trace.SimEnd); !ok {
		t.Errorf("event 3 = %T, want *trace.SimEnd", evs[3])
	}
}

// crossNet is a hand-built signalled four-way junction (intersection 0,
// approaches 0..3).
func crossNet() *network.Network {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 200}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		{ID: 2, Pos: network.Point{X: 0, Y: -200}},
		{ID: 3, Pos: network.Point{X: -200, Y: 0}},
		{ID: 4, Pos: network.Point{X: 0, Y: 0}},
	}
	var edges []network.Edge
	for i := 0; i < 4; i++ {
		edges = append(edges, network.Edge{
			ID: network.EdgeID(i), From: network.NodeID(i), To: 4, Length: 200, SpeedLimit: 13,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[i].Pos, nodes[4].Pos},
		})
	}
	return &network.Network{
		Nodes: nodes, Edges: edges,
		Intersections: []network.Intersection{{
			ID: 0, NodeID: 4, Incoming: []network.EdgeID{0, 1, 2, 3}, HasSignal: true,
		}},
	}
}

// TestLoadOverrides_ChecksPhasesAgainstIntersection: green_edges positions
// past the intersection's approaches, and approaches that no phase serves
// (they would wait at red forever), used to load silently.
func TestLoadOverrides_ChecksPhasesAgainstIntersection(t *testing.T) {
	cases := []struct {
		name, yaml, wantErr string
	}{
		{"position out of range", `
signals:
  - intersection_id: 0
    phases:
      - {green_edges: [0, 2], green_dur: 30, yellow_dur: 3}
      - {green_edges: [1, 3, 4], green_dur: 30, yellow_dur: 3}
`, "out of range"},
		{"approach never green", `
signals:
  - intersection_id: 0
    phases:
      - {green_edges: [0, 2], green_dur: 30, yellow_dur: 3}
      - {green_edges: [1], green_dur: 30, yellow_dur: 3}
`, "approach 3"},
		{"valid", `
signals:
  - intersection_id: 0
    phases:
      - {green_edges: [0, 2], green_dur: 30, yellow_dur: 3}
      - {green_edges: [1, 3], green_dur: 30, yellow_dur: 3}
`, ""},
		{"mode only inherits the default plan", `
signals:
  - intersection_id: 0
    mode: flash_a
`, ""},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			path := writeFile(t, "signals.yaml", c.yaml)
			_, err := loadOverrides(path, crossNet())
			if c.wantErr == "" {
				if err != nil {
					t.Fatalf("loadOverrides: %v", err)
				}
				return
			}
			if err == nil || !strings.Contains(err.Error(), c.wantErr) {
				t.Fatalf("loadOverrides = %v, want an error containing %q", err, c.wantErr)
			}
		})
	}
}

// TestSignalPlanEvents_UsesOverride: the recorded plan is the one the world
// runs, so a --signals override shows up in the trace.
func TestSignalPlanEvents_UsesOverride(t *testing.T) {
	net := crossNet()
	override := sim.SignalConfig{Phases: []sim.SignalPhase{
		{GreenEdges: []int{0, 1}, GreenDur: 20, YellowDur: 3},
		{GreenEdges: []int{2, 3}, GreenDur: 20, YellowDur: 3},
	}}
	w := sim.NewWorld(net, sim.NewRandomOD(net, 1, 0), map[network.IntersectionID]sim.SignalConfig{0: override})
	evs := signalPlanEvents(w)
	if len(evs) != 1 {
		t.Fatalf("got %d plans, want 1", len(evs))
	}
	plan := evs[0].(*trace.SignalPlan)
	if plan.IntersectionID != 0 || len(plan.Phases) != 2 ||
		len(plan.Phases[0]) != 2 || plan.Phases[0][0] != 0 || plan.Phases[0][1] != 1 ||
		len(plan.Phases[1]) != 2 || plan.Phases[1][0] != 2 || plan.Phases[1][1] != 3 {
		t.Errorf("plan = %+v, want phases [[0 1] [2 3]]", plan)
	}
}

// TestStepLive_HonoursDuration: --duration used to be ignored in live mode;
// the sim ran until the window was closed.
func TestStepLive_HonoursDuration(t *testing.T) {
	net := crossNet()
	w := sim.NewWorld(net, sim.NewRandomOD(net, 1, 0), nil)
	ticks := make(chan time.Time, 1000)
	for i := 0; i < cap(ticks); i++ {
		ticks <- time.Time{}
	}
	if !stepLive(context.Background(), w, time.Second, ticks) {
		t.Fatalf("stepLive returned false, want true (duration reached)")
	}
	if w.SimTime < 1.0 || w.SimTime > 1.0+sim.DefaultDt+1e-9 {
		t.Errorf("stopped at sim time %.3f, want 1.0", w.SimTime)
	}
	if len(ticks) == 0 {
		t.Errorf("stepLive consumed every tick; it should stop at the duration")
	}

	// Unbounded runs stop only on cancellation.
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if stepLive(ctx, w, 0, make(chan time.Time)) {
		t.Errorf("stepLive with a cancelled context returned true, want false")
	}
}
