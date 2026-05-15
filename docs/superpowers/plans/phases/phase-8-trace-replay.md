# Phase 8 — Trace + Replay

**Milestone:** Sim emits a binary trace file at `--trace <path>`. A separate `tracereplay` binary reads the file and plays it back in the viewer. A determinism integration test asserts byte-identical trace from two runs with the same seed.

---

### Task 8.1: Event types + binary encoding

**Files:**
- Create: `internal/trace/events.go`
- Create: `internal/trace/writer.go`
- Create: `internal/trace/reader.go`
- Create: `internal/trace/trace_test.go`

- [ ] **Step 1: Write the failing round-trip test**

Write `internal/trace/trace_test.go`:
```go
package trace

import (
	"bytes"
	"reflect"
	"testing"
)

func TestRoundTrip_AllEventKinds(t *testing.T) {
	var buf bytes.Buffer
	w := NewWriter(&buf)

	in := []Event{
		&SimStart{SeedHi: 1, SeedLo: 2, NetHash: 0xDEADBEEF},
		&VehicleSpawn{VehicleID: 100, Route: []uint32{0, 1, 2}, OriginNode: 0, DestNode: 3},
		&VehicleDespawn{VehicleID: 100},
		&SignalPhase{IntersectionID: 5, PhaseIdx: 1, IsYellow: false},
		&MetricsTick{TotalVehicles: 42, AvgSpeed: 7.5, CongestionIdx: 0.2},
		&SimEnd{Reason: "duration"},
	}
	for i, e := range in {
		if err := w.Write(uint64(i*10), float64(i)*0.5, e); err != nil {
			t.Fatalf("write %T: %v", e, err)
		}
	}
	if err := w.Close(); err != nil {
		t.Fatalf("close: %v", err)
	}

	r := NewReader(&buf)
	for i, want := range in {
		hdr, ev, err := r.Next()
		if err != nil {
			t.Fatalf("read %d: %v", i, err)
		}
		if hdr.Tick != uint64(i*10) {
			t.Errorf("event %d: tick %d, want %d", i, hdr.Tick, i*10)
		}
		if !reflect.DeepEqual(ev, want) {
			t.Errorf("event %d: got %+v, want %+v", i, ev, want)
		}
	}
	// EOF.
	_, _, err := r.Next()
	if err == nil {
		t.Errorf("want EOF, got nil")
	}
}
```

- [ ] **Step 2: Run to verify failure**

Run: `go test ./internal/trace/ -v`
Expected: FAIL — package doesn't exist.

- [ ] **Step 3: Implement events**

Write `internal/trace/events.go`:
```go
// Package trace defines a binary event stream format for sim playback.
//
// Wire format:
//   magic: "TSIM" (4 bytes)
//   version: uint16 (currently 1)
//   then a sequence of events, each:
//     tick: uint64
//     simTime: float64
//     kind: uint8
//     length: uint16 (bytes of payload that follow)
//     payload: kind-specific
//
// All integers little-endian.
package trace

type Kind uint8

const (
	KindSimStart       Kind = 1
	KindVehicleSpawn   Kind = 2
	KindVehicleDespawn Kind = 3
	KindSignalPhase    Kind = 4
	KindMetricsTick    Kind = 5
	KindSimEnd         Kind = 6
)

// Event is implemented by every concrete event type.
type Event interface {
	Kind() Kind
}

type Header struct {
	Tick    uint64
	SimTime float64
}

type SimStart struct {
	SeedHi, SeedLo uint64
	NetHash        uint64 // fingerprint of the network, for replay validation
}
func (*SimStart) Kind() Kind { return KindSimStart }

type VehicleSpawn struct {
	VehicleID  uint32
	OriginNode uint32
	DestNode   uint32
	Route      []uint32
}
func (*VehicleSpawn) Kind() Kind { return KindVehicleSpawn }

type VehicleDespawn struct {
	VehicleID uint32
}
func (*VehicleDespawn) Kind() Kind { return KindVehicleDespawn }

type SignalPhase struct {
	IntersectionID uint32
	PhaseIdx       uint8
	IsYellow       bool
}
func (*SignalPhase) Kind() Kind { return KindSignalPhase }

type MetricsTick struct {
	TotalVehicles uint32
	AvgSpeed      float64
	CongestionIdx float64
}
func (*MetricsTick) Kind() Kind { return KindMetricsTick }

type SimEnd struct {
	Reason string
}
func (*SimEnd) Kind() Kind { return KindSimEnd }
```

- [ ] **Step 4: Implement Writer**

Write `internal/trace/writer.go`:
```go
package trace

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
)

const fileMagic = "TSIM"
const fileVersion uint16 = 1

type Writer struct {
	w           io.Writer
	headerWritten bool
}

func NewWriter(w io.Writer) *Writer { return &Writer{w: w} }

func (w *Writer) writeHeader() error {
	if _, err := w.w.Write([]byte(fileMagic)); err != nil {
		return err
	}
	return binary.Write(w.w, binary.LittleEndian, fileVersion)
}

func (w *Writer) Write(tick uint64, simTime float64, e Event) error {
	if !w.headerWritten {
		if err := w.writeHeader(); err != nil {
			return err
		}
		w.headerWritten = true
	}
	var payload bytes.Buffer
	if err := encodePayload(&payload, e); err != nil {
		return err
	}
	if payload.Len() > 0xFFFF {
		return fmt.Errorf("trace event payload too large: %d bytes", payload.Len())
	}

	if err := binary.Write(w.w, binary.LittleEndian, tick); err != nil {
		return err
	}
	if err := binary.Write(w.w, binary.LittleEndian, simTime); err != nil {
		return err
	}
	if err := binary.Write(w.w, binary.LittleEndian, uint8(e.Kind())); err != nil {
		return err
	}
	if err := binary.Write(w.w, binary.LittleEndian, uint16(payload.Len())); err != nil {
		return err
	}
	_, err := w.w.Write(payload.Bytes())
	return err
}

func (w *Writer) Close() error {
	if !w.headerWritten {
		return w.writeHeader()
	}
	return nil
}

func encodePayload(b *bytes.Buffer, e Event) error {
	le := binary.LittleEndian
	switch ev := e.(type) {
	case *SimStart:
		if err := binary.Write(b, le, ev.SeedHi); err != nil { return err }
		if err := binary.Write(b, le, ev.SeedLo); err != nil { return err }
		return binary.Write(b, le, ev.NetHash)
	case *VehicleSpawn:
		if err := binary.Write(b, le, ev.VehicleID); err != nil { return err }
		if err := binary.Write(b, le, ev.OriginNode); err != nil { return err }
		if err := binary.Write(b, le, ev.DestNode); err != nil { return err }
		if err := binary.Write(b, le, uint16(len(ev.Route))); err != nil { return err }
		for _, eid := range ev.Route {
			if err := binary.Write(b, le, eid); err != nil { return err }
		}
		return nil
	case *VehicleDespawn:
		return binary.Write(b, le, ev.VehicleID)
	case *SignalPhase:
		if err := binary.Write(b, le, ev.IntersectionID); err != nil { return err }
		if err := binary.Write(b, le, ev.PhaseIdx); err != nil { return err }
		var y uint8
		if ev.IsYellow { y = 1 }
		return binary.Write(b, le, y)
	case *MetricsTick:
		if err := binary.Write(b, le, ev.TotalVehicles); err != nil { return err }
		if err := binary.Write(b, le, ev.AvgSpeed); err != nil { return err }
		return binary.Write(b, le, ev.CongestionIdx)
	case *SimEnd:
		if err := binary.Write(b, le, uint16(len(ev.Reason))); err != nil { return err }
		_, err := b.WriteString(ev.Reason)
		return err
	default:
		return fmt.Errorf("unknown event kind: %T", e)
	}
}
```

- [ ] **Step 5: Implement Reader**

Write `internal/trace/reader.go`:
```go
package trace

import (
	"encoding/binary"
	"fmt"
	"io"
)

type Reader struct {
	r            io.Reader
	headerParsed bool
}

func NewReader(r io.Reader) *Reader { return &Reader{r: r} }

func (r *Reader) readHeader() error {
	magic := make([]byte, 4)
	if _, err := io.ReadFull(r.r, magic); err != nil {
		return fmt.Errorf("read magic: %w", err)
	}
	if string(magic) != fileMagic {
		return fmt.Errorf("not a trace file: bad magic %q", magic)
	}
	var v uint16
	if err := binary.Read(r.r, binary.LittleEndian, &v); err != nil {
		return fmt.Errorf("read version: %w", err)
	}
	if v != fileVersion {
		return fmt.Errorf("unsupported trace version: %d (want %d)", v, fileVersion)
	}
	return nil
}

func (r *Reader) Next() (Header, Event, error) {
	if !r.headerParsed {
		if err := r.readHeader(); err != nil {
			return Header{}, nil, err
		}
		r.headerParsed = true
	}
	var hdr Header
	if err := binary.Read(r.r, binary.LittleEndian, &hdr.Tick); err != nil {
		return Header{}, nil, err
	}
	if err := binary.Read(r.r, binary.LittleEndian, &hdr.SimTime); err != nil {
		return Header{}, nil, err
	}
	var kind uint8
	if err := binary.Read(r.r, binary.LittleEndian, &kind); err != nil {
		return Header{}, nil, err
	}
	var plen uint16
	if err := binary.Read(r.r, binary.LittleEndian, &plen); err != nil {
		return Header{}, nil, err
	}
	payload := make([]byte, plen)
	if _, err := io.ReadFull(r.r, payload); err != nil {
		return Header{}, nil, err
	}
	ev, err := decodePayload(Kind(kind), payload)
	return hdr, ev, err
}

func decodePayload(k Kind, p []byte) (Event, error) {
	rd := &byteReader{data: p}
	le := binary.LittleEndian
	switch k {
	case KindSimStart:
		e := &SimStart{}
		_ = binary.Read(rd, le, &e.SeedHi)
		_ = binary.Read(rd, le, &e.SeedLo)
		_ = binary.Read(rd, le, &e.NetHash)
		return e, nil
	case KindVehicleSpawn:
		e := &VehicleSpawn{}
		_ = binary.Read(rd, le, &e.VehicleID)
		_ = binary.Read(rd, le, &e.OriginNode)
		_ = binary.Read(rd, le, &e.DestNode)
		var n uint16
		_ = binary.Read(rd, le, &n)
		e.Route = make([]uint32, n)
		for i := range e.Route {
			_ = binary.Read(rd, le, &e.Route[i])
		}
		return e, nil
	case KindVehicleDespawn:
		e := &VehicleDespawn{}
		_ = binary.Read(rd, le, &e.VehicleID)
		return e, nil
	case KindSignalPhase:
		e := &SignalPhase{}
		_ = binary.Read(rd, le, &e.IntersectionID)
		_ = binary.Read(rd, le, &e.PhaseIdx)
		var y uint8
		_ = binary.Read(rd, le, &y)
		e.IsYellow = y != 0
		return e, nil
	case KindMetricsTick:
		e := &MetricsTick{}
		_ = binary.Read(rd, le, &e.TotalVehicles)
		_ = binary.Read(rd, le, &e.AvgSpeed)
		_ = binary.Read(rd, le, &e.CongestionIdx)
		return e, nil
	case KindSimEnd:
		e := &SimEnd{}
		var n uint16
		_ = binary.Read(rd, le, &n)
		buf := make([]byte, n)
		_, _ = rd.Read(buf)
		e.Reason = string(buf)
		return e, nil
	}
	return nil, fmt.Errorf("unknown event kind: %d", k)
}

// byteReader is a tiny in-memory io.Reader over a []byte.
type byteReader struct {
	data []byte
	pos  int
}

func (r *byteReader) Read(p []byte) (int, error) {
	if r.pos >= len(r.data) {
		return 0, io.EOF
	}
	n := copy(p, r.data[r.pos:])
	r.pos += n
	return n, nil
}
```

- [ ] **Step 6: Run tests**

Run: `go test ./internal/trace/ -v`
Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add internal/trace/
git commit -m "feat(trace): binary event stream with writer and reader"
```

---

### Task 8.2: Sim emits trace events

**Files:**
- Modify: `internal/sim/world.go`

- [ ] **Step 1: Add TraceSink interface**

Read `internal/sim/world.go`.

Add to `internal/sim/world.go`:
```go
// TraceSink consumes trace events. The sim guarantees never to block on
// the sink (drops are logged). The sim package defines the interface to
// avoid an import cycle with the trace package.
type TraceSink interface {
	Send(tick uint64, simTime float64, kind uint8, payload any) // best-effort
}

// nopSink discards events.
type nopSink struct{}

func (nopSink) Send(uint64, float64, uint8, any) {}
```

Wait — better approach: use the `trace.Event` interface directly via a function callback to avoid the abstraction. Actually the cleanest: put the event types in `internal/trace` and accept a `func(tick uint64, simTime float64, e trace.Event)` callback. Use that.

Replace the snippet above with:
```go
// In world.go, top of file imports add:
//   "github.com/lab1702/traffic-sim/internal/trace"

// In World struct add:
//   EmitTrace func(tick uint64, simTime float64, e trace.Event)
```

In `NewWorld`, default `EmitTrace` to a no-op: `EmitTrace: func(uint64, float64, trace.Event) {}`.

- [ ] **Step 2: Emit spawn/despawn/signal events**

In `trySpawn`, after appending the new vehicle:
```go
route32 := make([]uint32, len(v.Route))
for i, eid := range v.Route { route32[i] = uint32(eid) }
w.EmitTrace(w.Tick, w.SimTime, &trace.VehicleSpawn{
	VehicleID: uint32(v.ID),
	OriginNode: uint32(r.OriginNode),
	DestNode: uint32(r.DestNode),
	Route: route32,
})
```

In the vehicle loop, when marking despawned, emit `VehicleDespawn`. Easiest place: after `stepIDM`, if `v.Despawned`, emit. Replace the inside of the integration loop's tail:
```go
stepIDM(v, lS, lV, has, w.Net, DefaultIDM(), w.dt)
if v.Despawned {
	w.EmitTrace(w.Tick, w.SimTime, &trace.VehicleDespawn{VehicleID: uint32(v.ID)})
}
```

For signal phase changes, modify `SignalState.Advance` to return a bool indicating a phase change (or just emit from world on detection). Simplest: track previous phase per state and emit on change. Add this loop after the `Advance` calls in `Step()`:
```go
for i, s := range w.SignalStates {
	if s == nil {
		continue
	}
	// Compare to last known phase recorded in this struct.
	if w.lastPhase == nil {
		w.lastPhase = make([]signalLast, len(w.SignalStates))
	}
	cur := signalLast{idx: s.PhaseIdx, yellow: s.IsYellow}
	if w.lastPhase[i] != cur {
		w.lastPhase[i] = cur
		w.EmitTrace(w.Tick, w.SimTime, &trace.SignalPhase{
			IntersectionID: uint32(i),
			PhaseIdx:       uint8(s.PhaseIdx),
			IsYellow:       s.IsYellow,
		})
	}
}
```
Add the supporting types to world.go:
```go
type signalLast struct {
	idx    int
	yellow bool
}
// And in World struct:
lastPhase []signalLast
```

- [ ] **Step 3: Verify build and tests pass**

Run: `go test ./...`
Expected: green.

- [ ] **Step 4: Commit**

```bash
git add internal/sim/world.go
git commit -m "feat(sim): emit trace events for spawn/despawn/signal phase"
```

---

### Task 8.3: CLI wires trace writer

**Files:**
- Modify: `cmd/trafficsim/main.go`

- [ ] **Step 1: Add --trace flag and writer goroutine**

In `runRun`:
```go
tracePath := fs.String("trace", "", "write trace events to this file")
```

After building `w := sim.NewWorld(...)`:
```go
if *tracePath != "" {
	f, err := os.Create(*tracePath)
	if err != nil {
		slog.Error("trace create failed", "err", err)
		os.Exit(1)
	}
	defer f.Close()
	tw := trace.NewWriter(f)
	// Buffered channel; drops are logged.
	type evMsg struct {
		tick uint64; simTime float64; e trace.Event
	}
	ch := make(chan evMsg, 4096)
	go func() {
		for m := range ch {
			if err := tw.Write(m.tick, m.simTime, m.e); err != nil {
				slog.Error("trace write failed", "err", err)
				return
			}
		}
		tw.Close()
	}()
	dropped := uint64(0)
	w.EmitTrace = func(tick uint64, simTime float64, e trace.Event) {
		select {
		case ch <- evMsg{tick, simTime, e}:
		default:
			dropped++
			if dropped%1000 == 1 {
				slog.Warn("trace dropped", "dropped_total", dropped)
			}
		}
	}
	// Emit start event with the seed.
	w.EmitTrace(0, 0, &trace.SimStart{SeedLo: *seed, NetHash: 0})
	defer func() {
		w.EmitTrace(w.Tick, w.SimTime, &trace.SimEnd{Reason: "exit"})
		close(ch)
	}()
}
```

Add `"github.com/lab1702/traffic-sim/internal/trace"` to imports.

- [ ] **Step 2: Build and smoke test**

Run:
```powershell
go build ./cmd/trafficsim/
.\trafficsim.exe run .\internal\osmload\testdata\tiny.osm --headless --duration 5s --spawn-rate 2 --trace out.trace
```
Expected: completes, `out.trace` exists with non-zero size.

- [ ] **Step 3: Commit**

```bash
git add cmd/trafficsim/main.go
git commit -m "feat(cli): --trace writes binary event stream during sim"
```

---

### Task 8.4: tracereplay binary

**Files:**
- Create: `cmd/tracereplay/main.go`

- [ ] **Step 1: Write the replay viewer**

Write `cmd/tracereplay/main.go`:
```go
// Command tracereplay reads a trace file and plays it back in the viewer.
//
// In this initial form, it reconstructs vehicle positions by replaying
// the simulation deterministically from the seed encoded in the trace's
// SimStart event, applying spawn/despawn events at their recorded ticks.
//
// Required: the same OSM file used for the original run.
package main

import (
	"flag"
	"fmt"
	"log/slog"
	"os"

	"github.com/hajimehoshi/ebiten/v2"

	"github.com/lab1702/traffic-sim/internal/netbuild"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/lab1702/traffic-sim/internal/render"
	"github.com/lab1702/traffic-sim/internal/snapshot"
	"github.com/lab1702/traffic-sim/internal/trace"
)

func main() {
	fs := flag.NewFlagSet("tracereplay", flag.ExitOnError)
	osmPath := fs.String("osm", "", "path to OSM file used for the original run")
	tracePath := fs.String("trace", "", "path to trace file")
	if err := fs.Parse(os.Args[1:]); err != nil {
		os.Exit(2)
	}
	if *osmPath == "" || *tracePath == "" {
		fmt.Fprintln(os.Stderr, "usage: tracereplay -osm <file> -trace <file>")
		os.Exit(2)
	}

	feat, err := osmload.Load(*osmPath)
	if err != nil { slog.Error("load osm", "err", err); os.Exit(1) }
	net, _, err := netbuild.Build(feat)
	if err != nil { slog.Error("build", "err", err); os.Exit(1) }

	tf, err := os.Open(*tracePath)
	if err != nil { slog.Error("open trace", "err", err); os.Exit(1) }
	defer tf.Close()

	buf := snapshot.New()
	player := newPlayer(net, trace.NewReader(tf), buf)
	go player.run()

	vp := render.NewViewport(net, buf, 1280, 800)
	ebiten.SetWindowSize(1280, 800)
	ebiten.SetWindowTitle("tracereplay")
	if err := ebiten.RunGame(&gameAdapter{vp: vp}); err != nil {
		slog.Error("ebiten", "err", err)
	}
}

type gameAdapter struct{ vp *render.Viewport }

func (g *gameAdapter) Update() error              { return g.vp.Update() }
func (g *gameAdapter) Draw(s *ebiten.Image)       { g.vp.Draw(s) }
func (g *gameAdapter) Layout(w, h int) (int, int) { return g.vp.Layout(w, h) }
```

Also create `cmd/tracereplay/player.go`:
```go
package main

import (
	"errors"
	"io"
	"math"
	"time"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
	"github.com/lab1702/traffic-sim/internal/trace"
)

// player advances trace events at real wall-clock speed (1x), reconstructing
// vehicle positions by simple kinematic extrapolation between events.
// Phase 8 keeps this simple: vehicles teleport at spawn and disappear at
// despawn; positions between are interpolated linearly along their route
// at the edge's speed limit. Phase 9 can extend this with state snapshots
// for faithful replay.
type player struct {
	net    *network.Network
	r      *trace.Reader
	buf    *snapshot.Buffer
	vehicles map[uint32]*replayVehicle
	signals  []snapshot.SignalView
}

type replayVehicle struct {
	route       []uint32
	routeIdx    int
	curEdge     network.EdgeID
	s           float64
	enteredEdgeAt float64
}

func newPlayer(net *network.Network, r *trace.Reader, buf *snapshot.Buffer) *player {
	sigs := make([]snapshot.SignalView, 0)
	for i := range net.Intersections {
		x := &net.Intersections[i]
		if !x.HasSignal { continue }
		node := net.Nodes[x.NodeID]
		sigs = append(sigs, snapshot.SignalView{
			IntersectionID: uint32(x.ID), X: node.Pos.X, Y: node.Pos.Y,
		})
	}
	return &player{
		net: net, r: r, buf: buf,
		vehicles: make(map[uint32]*replayVehicle),
		signals: sigs,
	}
}

func (p *player) run() {
	start := time.Now()
	for {
		hdr, ev, err := p.r.Next()
		if err != nil {
			if !errors.Is(err, io.EOF) {
				// Log once and exit.
			}
			return
		}
		// Real-time pacing.
		want := time.Duration(hdr.SimTime * float64(time.Second))
		elapsed := time.Since(start)
		if want > elapsed {
			time.Sleep(want - elapsed)
		}
		p.apply(hdr, ev)
		p.publish(hdr.SimTime)
	}
}

func (p *player) apply(hdr trace.Header, ev trace.Event) {
	switch e := ev.(type) {
	case *trace.VehicleSpawn:
		p.vehicles[e.VehicleID] = &replayVehicle{
			route:    e.Route,
			curEdge:  network.EdgeID(e.Route[0]),
			enteredEdgeAt: hdr.SimTime,
		}
	case *trace.VehicleDespawn:
		delete(p.vehicles, e.VehicleID)
	case *trace.SignalPhase:
		for i := range p.signals {
			if p.signals[i].IntersectionID == e.IntersectionID {
				// We don't know red vs green precisely; approximate:
				// phase idx 0 = green for some, IsYellow always honored.
				p.signals[i].IsYellow = e.IsYellow
				p.signals[i].IsRed = !e.IsYellow && e.PhaseIdx%2 == 1
			}
		}
	}
}

func (p *player) publish(simTime float64) {
	views := make([]snapshot.VehicleView, 0, len(p.vehicles))
	for id, v := range p.vehicles {
		// Advance position by elapsed time on current edge at speed limit.
		edge := &p.net.Edges[v.curEdge]
		dt := simTime - v.enteredEdgeAt
		s := edge.SpeedLimit * dt
		for s >= edge.Length && v.routeIdx+1 < len(v.route) {
			s -= edge.Length
			v.routeIdx++
			v.curEdge = network.EdgeID(v.route[v.routeIdx])
			v.enteredEdgeAt += edge.Length / edge.SpeedLimit
			edge = &p.net.Edges[v.curEdge]
		}
		v.s = s
		x, y, hd := positionOnEdge(p.net, v.curEdge, v.s)
		views = append(views, snapshot.VehicleView{
			ID: id, X: x, Y: y, Heading: hd, Speed: edge.SpeedLimit,
		})
	}
	p.buf.Publish(snapshot.Snapshot{
		SimTime: simTime, Vehicles: views, Signals: p.signals, Bounds: p.net.Bounds,
	})
}

func positionOnEdge(net *network.Network, eid network.EdgeID, s float64) (float64, float64, float64) {
	e := &net.Edges[eid]
	g := e.Geometry
	if len(g) < 2 { return 0, 0, 0 }
	remaining := s
	for i := 1; i < len(g); i++ {
		dx := g[i].X - g[i-1].X
		dy := g[i].Y - g[i-1].Y
		segLen := math.Sqrt(dx*dx + dy*dy)
		if remaining <= segLen || i == len(g)-1 {
			t := 0.0
			if segLen > 0 { t = remaining / segLen }
			if t > 1 { t = 1 }
			return g[i-1].X + dx*t, g[i-1].Y + dy*t, math.Atan2(dy, dx)
		}
		remaining -= segLen
	}
	return g[len(g)-1].X, g[len(g)-1].Y, 0
}
```

- [ ] **Step 2: Build and smoke test**

```powershell
go build ./cmd/tracereplay/
.\tracereplay.exe -osm .\internal\osmload\testdata\tiny.osm -trace out.trace
```
Expected: window opens with vehicles moving (sparsely — tiny fixture). Closing the window ends the program.

- [ ] **Step 3: Commit**

```bash
git add cmd/tracereplay/
git commit -m "feat(tracereplay): replay binary plays trace files in viewer"
```

---

### Task 8.5: Determinism integration test

**Files:**
- Modify: `internal/sim/world_test.go`

- [ ] **Step 1: Add the test**

Append to `internal/sim/world_test.go`:
```go
import (
	// add to existing imports:
	"bytes"
	"github.com/lab1702/traffic-sim/internal/trace"
)

func TestWorld_TraceDeterminism(t *testing.T) {
	run := func() []byte {
		net := build2x2Grid()
		w := NewWorld(net, NewRandomOD(net, 9001, 5.0), nil)
		var buf bytes.Buffer
		tw := trace.NewWriter(&buf)
		w.EmitTrace = func(tick uint64, simTime float64, e trace.Event) {
			_ = tw.Write(tick, simTime, e)
		}
		w.Run(3.0)
		_ = tw.Close()
		return buf.Bytes()
	}
	a, b := run(), run()
	if !bytes.Equal(a, b) {
		t.Fatalf("trace bytes differ across runs with same seed (len %d vs %d)", len(a), len(b))
	}
}
```

(NOTE: combine imports correctly — the existing imports block in the file needs to be updated rather than duplicated.)

- [ ] **Step 2: Run**

Run: `go test ./internal/sim/ -run TestWorld_TraceDeterminism -v`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add internal/sim/world_test.go
git commit -m "test(sim): byte-identical trace from same seed (determinism gate)"
```

---

**Phase 8 done when:**
- `go test ./...` green, including determinism test.
- `trafficsim run ... --trace out.trace` writes a file.
- `tracereplay -osm ... -trace ...` plays it back.
