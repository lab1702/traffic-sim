package main

import (
	"bufio"
	"context"
	"errors"
	"flag"
	"fmt"
	"log/slog"
	"os"
	"os/signal"
	"sync/atomic"
	"syscall"
	"time"

	"github.com/hajimehoshi/ebiten/v2"

	"github.com/lab1702/traffic-sim/internal/config"
	"github.com/lab1702/traffic-sim/internal/netbuild"
	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/lab1702/traffic-sim/internal/render"
	"github.com/lab1702/traffic-sim/internal/sim"
	"github.com/lab1702/traffic-sim/internal/trace"
)

func main() {
	if len(os.Args) < 2 {
		usage()
		os.Exit(2)
	}
	switch os.Args[1] {
	case "load":
		runLoad(os.Args[2:])
	case "run":
		if err := runRun(os.Args[2:]); err != nil {
			slog.Error("run failed", "err", err)
			os.Exit(1)
		}
	case "-h", "--help", "help":
		usage()
		os.Exit(0)
	default:
		usage()
		os.Exit(2)
	}
}

// usage prints the top-level help, including every flag for every
// subcommand. We rebuild each subcommand's FlagSet just to harvest its
// PrintDefaults output, so this can never drift from the real flags.
func usage() {
	out := os.Stderr
	fmt.Fprintln(out, "Usage: trafficsim <subcommand> [flags] <path-to-osm>")
	fmt.Fprintln(out, "")
	fmt.Fprintln(out, "A Go-based traffic simulator that reads OpenStreetMap files and runs a")
	fmt.Fprintln(out, "city-scale microsimulation with a live viewer and optional trace output.")
	fmt.Fprintln(out, "")
	fmt.Fprintln(out, "Subcommands:")
	fmt.Fprintln(out, "  load   Parse an OSM file and print graph stats (no simulation).")
	fmt.Fprintln(out, "  run    Run the simulation, with or without the live viewer.")
	fmt.Fprintln(out, "")

	loadFS, _ := newLoadFlagSet()
	if hasFlags(loadFS) {
		fmt.Fprintln(out, "Flags for 'load':")
		loadFS.SetOutput(out)
		loadFS.PrintDefaults()
		fmt.Fprintln(out, "")
	}

	fmt.Fprintln(out, "Flags for 'run':")
	runFS, _ := newRunFlagSet()
	runFS.SetOutput(out)
	runFS.PrintDefaults()
	fmt.Fprintln(out, "")

	fmt.Fprintln(out, "Notes:")
	fmt.Fprintln(out, "  - Flags must appear BEFORE the OSM path (Go flag-parser stops at the")
	fmt.Fprintln(out, "    first non-flag argument).")
	fmt.Fprintln(out, "  - `run --headless` requires `--duration > 0`.")
	fmt.Fprintln(out, "  - Same `--seed` + same OSM + same `--spawn-rate` yields a byte-identical trace.")
	fmt.Fprintln(out, "  - Ctrl+C (SIGINT/SIGTERM) triggers an orderly shutdown: the trace is")
	fmt.Fprintln(out, "    flushed with a final SimEnd event before the process exits.")
	fmt.Fprintln(out, "")
	fmt.Fprintln(out, "Examples:")
	fmt.Fprintln(out, "  trafficsim load city.osm.pbf")
	fmt.Fprintln(out, "  trafficsim run --spawn-rate 20 city.osm.pbf")
	fmt.Fprintln(out, "  trafficsim run --headless --duration 5m --trace run.trace city.osm.pbf")
	fmt.Fprintln(out, "")
	fmt.Fprintln(out, "Window controls (live mode): left-mouse drag to pan, wheel to zoom,")
	fmt.Fprintln(out, "drag edges/corners to resize.")
}

// loadFlags is empty today but exists for symmetry with runFlags; flag
// additions to the `load` subcommand happen here.
type loadFlags struct{}

func newLoadFlagSet() (*flag.FlagSet, *loadFlags) {
	fs := flag.NewFlagSet("load", flag.ExitOnError)
	f := &loadFlags{}
	fs.Usage = func() {
		fmt.Fprintln(fs.Output(), "Usage: trafficsim load <path-to-osm>")
		fmt.Fprintln(fs.Output(), "")
		fmt.Fprintln(fs.Output(), "Parse an OSM file (.osm or .osm.pbf) and print graph build stats.")
		fmt.Fprintln(fs.Output(), "No simulation is run.")
		fmt.Fprintln(fs.Output(), "")
		fmt.Fprintln(fs.Output(), "Flags:")
		fs.PrintDefaults()
	}
	return fs, f
}

func runLoad(args []string) {
	fs, _ := newLoadFlagSet()
	if err := fs.Parse(args); err != nil {
		os.Exit(2)
	}
	if fs.NArg() != 1 {
		fmt.Fprintln(os.Stderr, "load: need exactly one OSM path")
		fs.Usage()
		os.Exit(2)
	}
	path := fs.Arg(0)

	feat, err := osmload.Load(path)
	if err != nil {
		slog.Error("load failed", "err", err)
		os.Exit(1)
	}
	net, rpt, err := netbuild.Build(feat)
	if err != nil {
		slog.Error("build failed", "err", err)
		os.Exit(1)
	}
	fmt.Printf("nodes=%d edges=%d intersections=%d signals=%d\n",
		len(net.Nodes), len(net.Edges), len(net.Intersections), countSignals(net.Intersections))
	fmt.Printf("ways_skipped=%d components_dropped=%d\n",
		rpt.WaysSkipped, rpt.ComponentsDropped)
	fmt.Printf("turn_restrictions: applied=%d skipped=%d total_intersection_bans=%d\n",
		rpt.RestrictionsApplied, rpt.RestrictionsSkipped, countBannedTurns(net.Intersections))
	fmt.Printf("bounds=(%.1f,%.1f)-(%.1f,%.1f) m\n",
		net.Bounds.MinX, net.Bounds.MinY, net.Bounds.MaxX, net.Bounds.MaxY)
}

func countBannedTurns(xs []network.Intersection) int {
	n := 0
	for _, x := range xs {
		n += len(x.BannedTurns)
	}
	return n
}

type runFlags struct {
	headless    bool
	duration    time.Duration
	seed        uint64
	spawnRate   float64
	signalsPath string
	tracePath   string
	gpsShare    float64
}

func newRunFlagSet() (*flag.FlagSet, *runFlags) {
	fs := flag.NewFlagSet("run", flag.ExitOnError)
	f := &runFlags{}
	fs.BoolVar(&f.headless, "headless", false, "skip rendering, run sim only (requires --duration > 0)")
	fs.DurationVar(&f.duration, "duration", 0, "stop after this much sim time, e.g. 30s or 5m (0 = unbounded; required when --headless)")
	fs.Uint64Var(&f.seed, "seed", 1, "RNG seed; same seed + same OSM + same --spawn-rate gives a byte-identical trace")
	fs.Float64Var(&f.spawnRate, "spawn-rate", 5.0, "vehicles attempted per simulated second")
	fs.StringVar(&f.signalsPath, "signals", "", "path to a YAML file of per-intersection signal overrides")
	fs.StringVar(&f.tracePath, "trace", "", "write binary trace events to this file for replay/analysis")
	fs.Float64Var(&f.gpsShare, "gps-share", 1.0,
		"fraction of vehicles (0..1) with GPS rerouting around congestion")
	fs.Usage = func() {
		fmt.Fprintln(fs.Output(), "Usage: trafficsim run [flags] <path-to-osm>")
		fmt.Fprintln(fs.Output(), "")
		fmt.Fprintln(fs.Output(), "Run the simulation. Without --headless, opens a live viewer window.")
		fmt.Fprintln(fs.Output(), "Flags must appear BEFORE the OSM path.")
		fmt.Fprintln(fs.Output(), "")
		fmt.Fprintln(fs.Output(), "Flags:")
		fs.PrintDefaults()
	}
	return fs, f
}

// runRun parses flags, loads the OSM/config, builds the network and world,
// and dispatches to headless or live mode. Returns an error rather than
// calling os.Exit so deferreds (trace finalization) get a chance to run.
func runRun(args []string) error {
	fs, f := newRunFlagSet()
	if err := fs.Parse(args); err != nil {
		return err
	}
	if fs.NArg() != 1 {
		fs.Usage()
		return errors.New("run: need exactly one OSM path")
	}
	if f.headless && f.duration == 0 {
		return errors.New("--headless requires --duration > 0")
	}
	if f.gpsShare < 0 || f.gpsShare > 1 {
		return fmt.Errorf("--gps-share must be in [0,1], got %v", f.gpsShare)
	}
	osmPath := fs.Arg(0)

	feat, err := osmload.Load(osmPath)
	if err != nil {
		return fmt.Errorf("load osm: %w", err)
	}
	net, _, err := netbuild.Build(feat)
	if err != nil {
		return fmt.Errorf("build: %w", err)
	}

	overrides, err := loadOverrides(f.signalsPath, net)
	if err != nil {
		return err
	}

	spawner := sim.NewRandomOD(net, f.seed, f.spawnRate)
	w := sim.NewWorld(net, spawner, overrides)
	w.GpsShare = f.gpsShare

	// Control channel from the UI to the sim. Renderer pushes mode-toggle
	// events here; sim drains at the top of each Step. Buffered so a
	// pause in the sim goroutine doesn't block the UI.
	controlCh := make(chan sim.ControlEvent, 32)
	w.Control = controlCh

	// Incident channel from the UI to the sim, mirroring Control. Shift+click
	// on an edge in the viewer cycles its incident severity.
	incidentCh := make(chan sim.IncidentEvent, 32)
	w.IncidentControl = incidentCh

	// SIGINT/SIGTERM → ctx cancellation. The context is the single
	// orderly-shutdown signal: both headless and live modes watch it.
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	// Trace setup. The sink's Close is deferred so it runs after the sim
	// goroutine has stopped (see runLive for the wait sequence).
	var sink *traceSink
	if f.tracePath != "" {
		sink, err = newTraceSink(f.tracePath)
		if err != nil {
			return fmt.Errorf("trace create: %w", err)
		}
		defer func() {
			// Emit SimEnd at the final tick/simTime the sim reached, then
			// flush+fsync+close the file. Reading w.Tick/w.SimTime here is
			// race-free because the sim goroutine has already returned
			// (runHeadless and runLive both wait for it before returning).
			sink.emit(w.Tick, w.SimTime, &trace.SimEnd{Reason: "exit"})
			if cerr := sink.close(); cerr != nil {
				slog.Error("trace finalize failed", "err", cerr)
			}
		}()
		w.EmitTrace = sink.emit
		// SimStart includes a network fingerprint so tracereplay can warn
		// if the OSM doesn't match the original run.
		sink.emit(0, 0, &trace.SimStart{
			SeedLo:  f.seed,
			SeedHi:  f.seed ^ 0x9E3779B97F4A7C15, // matches RandomOD's PCG seed pair
			NetHash: network.Hash(net),
		})
	}

	if f.headless {
		runHeadless(ctx, w, f.duration)
		fmt.Printf("done. final_vehicles=%d ticks=%d sim_time=%.2fs\n",
			len(w.Vehicles), w.Tick, w.SimTime)
		return nil
	}
	return runLive(ctx, w, controlCh, incidentCh, net)
}

// loadOverrides reads a signals.yaml at signalsPath (if non-empty) and
// expands its contents into the maps the sim and network consume. Returns
// (nil, nil) if no path was provided. Returns a wrapped error on any
// parse, validate, or missing-file failure.
func loadOverrides(signalsPath string, net *network.Network) (map[network.IntersectionID]sim.SignalConfig, error) {
	if signalsPath == "" {
		return nil, nil
	}
	cfg, err := config.LoadConfig(signalsPath)
	if err != nil {
		return nil, fmt.Errorf("signals: %w", err)
	}
	overrides := map[network.IntersectionID]sim.SignalConfig{}
	for _, o := range cfg.Signals {
		if int(o.IntersectionID) >= len(net.Intersections) {
			slog.Warn("signal override references unknown intersection",
				"id", o.IntersectionID, "count", len(net.Intersections))
			continue
		}
		x := &net.Intersections[o.IntersectionID]
		if !x.HasSignal {
			slog.Warn("signal override targets an unsignalled intersection; ignored",
				"id", o.IntersectionID, "node_id", x.NodeID)
			continue
		}
		mode, ok := sim.ParseSignalMode(o.Mode)
		if !ok {
			slog.Warn("signal override has unknown mode; using normal",
				"id", o.IntersectionID, "mode", o.Mode)
		}
		phases := make([]sim.SignalPhase, len(o.Phases))
		for i, p := range o.Phases {
			phases[i] = sim.SignalPhase{
				GreenEdges: p.GreenEdges, GreenDur: p.GreenDur, YellowDur: p.YellowDur,
			}
		}
		sc := sim.SignalConfig{
			IntersectionID: network.IntersectionID(o.IntersectionID),
			Phases:         phases,
			InitialMode:    mode,
		}
		// If only `mode:` is set (no phases), inherit the auto-generated
		// plan for that intersection so phase 0/1 groupings still work
		// for flash-mode semantics.
		if len(phases) == 0 {
			sc.Phases = sim.DefaultSignalConfig(x.Incoming, net).Phases
		}
		overrides[network.IntersectionID(o.IntersectionID)] = sc
	}
	// Apply turn restrictions to the network before constructing the
	// Router (which caches the banned-turn map at construction time).
	applyTurnRestrictions(net, cfg.TurnRestrictions)
	return overrides, nil
}

// runHeadless ticks the world in-place until duration elapses or ctx is
// cancelled (SIGINT/SIGTERM). Caller is responsible for emitting SimEnd
// after this returns.
func runHeadless(ctx context.Context, w *sim.World, duration time.Duration) {
	target := w.SimTime + duration.Seconds()
	lastLog := w.SimTime
	for w.SimTime < target {
		if ctx.Err() != nil {
			slog.Info("shutdown signal received; finalizing trace")
			return
		}
		w.Step()
		if w.SimTime-lastLog >= 1.0 {
			slog.Info("sim progress",
				"sim_time", w.SimTime,
				"vehicles", len(w.Vehicles),
				"tick", w.Tick,
			)
			lastLog = w.SimTime
		}
	}
}

// runLive runs the sim goroutine + ebiten viewer concurrently. Shutdown
// sequence: SIGINT cancellation OR ebiten-window close triggers the sim
// goroutine to return; we wait for it before returning so the deferred
// trace finalizer in runRun reads a quiescent w.Tick/w.SimTime.
func runLive(parentCtx context.Context, w *sim.World, controlCh chan<- sim.ControlEvent, incidentCh chan<- sim.IncidentEvent, net *network.Network) error {
	// Derive a child context we can cancel ourselves once ebiten exits,
	// so the sim goroutine stops cleanly even if SIGINT never fires.
	ctx, cancel := context.WithCancel(parentCtx)
	defer cancel()

	simDone := make(chan struct{})
	go func() {
		defer close(simDone)
		ticker := time.NewTicker(time.Duration(sim.DefaultDt * float64(time.Second)))
		defer ticker.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-ticker.C:
				w.Step()
			}
		}
	}()

	vp := render.NewViewport(net, w.SnapshotBuf, 1280, 800)
	vp.OnSetMode = func(intersectionID uint32, mode uint8) {
		select {
		case controlCh <- sim.ControlEvent{
			IntersectionID: network.IntersectionID(intersectionID),
			Mode:           sim.SignalMode(mode),
		}:
		default:
			slog.Warn("control channel full; dropping signal mode change")
		}
	}
	vp.OnIncident = func(edgeID uint32, severity uint8) {
		select {
		case incidentCh <- sim.IncidentEvent{
			EdgeID:   network.EdgeID(edgeID),
			Severity: sim.Severity(severity),
		}:
		default:
			slog.Warn("incident channel full; dropping incident change")
		}
	}
	ebiten.SetWindowSize(1280, 800)
	ebiten.SetWindowTitle("traffic-sim")
	ebiten.SetWindowResizingMode(ebiten.WindowResizingModeEnabled)

	// If SIGINT fires while ebiten is running, the gameAdapter's Update
	// returns ebiten.Termination so RunGame returns cleanly. The window
	// is destroyed and shutdown continues via the normal path.
	runErr := ebiten.RunGame(&gameAdapter{vp: vp, ctx: ctx})

	// Stop the sim goroutine and wait for it to exit before returning.
	// This wait is the precondition for the deferred trace finalizer in
	// runRun to safely read w.Tick/w.SimTime.
	cancel()
	<-simDone
	return runErr
}

// applyTurnRestrictions expands each high-level Ban category into concrete
// (from, to) edge pairs at the named intersection, classifying each
// incoming/outgoing pair by network.ClassifyTurn. The resulting
// TurnRestrictions are appended to the intersection's BannedTurns.
//
// Unknown intersection IDs and unknown ban categories are logged and skipped.
func applyTurnRestrictions(net *network.Network, restrictions []config.TurnRestrictionOverride) {
	for _, r := range restrictions {
		if int(r.IntersectionID) >= len(net.Intersections) {
			slog.Warn("turn restriction references unknown intersection",
				"id", r.IntersectionID, "count", len(net.Intersections))
			continue
		}
		x := &net.Intersections[r.IntersectionID]
		for _, banName := range r.Ban {
			cat, ok := parseTurnCategory(banName)
			if !ok {
				slog.Warn("unknown turn ban category; skipping",
					"id", r.IntersectionID, "ban", banName)
				continue
			}
			for _, fromEdge := range x.Incoming {
				for _, toEdge := range x.Outgoing {
					if network.ClassifyTurn(net, fromEdge, toEdge) == cat {
						x.BannedTurns = append(x.BannedTurns,
							network.TurnRestriction{From: fromEdge, To: toEdge})
					}
				}
			}
		}
	}
}

// parseTurnCategory maps the YAML string names to network.TurnCategory.
// Returns false for unknown names.
func parseTurnCategory(s string) (network.TurnCategory, bool) {
	switch s {
	case "left_turn":
		return network.TurnLeft, true
	case "right_turn":
		return network.TurnRight, true
	case "u_turn":
		return network.TurnUTurn, true
	case "straight_on":
		return network.TurnStraight, true
	}
	return 0, false
}

func countSignals(xs []network.Intersection) int {
	n := 0
	for _, x := range xs {
		if x.HasSignal {
			n++
		}
	}
	return n
}

func hasFlags(fs *flag.FlagSet) bool {
	any := false
	fs.VisitAll(func(*flag.Flag) { any = true })
	return any
}

// gameAdapter wraps a Viewport into Ebitengine's Game interface. If ctx
// is non-nil, Update returns ebiten.Termination as soon as ctx is done
// (used to wire SIGINT/SIGTERM into a clean ebiten shutdown).
type gameAdapter struct {
	vp  *render.Viewport
	ctx context.Context
}

func (g *gameAdapter) Update() error {
	if g.ctx != nil {
		select {
		case <-g.ctx.Done():
			return ebiten.Termination
		default:
		}
	}
	return g.vp.Update()
}
func (g *gameAdapter) Draw(screen *ebiten.Image)  { g.vp.Draw(screen) }
func (g *gameAdapter) Layout(w, h int) (int, int) { return g.vp.Layout(w, h) }

// --- trace sink ---

// evMsg is one queued trace event.
type evMsg struct {
	tick    uint64
	simTime float64
	e       trace.Event
}

// traceSink owns the file/bufio/writer triple plus a bounded backpressure
// channel and the drain goroutine that empties it. EmitTrace is set to
// sink.emit on the World; SimEnd and Close are driven from runRun's
// deferred shutdown.
type traceSink struct {
	file    *os.File
	bw      *bufio.Writer
	tw      *trace.Writer
	ch      chan evMsg
	done    chan struct{}
	dropped atomic.Uint64
}

const traceChannelDepth = 4096

func newTraceSink(path string) (*traceSink, error) {
	f, err := os.Create(path)
	if err != nil {
		return nil, err
	}
	bw := bufio.NewWriterSize(f, 64<<10) // 64 KiB
	ts := &traceSink{
		file: f,
		bw:   bw,
		tw:   trace.NewWriter(bw),
		ch:   make(chan evMsg, traceChannelDepth),
		done: make(chan struct{}),
	}
	go ts.drain()
	return ts, nil
}

// emit is the EmitTrace hook the sim calls on every event. Non-blocking
// to keep the sim's tick deterministic in wall-clock; overflow increments
// a counter that the drain loop later materializes as a KindTraceDropped
// marker in the on-disk stream.
func (ts *traceSink) emit(tick uint64, simTime float64, e trace.Event) {
	select {
	case ts.ch <- evMsg{tick, simTime, e}:
	default:
		n := ts.dropped.Add(1)
		if n == 1 || n%1000 == 0 {
			slog.Warn("trace event dropped (writer backpressure)", "dropped_total", n)
		}
	}
}

// drain pulls events from the channel and writes them through the
// bufio-wrapped trace.Writer. Before writing each event, if the dropped
// counter has grown since last seen, a KindTraceDropped marker is emitted
// FIRST so the replay tool can warn the user the stream is incomplete.
func (ts *traceSink) drain() {
	defer close(ts.done)
	var lastDropped uint64
	for m := range ts.ch {
		if cur := ts.dropped.Load(); cur > lastDropped {
			marker := &trace.TraceDropped{Count: uint32(cur - lastDropped)}
			if err := ts.tw.Write(m.tick, m.simTime, marker); err != nil {
				slog.Error("trace drop-marker write failed", "err", err)
				return
			}
			lastDropped = cur
		}
		if err := ts.tw.Write(m.tick, m.simTime, m.e); err != nil {
			slog.Error("trace write failed", "err", err)
			return
		}
	}
}

// close finalizes the trace: closes the channel, waits for the drain
// goroutine to flush all queued events, calls trace.Writer.Close to
// guarantee a header exists, then Flushes the bufio buffer, Syncs the
// file to disk, and closes it. Returns the first error encountered.
//
// Must be called exactly once, AFTER the sim goroutine has stopped and
// the caller has emitted SimEnd (so it lands in the queued events).
func (ts *traceSink) close() error {
	close(ts.ch)
	<-ts.done
	var firstErr error
	if err := ts.tw.Close(); err != nil {
		firstErr = fmt.Errorf("writer close: %w", err)
	}
	if err := ts.bw.Flush(); err != nil && firstErr == nil {
		firstErr = fmt.Errorf("bufio flush: %w", err)
	}
	if err := ts.file.Sync(); err != nil && firstErr == nil {
		firstErr = fmt.Errorf("fsync: %w", err)
	}
	if err := ts.file.Close(); err != nil && firstErr == nil {
		firstErr = fmt.Errorf("file close: %w", err)
	}
	if n := ts.dropped.Load(); n > 0 {
		slog.Warn("trace finalized with dropped events", "count", n)
	}
	return firstErr
}
