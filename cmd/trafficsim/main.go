package main

import (
	"flag"
	"fmt"
	"log/slog"
	"os"
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
		runRun(os.Args[2:])
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
	fmt.Printf("bounds=(%.1f,%.1f)-(%.1f,%.1f) m\n",
		net.Bounds.MinX, net.Bounds.MinY, net.Bounds.MaxX, net.Bounds.MaxY)
}

type runFlags struct {
	headless    bool
	duration    time.Duration
	seed        uint64
	spawnRate   float64
	signalsPath string
	tracePath   string
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

func runRun(args []string) {
	fs, f := newRunFlagSet()
	if err := fs.Parse(args); err != nil {
		os.Exit(2)
	}
	if fs.NArg() != 1 {
		fmt.Fprintln(os.Stderr, "run: need exactly one OSM path")
		fs.Usage()
		os.Exit(2)
	}
	path := fs.Arg(0)
	headless := &f.headless
	duration := &f.duration
	seed := &f.seed
	spawnRate := &f.spawnRate
	signalsPath := &f.signalsPath
	tracePath := &f.tracePath

	feat, err := osmload.Load(path)
	if err != nil {
		slog.Error("load failed", "err", err)
		os.Exit(1)
	}
	net, _, err := netbuild.Build(feat)
	if err != nil {
		slog.Error("build failed", "err", err)
		os.Exit(1)
	}

	overrides := map[network.IntersectionID]sim.SignalConfig{}
	if *signalsPath != "" {
		list, err := config.LoadSignalOverrides(*signalsPath)
		if err != nil {
			slog.Error("signals load failed", "err", err)
			os.Exit(1)
		}
		for _, o := range list {
			phases := make([]sim.SignalPhase, len(o.Phases))
			for i, p := range o.Phases {
				phases[i] = sim.SignalPhase{
					GreenEdges: p.GreenEdges, GreenDur: p.GreenDur, YellowDur: p.YellowDur,
				}
			}
			// Validate intersection_id is in range; warn and skip if not.
			if int(o.IntersectionID) >= len(net.Intersections) {
				slog.Warn("signal override references unknown intersection",
					"id", o.IntersectionID, "max", len(net.Intersections)-1)
				continue
			}
			overrides[network.IntersectionID(o.IntersectionID)] = sim.SignalConfig{
				IntersectionID: network.IntersectionID(o.IntersectionID),
				Phases:         phases,
			}
		}
	}

	spawner := sim.NewRandomOD(net, *seed, *spawnRate)
	w := sim.NewWorld(net, spawner, overrides)

	if *tracePath != "" {
		f, err := os.Create(*tracePath)
		if err != nil {
			slog.Error("trace create failed", "err", err)
			os.Exit(1)
		}
		tw := trace.NewWriter(f)
		type evMsg struct {
			tick    uint64
			simTime float64
			e       trace.Event
		}
		ch := make(chan evMsg, 4096)
		done := make(chan struct{})
		go func() {
			defer close(done)
			for m := range ch {
				if err := tw.Write(m.tick, m.simTime, m.e); err != nil {
					slog.Error("trace write failed", "err", err)
					return
				}
			}
			_ = tw.Close()
			_ = f.Close()
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
		// TODO: compute and emit a real NetHash so tracereplay can validate the OSM matches.
		w.EmitTrace(0, 0, &trace.SimStart{
			SeedLo:  *seed,
			SeedHi:  *seed ^ 0x9E3779B97F4A7C15, // matches RandomOD's PCG seed pair
			NetHash: 0,
		})
		defer func() {
			w.EmitTrace(w.Tick, w.SimTime, &trace.SimEnd{Reason: "exit"})
			close(ch)
			<-done // wait for goroutine to flush and close the file
			if dropped > 0 {
				slog.Warn("trace: total events dropped", "count", dropped)
			}
		}()
	}

	if *headless {
		if *duration == 0 {
			fmt.Fprintln(os.Stderr, "error: --headless requires --duration > 0")
			os.Exit(2)
		}
		w.Run(duration.Seconds())
		fmt.Printf("done. final_vehicles=%d ticks=%d sim_time=%.2fs\n",
			len(w.Vehicles), w.Tick, w.SimTime)
		return
	}

	// Live mode: sim runs on its own goroutine at 20 Hz wall-clock,
	// renderer runs at Ebitengine's default frame rate (~60 FPS).
	stop := make(chan struct{})
	go func() {
		ticker := time.NewTicker(time.Duration(sim.DefaultDt * float64(time.Second)))
		defer ticker.Stop()
		for {
			select {
			case <-stop:
				return
			case <-ticker.C:
				w.Step()
			}
		}
	}()

	vp := render.NewViewport(net, w.SnapshotBuf, 1280, 800)
	ebiten.SetWindowSize(1280, 800)
	ebiten.SetWindowTitle("traffic-sim")
	ebiten.SetWindowResizingMode(ebiten.WindowResizingModeEnabled)
	if err := ebiten.RunGame(&gameAdapter{vp: vp}); err != nil {
		close(stop)
		slog.Error("ebiten exited", "err", err)
		os.Exit(1)
	}
	close(stop)
}

func hasFlags(fs *flag.FlagSet) bool {
	any := false
	fs.VisitAll(func(*flag.Flag) { any = true })
	return any
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

// gameAdapter wraps a Viewport into Ebitengine's Game interface.
type gameAdapter struct {
	vp *render.Viewport
}

func (g *gameAdapter) Update() error             { return g.vp.Update() }
func (g *gameAdapter) Draw(screen *ebiten.Image) { g.vp.Draw(screen) }
func (g *gameAdapter) Layout(w, h int) (int, int) { return g.vp.Layout(w, h) }
