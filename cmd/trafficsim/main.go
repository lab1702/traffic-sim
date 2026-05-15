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
	default:
		usage()
		os.Exit(2)
	}
}

func usage() {
	fmt.Fprintln(os.Stderr, "usage: trafficsim <subcommand> [flags]")
	fmt.Fprintln(os.Stderr, "subcommands:")
	fmt.Fprintln(os.Stderr, "  load <path-to-osm>           parse and print graph stats")
	fmt.Fprintln(os.Stderr, "  run  <path-to-osm> [flags]   run the simulation")
}

func runLoad(args []string) {
	fs := flag.NewFlagSet("load", flag.ExitOnError)
	if err := fs.Parse(args); err != nil {
		os.Exit(2)
	}
	if fs.NArg() != 1 {
		fmt.Fprintln(os.Stderr, "load: need exactly one OSM path")
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

func runRun(args []string) {
	fs := flag.NewFlagSet("run", flag.ExitOnError)
	var (
		headless    = fs.Bool("headless", false, "skip rendering, run sim only")
		duration    = fs.Duration("duration", 0, "stop after this much sim time (0 = unbounded)")
		seed        = fs.Uint64("seed", 1, "RNG seed for deterministic runs")
		spawnRate   = fs.Float64("spawn-rate", 5.0, "vehicles spawned per simulated second")
		signalsPath = fs.String("signals", "", "path to signal overrides YAML")
		tracePath   = fs.String("trace", "", "write binary trace events to this file")
	)
	if err := fs.Parse(args); err != nil {
		os.Exit(2)
	}
	if fs.NArg() != 1 {
		fmt.Fprintln(os.Stderr, "run: need exactly one OSM path")
		os.Exit(2)
	}
	path := fs.Arg(0)

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
		w.EmitTrace(0, 0, &trace.SimStart{SeedLo: *seed, NetHash: 0})
		defer func() {
			w.EmitTrace(w.Tick, w.SimTime, &trace.SimEnd{Reason: "exit"})
			close(ch)
			<-done // wait for goroutine to flush and close the file
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
	if err := ebiten.RunGame(&gameAdapter{vp: vp}); err != nil {
		close(stop)
		slog.Error("ebiten exited", "err", err)
		os.Exit(1)
	}
	close(stop)
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
