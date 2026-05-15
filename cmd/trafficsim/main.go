package main

import (
	"flag"
	"fmt"
	"log/slog"
	"os"

	"github.com/lab1702/traffic-sim/internal/config"
	"github.com/lab1702/traffic-sim/internal/netbuild"
	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/lab1702/traffic-sim/internal/sim"
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
	)
	if err := fs.Parse(args); err != nil {
		os.Exit(2)
	}
	if fs.NArg() != 1 {
		fmt.Fprintln(os.Stderr, "run: need exactly one OSM path")
		os.Exit(2)
	}
	path := fs.Arg(0)

	if !*headless {
		fmt.Fprintln(os.Stderr, "warning: rendering not implemented yet; pass --headless")
		os.Exit(2)
	}
	if *duration == 0 {
		fmt.Fprintln(os.Stderr, "error: --headless requires --duration > 0")
		os.Exit(2)
	}

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

	w.Run(duration.Seconds())
	fmt.Printf("done. final_vehicles=%d ticks=%d sim_time=%.2fs\n",
		len(w.Vehicles), w.Tick, w.SimTime)
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
