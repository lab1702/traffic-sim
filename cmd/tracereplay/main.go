// Command tracereplay reads a trace file and plays it back in the viewer.
//
// It reconstructs vehicle positions by replaying spawn/despawn and
// signal-phase events at their recorded ticks. Vehicles between events
// are interpolated linearly along their route at the edge speed limit
// (no IDM — phase 8 simplification).
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
	osmPath := fs.String("osm", "", "path to the OSM file used for the original run (required)")
	tracePath := fs.String("trace", "", "path to a trace file written by 'trafficsim run --trace' (required)")
	speed := fs.Float64("speed", 1.0, "playback speed multiplier (1.0 = real time, 2.0 = 2x faster, 0.5 = half speed)")
	fs.Usage = func() {
		out := fs.Output()
		fmt.Fprintln(out, "Usage: tracereplay -osm <path> -trace <path> [-speed N]")
		fmt.Fprintln(out, "")
		fmt.Fprintln(out, "Replay a trace file in the viewer. Both -osm and -trace are required")
		fmt.Fprintln(out, "and must reference the same OSM file used by the original")
		fmt.Fprintln(out, "`trafficsim run --trace` invocation.")
		fmt.Fprintln(out, "")
		fmt.Fprintln(out, "Flags:")
		fs.PrintDefaults()
		fmt.Fprintln(out, "")
		fmt.Fprintln(out, "Examples:")
		fmt.Fprintln(out, "  tracereplay -osm city.osm.pbf -trace run.trace")
		fmt.Fprintln(out, "  tracereplay -speed 4 -osm city.osm.pbf -trace run.trace   # 4x faster")
		fmt.Fprintln(out, "")
		fmt.Fprintln(out, "Window controls: left-mouse drag to pan, wheel to zoom,")
		fmt.Fprintln(out, "drag edges/corners to resize.")
		fmt.Fprintln(out, "Signal hotkeys (N/Y/R/O on a selected intersection) are INERT in replay")
		fmt.Fprintln(out, "since trace events are immutable.")
	}
	if err := fs.Parse(os.Args[1:]); err != nil {
		os.Exit(2)
	}
	if *osmPath == "" || *tracePath == "" {
		fmt.Fprintln(os.Stderr, "tracereplay: -osm and -trace are required")
		fs.Usage()
		os.Exit(2)
	}
	if *speed <= 0 {
		fmt.Fprintln(os.Stderr, "tracereplay: -speed must be > 0")
		os.Exit(2)
	}

	feat, err := osmload.Load(*osmPath)
	if err != nil {
		slog.Error("load osm", "err", err)
		os.Exit(1)
	}
	net, _, err := netbuild.Build(feat)
	if err != nil {
		slog.Error("build", "err", err)
		os.Exit(1)
	}

	tf, err := os.Open(*tracePath)
	if err != nil {
		slog.Error("open trace", "err", err)
		os.Exit(1)
	}
	defer tf.Close()

	buf := snapshot.New()
	p := newPlayer(net, trace.NewReader(tf), buf, *speed)
	go p.run()

	vp := render.NewViewport(net, buf, 1280, 800)
	ebiten.SetWindowSize(1280, 800)
	ebiten.SetWindowTitle("tracereplay")
	ebiten.SetWindowResizingMode(ebiten.WindowResizingModeEnabled)
	if err := ebiten.RunGame(&gameAdapter{vp: vp}); err != nil {
		slog.Error("ebiten", "err", err)
	}
}

type gameAdapter struct{ vp *render.Viewport }

func (g *gameAdapter) Update() error               { return g.vp.Update() }
func (g *gameAdapter) Draw(s *ebiten.Image)        { g.vp.Draw(s) }
func (g *gameAdapter) Layout(w, h int) (int, int)  { return g.vp.Layout(w, h) }
