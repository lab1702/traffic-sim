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
	p := newPlayer(net, trace.NewReader(tf), buf)
	go p.run()

	vp := render.NewViewport(net, buf, 1280, 800)
	ebiten.SetWindowSize(1280, 800)
	ebiten.SetWindowTitle("tracereplay")
	if err := ebiten.RunGame(&gameAdapter{vp: vp}); err != nil {
		slog.Error("ebiten", "err", err)
	}
}

type gameAdapter struct{ vp *render.Viewport }

func (g *gameAdapter) Update() error              { return g.vp.Update() }
func (g *gameAdapter) Draw(s *ebiten.Image)        { g.vp.Draw(s) }
func (g *gameAdapter) Layout(w, h int) (int, int) { return g.vp.Layout(w, h) }
