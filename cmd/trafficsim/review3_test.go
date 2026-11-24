package main

import (
	"path/filepath"
	"testing"

	"github.com/lab1702/traffic-sim/internal/config"
	"github.com/lab1702/traffic-sim/internal/netbuild"
	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/lab1702/traffic-sim/internal/sim"
	"github.com/lab1702/traffic-sim/internal/trace"
)

// TestLoadOverrides_ModeOnlyKeepsDefaultPlan: an override that sets only
// mode: used to copy just the default plan's phases, leaving the signal
// fixed-time instead of semi-actuated once it ran in normal mode.
func TestLoadOverrides_ModeOnlyKeepsDefaultPlan(t *testing.T) {
	net := crossNet()
	path := writeFile(t, "signals.yaml", `
signals:
  - intersection_id: 0
    mode: flash_a
`)
	overrides, err := loadOverrides(path, net)
	if err != nil {
		t.Fatalf("loadOverrides: %v", err)
	}
	got, ok := overrides[0]
	if !ok {
		t.Fatal("no override for intersection 0")
	}
	def := sim.DefaultSignalConfig(net.Intersections[0].Incoming, net)
	if def.Plan != sim.PlanSemiActuated {
		t.Fatalf("fixture: default plan is %v, want semi-actuated", def.Plan)
	}
	if got.Plan != def.Plan || got.MajorPhase != def.MajorPhase || len(got.Phases) != len(def.Phases) {
		t.Errorf("override plan %v major %d phases %d, want the default's %v major %d phases %d",
			got.Plan, got.MajorPhase, len(got.Phases), def.Plan, def.MajorPhase, len(def.Phases))
	}
	if got.InitialMode != sim.ModeFlashA || got.IntersectionID != 0 {
		t.Errorf("override mode %v id %d, want flash_a on intersection 0", got.InitialMode, got.IntersectionID)
	}
}

// TestRunRun_NetHashIgnoresConfigTurnBans: the trace fingerprint used to be
// taken after --signals added its turn restrictions, which network.Hash
// covers, so tracereplay (which builds the OSM file without them) warned of a
// mismatch for every such trace.
func TestRunRun_NetHashIgnoresConfigTurnBans(t *testing.T) {
	osmPath := writeFile(t, "cross.osm", crossOSM)
	signals := writeFile(t, "signals.yaml", `
turn_restrictions:
  - intersection_id: 0
    ban: [left_turn]
`)
	tracePath := filepath.Join(t.TempDir(), "run.trace")
	if err := runRun([]string{"--headless", "--duration", "1s", "--signals", signals, "--trace", tracePath, osmPath}); err != nil {
		t.Fatalf("runRun: %v", err)
	}
	start, ok := readTrace(t, tracePath)[0].(*trace.SimStart)
	if !ok {
		t.Fatal("trace does not start with SimStart")
	}

	feat, err := osmload.Load(osmPath)
	if err != nil {
		t.Fatalf("load: %v", err)
	}
	net, _, err := netbuild.Build(feat)
	if err != nil {
		t.Fatalf("build: %v", err)
	}
	plain := network.Hash(net)
	if start.NetHash != plain {
		t.Fatalf("SimStart NetHash = %x, want the plain OSM build's %x (what tracereplay computes)", start.NetHash, plain)
	}
	// The config bans really change the hash, so the check above is not vacuous.
	cfg, err := config.LoadConfig(signals)
	if err != nil {
		t.Fatalf("config: %v", err)
	}
	applyTurnRestrictions(net, cfg.TurnRestrictions)
	if network.Hash(net) == plain {
		t.Fatal("fixture: the config turn ban did not change the network hash")
	}
}
