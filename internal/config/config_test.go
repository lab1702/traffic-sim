package config

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestLoadConfig_SignalsExample(t *testing.T) {
	path := filepath.Join("..", "..", "configs", "signals.example.yaml")
	cfg, err := LoadConfig(path)
	if err != nil {
		t.Fatalf("LoadConfig: %v", err)
	}
	if len(cfg.Signals) != 3 {
		t.Fatalf("want 3 signal overrides (custom phases + flash_a + off), got %d", len(cfg.Signals))
	}

	// First entry: custom phases, no mode.
	o0 := cfg.Signals[0]
	if o0.IntersectionID != 42 {
		t.Errorf("signals[0]: want IntersectionID 42, got %d", o0.IntersectionID)
	}
	if len(o0.Phases) != 2 {
		t.Errorf("signals[0]: want 2 phases, got %d", len(o0.Phases))
	}
	if o0.Phases[0].GreenDur != 45 {
		t.Errorf("signals[0] phase 0 green: want 45, got %v", o0.Phases[0].GreenDur)
	}
	if o0.Mode != "" {
		t.Errorf("signals[0]: want empty mode (defaults to normal), got %q", o0.Mode)
	}

	// Second entry: flash_a mode, no phases.
	if cfg.Signals[1].Mode != "flash_a" {
		t.Errorf("signals[1]: want mode 'flash_a', got %q", cfg.Signals[1].Mode)
	}
	if len(cfg.Signals[1].Phases) != 0 {
		t.Errorf("signals[1]: want no phases (inherits default), got %d", len(cfg.Signals[1].Phases))
	}

	// Third entry: off mode.
	if cfg.Signals[2].Mode != "off" {
		t.Errorf("signals[2]: want mode 'off', got %q", cfg.Signals[2].Mode)
	}
}

// TestLoadConfig_MissingFile_IsError documents the contract change: an
// explicit --signals path that doesn't exist now errors loudly rather
// than silently producing zero overrides. Callers that want the "no
// config" semantics should pass an empty path and skip LoadConfig.
func TestLoadConfig_MissingFile_IsError(t *testing.T) {
	_, err := LoadConfig("does-not-exist.yaml")
	if err == nil {
		t.Errorf("missing file should error; got nil")
	}
}

func TestLoadConfig_TurnRestrictions(t *testing.T) {
	path := filepath.Join("..", "..", "configs", "signals.example.yaml")
	cfg, err := LoadConfig(path)
	if err != nil {
		t.Fatalf("LoadConfig: %v", err)
	}
	if len(cfg.TurnRestrictions) != 2 {
		t.Fatalf("want 2 turn-restriction entries, got %d", len(cfg.TurnRestrictions))
	}
	if cfg.TurnRestrictions[0].IntersectionID != 100 {
		t.Errorf("entry[0].IntersectionID: want 100, got %d", cfg.TurnRestrictions[0].IntersectionID)
	}
	wantBans := []string{"left_turn", "u_turn"}
	if len(cfg.TurnRestrictions[0].Ban) != len(wantBans) {
		t.Fatalf("entry[0].Ban len: want %d, got %d", len(wantBans), len(cfg.TurnRestrictions[0].Ban))
	}
	for i, w := range wantBans {
		if cfg.TurnRestrictions[0].Ban[i] != w {
			t.Errorf("entry[0].Ban[%d]: want %q, got %q", i, w, cfg.TurnRestrictions[0].Ban[i])
		}
	}
	if len(cfg.TurnRestrictions[1].Ban) != 1 || cfg.TurnRestrictions[1].Ban[0] != "u_turn" {
		t.Errorf("entry[1].Ban: want [u_turn], got %v", cfg.TurnRestrictions[1].Ban)
	}
}

// TestLoadConfig_RejectsDegeneratePhase covers the validator: a phase
// with both green_dur=0 and yellow_dur=0 would spin SignalState.Advance
// forever, so the loader must reject it.
func TestLoadConfig_RejectsDegeneratePhase(t *testing.T) {
	yaml := `
signals:
  - intersection_id: 1
    phases:
      - green_edges: [0]
        green_dur: 0
        yellow_dur: 0
`
	path := writeTempYAML(t, yaml)
	_, err := LoadConfig(path)
	if err == nil {
		t.Fatalf("zero/zero phase must be rejected; got no error")
	}
	if !strings.Contains(err.Error(), "must be positive") {
		t.Errorf("error should mention the positive-duration requirement; got: %v", err)
	}
}

func TestLoadConfig_RejectsNegativeDuration(t *testing.T) {
	yaml := `
signals:
  - intersection_id: 1
    phases:
      - green_edges: [0]
        green_dur: -1
        yellow_dur: 3
`
	path := writeTempYAML(t, yaml)
	_, err := LoadConfig(path)
	if err == nil {
		t.Fatalf("negative green_dur must be rejected; got no error")
	}
}

func TestLoadConfig_RejectsUnknownMode(t *testing.T) {
	yaml := `
signals:
  - intersection_id: 1
    mode: blink_bonkers
`
	path := writeTempYAML(t, yaml)
	_, err := LoadConfig(path)
	if err == nil {
		t.Fatalf("unknown mode must be rejected; got no error")
	}
}

func TestLoadConfig_RejectsUnknownBanCategory(t *testing.T) {
	yaml := `
turn_restrictions:
  - intersection_id: 1
    ban: [diagonal_turn]
`
	path := writeTempYAML(t, yaml)
	_, err := LoadConfig(path)
	if err == nil {
		t.Fatalf("unknown ban category must be rejected; got no error")
	}
}

func writeTempYAML(t *testing.T, content string) string {
	t.Helper()
	f, err := os.CreateTemp(t.TempDir(), "config-*.yaml")
	if err != nil {
		t.Fatalf("CreateTemp: %v", err)
	}
	if _, err := f.WriteString(content); err != nil {
		t.Fatalf("WriteString: %v", err)
	}
	if err := f.Close(); err != nil {
		t.Fatalf("Close: %v", err)
	}
	return f.Name()
}
