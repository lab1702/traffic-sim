package config

import (
	"path/filepath"
	"testing"
)

func TestLoadSignalOverrides(t *testing.T) {
	path := filepath.Join("..", "..", "configs", "signals.example.yaml")
	overrides, err := LoadSignalOverrides(path)
	if err != nil {
		t.Fatalf("LoadSignalOverrides: %v", err)
	}
	if len(overrides) != 3 {
		t.Fatalf("want 3 overrides (custom phases + flash_a + off), got %d", len(overrides))
	}

	// First entry: custom phases, no mode.
	o0 := overrides[0]
	if o0.IntersectionID != 42 {
		t.Errorf("override[0]: want IntersectionID 42, got %d", o0.IntersectionID)
	}
	if len(o0.Phases) != 2 {
		t.Errorf("override[0]: want 2 phases, got %d", len(o0.Phases))
	}
	if o0.Phases[0].GreenDur != 45 {
		t.Errorf("override[0] phase 0 green: want 45, got %v", o0.Phases[0].GreenDur)
	}
	if o0.Mode != "" {
		t.Errorf("override[0]: want empty mode (defaults to normal), got %q", o0.Mode)
	}

	// Second entry: flash_a mode, no phases.
	if overrides[1].Mode != "flash_a" {
		t.Errorf("override[1]: want mode 'flash_a', got %q", overrides[1].Mode)
	}
	if len(overrides[1].Phases) != 0 {
		t.Errorf("override[1]: want no phases (inherits default), got %d", len(overrides[1].Phases))
	}

	// Third entry: off mode.
	if overrides[2].Mode != "off" {
		t.Errorf("override[2]: want mode 'off', got %q", overrides[2].Mode)
	}
}

func TestLoadSignalOverrides_MissingFile_NotAnError(t *testing.T) {
	overrides, err := LoadSignalOverrides("does-not-exist.yaml")
	if err != nil {
		t.Errorf("missing file should return empty list, not error: %v", err)
	}
	if len(overrides) != 0 {
		t.Errorf("want empty, got %d entries", len(overrides))
	}
}
