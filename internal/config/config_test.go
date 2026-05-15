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
	if len(overrides) != 1 {
		t.Fatalf("want 1 override, got %d", len(overrides))
	}
	o := overrides[0]
	if o.IntersectionID != 42 {
		t.Errorf("want IntersectionID 42, got %d", o.IntersectionID)
	}
	if len(o.Phases) != 2 {
		t.Errorf("want 2 phases, got %d", len(o.Phases))
	}
	if o.Phases[0].GreenDur != 45 {
		t.Errorf("phase 0 green: want 45, got %v", o.Phases[0].GreenDur)
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
