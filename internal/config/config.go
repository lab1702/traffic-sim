// Package config loads YAML configuration files (signal overrides and
// turn restrictions today). Missing files are not errors — defaults apply.
package config

import (
	"errors"
	"fmt"
	"io/fs"
	"os"

	"gopkg.in/yaml.v3"
)

// Config bundles all per-intersection configuration loaded from one YAML
// file. Both lists are optional; empty slices are valid.
type Config struct {
	Signals          []SignalOverride
	TurnRestrictions []TurnRestrictionOverride
}

type SignalOverride struct {
	IntersectionID uint32        `yaml:"intersection_id"`
	Phases         []PhaseConfig `yaml:"phases"`
	// Mode is the initial operating mode: "normal" (default), "flash_a",
	// "flash_b", or "off". Empty == "normal". Phases are still honored
	// for the geometry of flash modes (phase 0 vs phase 1 grouping).
	Mode string `yaml:"mode"`
}

type PhaseConfig struct {
	GreenEdges []int   `yaml:"green_edges"`
	GreenDur   float64 `yaml:"green_dur"`
	YellowDur  float64 `yaml:"yellow_dur"`
}

// TurnRestrictionOverride declares forbidden turns at one intersection.
// Each entry in Ban is a high-level category — the loader caller expands
// these into concrete (from, to) edge pairs using arrival/departure
// headings (see network.ClassifyTurn).
//
// Valid Ban categories: "left_turn", "right_turn", "u_turn", "straight_on".
type TurnRestrictionOverride struct {
	IntersectionID uint32   `yaml:"intersection_id"`
	Ban            []string `yaml:"ban"`
}

// rawFile mirrors the on-disk YAML structure.
type rawFile struct {
	Signals          []SignalOverride          `yaml:"signals"`
	TurnRestrictions []TurnRestrictionOverride `yaml:"turn_restrictions"`
}

// LoadConfig reads the YAML config file at path. Returns an empty Config
// (not an error) if the file does not exist. Malformed YAML or read errors
// are returned as errors.
func LoadConfig(path string) (*Config, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		if errors.Is(err, fs.ErrNotExist) {
			return &Config{}, nil
		}
		return nil, fmt.Errorf("read %s: %w", path, err)
	}
	var raw rawFile
	if err := yaml.Unmarshal(data, &raw); err != nil {
		return nil, fmt.Errorf("parse %s: %w", path, err)
	}
	return &Config{
		Signals:          raw.Signals,
		TurnRestrictions: raw.TurnRestrictions,
	}, nil
}

// LoadSignalOverrides is retained as a convenience wrapper over LoadConfig
// for callers that only care about the signal section.
func LoadSignalOverrides(path string) ([]SignalOverride, error) {
	cfg, err := LoadConfig(path)
	if err != nil {
		return nil, err
	}
	return cfg.Signals, nil
}
