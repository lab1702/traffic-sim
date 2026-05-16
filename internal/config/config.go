// Package config loads YAML configuration files (signal overrides and
// turn restrictions today).
//
// LoadConfig errors on missing files: when the caller passes an explicit
// path, a typo or wrong working directory should fail loudly rather than
// silently producing zero overrides. Callers who want the "no config"
// semantics should not call LoadConfig at all (pass an empty path).
package config

import (
	"errors"
	"fmt"
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

// LoadConfig reads, parses, and validates the YAML config file at path.
// Returns an error if the file does not exist, fails to parse, or fails
// validation (see validate).
func LoadConfig(path string) (*Config, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("read %s: %w", path, err)
	}
	var raw rawFile
	if err := yaml.Unmarshal(data, &raw); err != nil {
		return nil, fmt.Errorf("parse %s: %w", path, err)
	}
	cfg := &Config{
		Signals:          raw.Signals,
		TurnRestrictions: raw.TurnRestrictions,
	}
	if err := cfg.validate(); err != nil {
		return nil, fmt.Errorf("validate %s: %w", path, err)
	}
	return cfg, nil
}

// validate runs schema-level checks that don't need a Network: phase
// durations are non-negative, at least one of green/yellow is positive
// per phase (so SignalState.Advance can make progress), green_edges
// positions are non-negative, ban categories are recognized.
//
// Cross-checks against the loaded Network (e.g. intersection_id range,
// HasSignal, edge positions in range) happen at the apply call site,
// where the network is available.
func (c *Config) validate() error {
	var errs []string
	for i, sig := range c.Signals {
		for j, p := range sig.Phases {
			if p.GreenDur < 0 {
				errs = append(errs, fmt.Sprintf("signals[%d].phases[%d].green_dur: negative (%.3f)", i, j, p.GreenDur))
			}
			if p.YellowDur < 0 {
				errs = append(errs, fmt.Sprintf("signals[%d].phases[%d].yellow_dur: negative (%.3f)", i, j, p.YellowDur))
			}
			if p.GreenDur <= 0 && p.YellowDur <= 0 {
				errs = append(errs, fmt.Sprintf("signals[%d].phases[%d]: at least one of green_dur/yellow_dur must be positive (degenerate phase would not progress)", i, j))
			}
			for k, pos := range p.GreenEdges {
				if pos < 0 {
					errs = append(errs, fmt.Sprintf("signals[%d].phases[%d].green_edges[%d]: negative position (%d)", i, j, k, pos))
				}
			}
		}
		if _, ok := parseSignalModeName(sig.Mode); !ok {
			errs = append(errs, fmt.Sprintf("signals[%d].mode: unrecognized %q (want normal/flash_a/flash_b/off, or empty)", i, sig.Mode))
		}
	}
	for i, r := range c.TurnRestrictions {
		for j, ban := range r.Ban {
			if !validBanCategory(ban) {
				errs = append(errs, fmt.Sprintf("turn_restrictions[%d].ban[%d]: unrecognized %q (want left_turn/right_turn/u_turn/straight_on)", i, j, ban))
			}
		}
	}
	if len(errs) > 0 {
		return errors.New(joinErrs(errs))
	}
	return nil
}

// parseSignalModeName accepts the same strings as sim.ParseSignalMode but
// without importing sim (which would be a cycle for tests). Kept in sync
// with sim.ParseSignalMode by inspection — both packages have tests.
func parseSignalModeName(s string) (string, bool) {
	switch s {
	case "", "normal", "flash_a", "flash_b", "off":
		return s, true
	}
	return s, false
}

func validBanCategory(s string) bool {
	switch s {
	case "left_turn", "right_turn", "u_turn", "straight_on":
		return true
	}
	return false
}

func joinErrs(ss []string) string {
	out := ""
	for i, s := range ss {
		if i > 0 {
			out += "; "
		}
		out += s
	}
	return out
}
