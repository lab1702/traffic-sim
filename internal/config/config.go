// Package config loads YAML configuration files (signal overrides today,
// more later). Missing files are not errors — defaults apply.
package config

import (
	"errors"
	"fmt"
	"io/fs"
	"os"

	"gopkg.in/yaml.v3"
)

type SignalOverride struct {
	IntersectionID uint32        `yaml:"intersection_id"`
	Phases         []PhaseConfig `yaml:"phases"`
}

type PhaseConfig struct {
	GreenEdges []int   `yaml:"green_edges"`
	GreenDur   float64 `yaml:"green_dur"`
	YellowDur  float64 `yaml:"yellow_dur"`
}

type signalFile struct {
	Signals []SignalOverride `yaml:"signals"`
}

func LoadSignalOverrides(path string) ([]SignalOverride, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		if errors.Is(err, fs.ErrNotExist) {
			return nil, nil
		}
		return nil, fmt.Errorf("read %s: %w", path, err)
	}
	var sf signalFile
	if err := yaml.Unmarshal(data, &sf); err != nil {
		return nil, fmt.Errorf("parse %s: %w", path, err)
	}
	return sf.Signals, nil
}
