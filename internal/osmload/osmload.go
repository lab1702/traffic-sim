// Package osmload parses .osm XML and .osm.pbf files and emits the raw OSM
// features (nodes, ways, relations) needed by netbuild. Format is detected
// from the file extension.
package osmload

import (
	"context"
	"errors"
	"fmt"
	"io"
	"os"
	"strings"

	"github.com/paulmach/osm"
	"github.com/paulmach/osm/osmpbf"
	"github.com/paulmach/osm/osmxml"
)

// Features is what netbuild consumes. Only highway-tagged ways are kept;
// nodes are kept if referenced by a kept way OR tagged with traffic_signals.
type Features struct {
	Nodes map[osm.NodeID]*osm.Node
	Ways  []*osm.Way
}

// Load reads an OSM file and returns the filtered Features set.
func Load(path string) (*Features, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, fmt.Errorf("open %s: %w", path, err)
	}
	defer f.Close()

	scanner, err := scannerFor(path, f)
	if err != nil {
		return nil, err
	}
	defer scanner.Close()

	return collect(scanner)
}

type scanner interface {
	Scan() bool
	Object() osm.Object
	Err() error
	Close() error
}

func scannerFor(path string, r io.Reader) (scanner, error) {
	switch {
	case strings.HasSuffix(path, ".osm.pbf"), strings.HasSuffix(path, ".pbf"):
		return osmpbf.New(context.Background(), r, 4), nil
	case strings.HasSuffix(path, ".osm"), strings.HasSuffix(path, ".xml"):
		return osmxml.New(context.Background(), r), nil
	default:
		return nil, fmt.Errorf("unrecognized extension on %s (want .osm, .osm.pbf)", path)
	}
}

func collect(s scanner) (*Features, error) {
	feat := &Features{Nodes: make(map[osm.NodeID]*osm.Node)}
	var allNodes []*osm.Node

	for s.Scan() {
		switch o := s.Object().(type) {
		case *osm.Node:
			allNodes = append(allNodes, o)
		case *osm.Way:
			if isHighway(o) {
				feat.Ways = append(feat.Ways, o)
			}
		}
	}
	if err := s.Err(); err != nil && !errors.Is(err, io.EOF) {
		return nil, fmt.Errorf("scan: %w", err)
	}

	// Keep only nodes that are referenced by a kept way OR are signals.
	want := make(map[osm.NodeID]bool)
	for _, w := range feat.Ways {
		for _, n := range w.Nodes {
			want[n.ID] = true
		}
	}
	for _, n := range allNodes {
		if want[n.ID] || hasTag(n.Tags, "highway", "traffic_signals") {
			feat.Nodes[n.ID] = n
		}
	}
	return feat, nil
}

func isHighway(w *osm.Way) bool {
	for _, t := range w.Tags {
		if t.Key == "highway" && drivableHighway(t.Value) {
			return true
		}
	}
	return false
}

// drivableHighway returns true for highway= values that carry motor traffic.
// Excludes footways, cycleways, paths, steps, etc.
func drivableHighway(v string) bool {
	switch v {
	case "motorway", "trunk", "primary", "secondary", "tertiary",
		"unclassified", "residential", "service", "living_street",
		"motorway_link", "trunk_link", "primary_link",
		"secondary_link", "tertiary_link":
		return true
	}
	return false
}

func hasTag(tags osm.Tags, key, value string) bool {
	for _, t := range tags {
		if t.Key == key && t.Value == value {
			return true
		}
	}
	return false
}
