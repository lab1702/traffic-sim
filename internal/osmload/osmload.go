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
// Restrictions holds OSM `type=restriction` relations (turn restrictions);
// netbuild resolves them into Intersection.BannedTurns entries.
type Features struct {
	Nodes        map[osm.NodeID]*osm.Node
	Ways         []*osm.Way
	Restrictions []*osm.Relation
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
		case *osm.Relation:
			if isTurnRestriction(o) {
				feat.Restrictions = append(feat.Restrictions, o)
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
		if want[n.ID] || isControlNode(n.Tags) {
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

// isTurnRestriction returns true for OSM relations tagged
// `type=restriction` (turn restrictions: no_left_turn, only_straight_on, etc.).
// netbuild filters these further; here we only triage what to retain.
func isTurnRestriction(r *osm.Relation) bool {
	for _, t := range r.Tags {
		if t.Key == "type" && t.Value == "restriction" {
			return true
		}
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

// isControlNode reports whether a node carries any tag that affects
// intersection right-of-way: traffic signal, stop sign, yield sign, or
// the way-scoped stop=all / stop=minor attribute (which is on the
// intersection node in OSM convention).
func isControlNode(tags osm.Tags) bool {
	for _, t := range tags {
		if t.Key == "highway" && (t.Value == "traffic_signals" || t.Value == "stop" || t.Value == "give_way") {
			return true
		}
		if t.Key == "stop" && (t.Value == "all" || t.Value == "minor") {
			return true
		}
	}
	return false
}
