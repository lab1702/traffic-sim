// Package netbuild: lane-to-turn assignment. Populates Lane.AllowedTurns
// for every edge whose downstream node is a multi-edge intersection,
// either from the OSM `turn:lanes=*` tag or via geometric inference.
package netbuild

import (
	"strings"

	"github.com/lab1702/traffic-sim/internal/network"
)

// parseTurnLaneSpec parses one lane's spec from an OSM turn:lanes string.
// A spec can list multiple turn types separated by ';'. Unknown tokens
// are dropped. An empty spec ("" or "none") maps to TurnStraight.
//
// Returns nil if every token in the spec was unrecognized (e.g., "reverse"
// or "floof"). Callers should treat nil as "no usable OSM data for this
// lane" — distinct from a non-empty result like {TurnStraight}, which
// means the OSM data explicitly permits only that turn.
func parseTurnLaneSpec(spec string) []network.TurnCategory {
	if spec == "" || spec == "none" {
		return []network.TurnCategory{network.TurnStraight}
	}
	var out []network.TurnCategory
	seen := map[network.TurnCategory]bool{}
	for _, tok := range strings.Split(spec, ";") {
		tok = strings.TrimSpace(tok)
		c, ok := turnTokenMap[tok]
		if !ok {
			continue
		}
		if !seen[c] {
			out = append(out, c)
			seen[c] = true
		}
	}
	return out
}

// parseTurnLanesString parses a full OSM turn:lanes value (pipe-delimited
// per-lane specs). Returns one entry per lane. Returns nil for empty input.
func parseTurnLanesString(s string) [][]network.TurnCategory {
	if s == "" {
		return nil
	}
	parts := strings.Split(s, "|")
	out := make([][]network.TurnCategory, len(parts))
	for i, p := range parts {
		out[i] = parseTurnLaneSpec(p)
	}
	return out
}

var turnTokenMap = map[string]network.TurnCategory{
	"left":           network.TurnLeft,
	"slight_left":    network.TurnLeft,
	"sharp_left":     network.TurnLeft,
	"merge_to_left":  network.TurnLeft,
	"right":          network.TurnRight,
	"slight_right":   network.TurnRight,
	"sharp_right":    network.TurnRight,
	"merge_to_right": network.TurnRight,
	"through":        network.TurnStraight,
	// "none" and "" handled by parseTurnLaneSpec directly.
	// "reverse" intentionally absent — U-turns are dropped.
}
