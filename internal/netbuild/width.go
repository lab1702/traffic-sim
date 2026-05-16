package netbuild

import (
	"strconv"
	"strings"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/paulmach/osm"
)

// parseWidthMeters interprets one OSM `width=*` value and returns the
// physical road width in meters.
//
// Supported value forms:
//
//	"7.5"         -> 7.5 m (bare number = meters per OSM convention)
//	"7.5 m"       -> 7.5 m
//	"7.5m"        -> 7.5 m
//	"7.5 meters"  -> 7.5 m
//	"24 ft"       -> 7.32 m
//	"24'"         -> 7.32 m (feet symbol)
//	"24'6\""      -> 7.47 m (feet-and-inches form, common in US data)
//
// Returns (0, false) on anything unrecognized so the caller can fall back
// to the lane-count estimate.
func parseWidthMeters(raw string) (float64, bool) {
	s := strings.TrimSpace(raw)
	if s == "" {
		return 0, false
	}

	// Feet-and-inches form: e.g. `24'6"`. Split on the apostrophe and
	// parse each side. The inches part may include a trailing quote.
	if idx := strings.Index(s, "'"); idx > 0 {
		feetStr := strings.TrimSpace(s[:idx])
		rest := strings.TrimSpace(s[idx+1:])
		rest = strings.TrimSuffix(rest, "\"")
		rest = strings.TrimSpace(rest)
		feet, err := strconv.ParseFloat(feetStr, 64)
		if err != nil || feet < 0 {
			return 0, false
		}
		var inches float64
		if rest != "" {
			n, err := strconv.ParseFloat(rest, 64)
			if err != nil || n < 0 {
				return 0, false
			}
			inches = n
		}
		return (feet*12 + inches) * 0.0254, true
	}

	// Suffix forms.
	lower := strings.ToLower(s)
	switch {
	case strings.HasSuffix(lower, "ft"), strings.HasSuffix(lower, "feet"):
		num := strings.TrimSpace(strings.TrimSuffix(strings.TrimSuffix(lower, "ft"), "feet"))
		n, err := strconv.ParseFloat(num, 64)
		if err != nil || n <= 0 {
			return 0, false
		}
		return n * 0.3048, true
	case strings.HasSuffix(lower, "meters"), strings.HasSuffix(lower, "metres"):
		num := strings.TrimSpace(strings.TrimSuffix(strings.TrimSuffix(lower, "meters"), "metres"))
		n, err := strconv.ParseFloat(num, 64)
		if err != nil || n <= 0 {
			return 0, false
		}
		return n, true
	case strings.HasSuffix(lower, "m"):
		num := strings.TrimSpace(strings.TrimSuffix(lower, "m"))
		n, err := strconv.ParseFloat(num, 64)
		if err != nil || n <= 0 {
			return 0, false
		}
		return n, true
	}

	// Bare number → meters.
	if n, err := strconv.ParseFloat(s, 64); err == nil && n > 0 {
		return n, true
	}
	return 0, false
}

// wayWidthMeters returns the total physical road width for an OSM way.
// Honors the `width=*` tag if present and parseable, otherwise estimates
// from the lane count: a one-way contributes lanesFwd lanes; a two-way
// contributes lanesFwd + lanesBwd. lanesFwd and lanesBwd are the values
// produced by parseLanesPerDirection.
//
// Both edges of a two-way way receive the same width so the renderer can
// paint one band over the shared centerline geometry.
func wayWidthMeters(w *osm.Way, dir onewayDir, lanesFwd, lanesBwd uint8) float64 {
	for _, t := range w.Tags {
		if t.Key == "width" {
			if v, ok := parseWidthMeters(t.Value); ok {
				return v
			}
		}
	}
	totalLanes := float64(lanesFwd)
	if dir == onewayTwoWay {
		totalLanes = float64(lanesFwd) + float64(lanesBwd)
	}
	if totalLanes < 1 {
		totalLanes = 1
	}
	return totalLanes * network.LaneWidthMeters
}
