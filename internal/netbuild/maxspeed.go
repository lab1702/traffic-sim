package netbuild

import (
	"strconv"
	"strings"

	"github.com/paulmach/osm"
)

// kmhToMs converts km/h to m/s.
func kmhToMs(v float64) float64 { return v / 3.6 }

// mphToMs converts mph to m/s.
func mphToMs(v float64) float64 { return v * 0.44704 }

// implicitZoneLimits holds km/h defaults for OSM-standard implicit zone
// tags (used when the maxspeed value is "urban", "rural", etc. without a
// country prefix). Values are reasonable cross-country averages; for
// country-specific values use countryZoneLimits below.
var implicitZoneLimits = map[string]float64{
	"urban":         50,  // 50 km/h is the EU/global majority
	"rural":         90,  // 90 km/h is typical EU
	"motorway":      110, // common EU motorway when not "none"
	"trunk":         100,
	"living_street": 10, // walking-pace zone
	"walk":          6,  // shared zones, ~6 km/h
	"school":        30, // school zone
	"nsl_single":    96, // ≈60 mph (UK national speed limit, single carriageway)
	"nsl_dual":      112, // ≈70 mph (UK NSL dual carriageway)
}

// countryZoneLimits provides per-country defaults for tags like
// `maxspeed=DE:urban`. The outer key is the ISO 3166-1 alpha-2 country
// code; the inner key is the zone name. Values are km/h.
//
// Coverage is the top-N countries by OSM data volume / tagging conventions.
// Unknown countries fall through to implicitZoneLimits.
var countryZoneLimits = map[string]map[string]float64{
	"DE": { // Germany
		"urban": 50, "rural": 100, "motorway": 130, "trunk": 100,
		"living_street": 7, "bicycle_road": 30,
	},
	"AT": { // Austria
		"urban": 50, "rural": 100, "motorway": 130, "trunk": 100,
	},
	"CH": { // Switzerland
		"urban": 50, "rural": 80, "motorway": 120, "trunk": 100,
	},
	"FR": { // France
		"urban": 50, "rural": 80, "motorway": 130, "trunk": 110,
		"living_street": 20,
	},
	"IT": { // Italy
		"urban": 50, "rural": 90, "motorway": 130, "trunk": 110,
	},
	"ES": { // Spain
		"urban": 50, "rural": 90, "motorway": 120, "trunk": 100,
	},
	"NL": { // Netherlands
		"urban": 50, "rural": 80, "motorway": 130, "trunk": 100,
	},
	"BE": { // Belgium
		"urban": 50, "rural": 70, "motorway": 120, "trunk": 90,
	},
	"GB": { // United Kingdom (values in mph internally — convert)
		"nsl_single": 96, "nsl_dual": 112, "motorway": 112,
		"urban": 48, "rural": 96, "living_street": 32,
	},
	"IE": { // Ireland
		"urban": 50, "rural": 80, "motorway": 120,
	},
	"US": { // United States — typical values (vary widely by state)
		"urban": 40, "rural": 89, "motorway": 105, "trunk": 89, // km/h equivalents of ~25/55/65/55 mph
		"school": 24, // ~15 mph
	},
	"CA": { // Canada
		"urban": 50, "rural": 80, "motorway": 100, "trunk": 90,
	},
	"AU": { // Australia
		"urban": 50, "rural": 100, "motorway": 110, "trunk": 100,
	},
	"NZ": { // New Zealand
		"urban": 50, "rural": 100, "motorway": 100,
	},
	"JP": { // Japan
		"urban": 40, "rural": 60, "motorway": 100,
	},
}

// noneSpeedLimit is the cap for `maxspeed=none` ways (German Autobahn).
// OSM convention uses the recommendation of 130 km/h.
const noneSpeedLimit = 130.0 / 3.6 // m/s

// parseMaxspeedValue interprets a single maxspeed tag value and returns
// the speed in m/s if recognized. Returns (0, false) on unknown values.
//
// Supported value forms:
//
//	"50"            -> 50 km/h
//	"30 mph"        -> 30 mph
//	"none"          -> noneSpeedLimit (cap, Autobahn)
//	"signals" / "variable" -> not recognized (let caller fall back)
//	"urban" / "rural" / "motorway" / "living_street" / "walk" / "school"
//	    -> implicit-zone default
//	"DE:urban", "US:rural", etc.
//	    -> country-specific implicit zone (km/h)
//	"DE:zone30" / "DE:zone:30"
//	    -> 30 km/h (German Tempo-30 zones)
func parseMaxspeedValue(raw string) (float64, bool) {
	s := strings.TrimSpace(raw)
	if s == "" {
		return 0, false
	}

	// 1. mph form.
	if strings.HasSuffix(s, "mph") {
		num := strings.TrimSpace(strings.TrimSuffix(s, "mph"))
		if n, err := strconv.ParseFloat(num, 64); err == nil {
			return mphToMs(n), true
		}
		return 0, false
	}

	// 2. Plain numeric value (km/h).
	if n, err := strconv.ParseFloat(s, 64); err == nil {
		return kmhToMs(n), true
	}

	// 3. Special-case "none".
	if s == "none" {
		return noneSpeedLimit, true
	}

	// 4. Country-prefixed: "DE:urban", "US:rural", "DE:zone30", etc.
	if colon := strings.Index(s, ":"); colon > 0 {
		country := strings.ToUpper(s[:colon])
		zone := strings.ToLower(s[colon+1:])

		// "zone30" or "zone:30" forms encode a numeric km/h limit in a zone.
		// Strip "zone" prefix and try to parse the rest as a number.
		if numStr, ok := stripPrefix(zone, "zone"); ok {
			numStr = strings.TrimPrefix(numStr, ":")
			if n, err := strconv.ParseFloat(numStr, 64); err == nil {
				return kmhToMs(n), true
			}
		}

		if cm, ok := countryZoneLimits[country]; ok {
			if v, ok := cm[zone]; ok {
				return kmhToMs(v), true
			}
		}
		// Country known but zone unknown — fall through to implicit table.
		if v, ok := implicitZoneLimits[zone]; ok {
			return kmhToMs(v), true
		}
		return 0, false
	}

	// 5. Bare implicit zone (no country prefix).
	if v, ok := implicitZoneLimits[s]; ok {
		return kmhToMs(v), true
	}

	// "signals" / "variable" / anything else: caller's fallback.
	return 0, false
}

// stripPrefix returns (s without prefix, true) if s starts with prefix,
// else (s, false).
func stripPrefix(s, prefix string) (string, bool) {
	if strings.HasPrefix(s, prefix) {
		return s[len(prefix):], true
	}
	return s, false
}

// parseSpeedForDirection finds the speed-limit value for one direction
// of an OSM way. It prefers the direction-specific tag if present:
//
//	`maxspeed:forward` or `maxspeed:backward`
//
// and falls back to `maxspeed` if not. Unknown / unparseable values fall
// through to the caller's fallback.
//
// `forward` selects which direction is being built (true for the way's
// natural direction, false for the reverse).
func parseSpeedForDirection(w *osm.Way, forward bool, fallback float64) float64 {
	directional := "maxspeed:forward"
	if !forward {
		directional = "maxspeed:backward"
	}
	for _, t := range w.Tags {
		if t.Key == directional {
			if v, ok := parseMaxspeedValue(t.Value); ok {
				return v
			}
		}
	}
	for _, t := range w.Tags {
		if t.Key == "maxspeed" {
			if v, ok := parseMaxspeedValue(t.Value); ok {
				return v
			}
		}
	}
	return fallback
}
