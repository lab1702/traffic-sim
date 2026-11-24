package netbuild

import (
	"testing"

	"github.com/paulmach/osm"
)

// TestParseMaxspeedValue_RejectsUnusableNumbers: strconv.ParseFloat reads
// "0", negative numbers, "NaN" and "inf", and they used to become the edge's
// speed limit: 0 stalls every vehicle on the edge, NaN poisons its speed and
// position. They must fall back like any other unrecognized value.
func TestParseMaxspeedValue_RejectsUnusableNumbers(t *testing.T) {
	for _, raw := range []string{"0", "-30", "NaN", "inf", "+Inf", "0 mph", "-20 mph", "NaN mph", "DE:zone0", "DE:zone:-30"} {
		if v, ok := parseMaxspeedValue(raw); ok {
			t.Errorf("parseMaxspeedValue(%q) = %v, true; want false", raw, v)
		}
	}
	// Positive values still parse.
	for raw, want := range map[string]float64{"50": 50 / 3.6, "30 mph": 30 * 0.44704, "DE:zone30": 30 / 3.6} {
		if v, ok := parseMaxspeedValue(raw); !ok || !nearly(v, want) {
			t.Errorf("parseMaxspeedValue(%q) = %v, %v; want %v, true", raw, v, ok, want)
		}
	}
}

// TestParseSpeedForDirection_ZeroFallsBack: a way tagged maxspeed=0 gets the
// highway-type default, not a zero speed limit.
func TestParseSpeedForDirection_ZeroFallsBack(t *testing.T) {
	w := &osm.Way{Tags: osm.Tags{{Key: "highway", Value: "residential"}, {Key: "maxspeed", Value: "0"}}}
	if got := parseSpeedForDirection(w, true, 11.2); got != 11.2 {
		t.Fatalf("speed = %v, want the 11.2 m/s fallback", got)
	}
}
