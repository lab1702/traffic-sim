package netbuild

import (
	"math"
	"testing"

	"github.com/paulmach/osm"
)

func nearly(a, b float64) bool {
	return math.Abs(a-b) < 0.05
}

func TestParseMaxspeedValue(t *testing.T) {
	cases := []struct {
		in   string
		want float64 // m/s
		ok   bool
	}{
		// Numeric forms
		{"50", kmhToMs(50), true},
		{"30 mph", mphToMs(30), true},
		{" 100 ", kmhToMs(100), true},

		// Special
		{"none", noneSpeedLimit, true},
		{"signals", 0, false}, // not recognized; caller falls back

		// Bare implicit zones
		{"urban", kmhToMs(50), true},
		{"rural", kmhToMs(90), true},
		{"motorway", kmhToMs(110), true},
		{"living_street", kmhToMs(10), true},
		{"walk", kmhToMs(6), true},

		// Country-prefixed
		{"DE:urban", kmhToMs(50), true},
		{"DE:rural", kmhToMs(100), true},
		{"DE:motorway", kmhToMs(130), true},
		{"DE:living_street", kmhToMs(7), true},
		{"GB:nsl_single", kmhToMs(96), true},
		{"GB:motorway", kmhToMs(112), true},
		{"US:urban", kmhToMs(40), true},
		{"FR:rural", kmhToMs(80), true},

		// Zone30 forms
		{"DE:zone30", kmhToMs(30), true},
		{"DE:zone:30", kmhToMs(30), true},

		// Unknown
		{"", 0, false},
		{"variable", 0, false},
		{"XX:nonsense", 0, false}, // unknown country and unknown zone
		{"abc", 0, false},
	}
	for _, c := range cases {
		got, ok := parseMaxspeedValue(c.in)
		if ok != c.ok {
			t.Errorf("parseMaxspeedValue(%q) ok=%v, want %v (got=%v)", c.in, ok, c.ok, got)
			continue
		}
		if ok && !nearly(got, c.want) {
			t.Errorf("parseMaxspeedValue(%q) = %.3f m/s, want %.3f", c.in, got, c.want)
		}
	}
}

func TestParseSpeedForDirection(t *testing.T) {
	mkWayWithTags := func(tags ...string) *osm.Way {
		w := &osm.Way{}
		for i := 0; i+1 < len(tags); i += 2 {
			w.Tags = append(w.Tags, osm.Tag{Key: tags[i], Value: tags[i+1]})
		}
		return w
	}
	fallback := kmhToMs(30) // arbitrary distinctive fallback

	// 1. Plain maxspeed applies to both directions.
	{
		w := mkWayWithTags("maxspeed", "60")
		if v := parseSpeedForDirection(w, true, fallback); !nearly(v, kmhToMs(60)) {
			t.Errorf("plain maxspeed forward: want 60 km/h, got %.2f m/s", v)
		}
		if v := parseSpeedForDirection(w, false, fallback); !nearly(v, kmhToMs(60)) {
			t.Errorf("plain maxspeed backward: want 60 km/h, got %.2f m/s", v)
		}
	}

	// 2. Direction-specific overrides plain.
	{
		w := mkWayWithTags(
			"maxspeed", "60",
			"maxspeed:forward", "80",
			"maxspeed:backward", "40",
		)
		if v := parseSpeedForDirection(w, true, fallback); !nearly(v, kmhToMs(80)) {
			t.Errorf("forward override: want 80 km/h, got %.2f", v)
		}
		if v := parseSpeedForDirection(w, false, fallback); !nearly(v, kmhToMs(40)) {
			t.Errorf("backward override: want 40 km/h, got %.2f", v)
		}
	}

	// 3. Only forward override; reverse falls back to plain.
	{
		w := mkWayWithTags(
			"maxspeed", "60",
			"maxspeed:forward", "100",
		)
		if v := parseSpeedForDirection(w, true, fallback); !nearly(v, kmhToMs(100)) {
			t.Errorf("forward override (no backward): want 100, got %.2f", v)
		}
		if v := parseSpeedForDirection(w, false, fallback); !nearly(v, kmhToMs(60)) {
			t.Errorf("backward fallback to plain: want 60, got %.2f", v)
		}
	}

	// 4. No tags at all → fallback.
	{
		w := mkWayWithTags()
		if v := parseSpeedForDirection(w, true, fallback); !nearly(v, fallback) {
			t.Errorf("no tags: want fallback, got %.2f", v)
		}
	}

	// 5. Implicit zone tag.
	{
		w := mkWayWithTags("maxspeed", "DE:urban")
		if v := parseSpeedForDirection(w, true, fallback); !nearly(v, kmhToMs(50)) {
			t.Errorf("DE:urban: want 50 km/h, got %.2f", v)
		}
	}
}
