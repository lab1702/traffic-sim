package osmload

// Regression tests for the fifth code review. Each fails before its fix.

import (
	"testing"

	"github.com/paulmach/osm"
)

// TestIsHighway_AccessAndArea: ways closed to motor cars, and highway areas
// (the outline of a square or yard), were loaded as roads, so cars were
// routed through pedestrian zones and bus gates and round area outlines.
func TestIsHighway_AccessAndArea(t *testing.T) {
	for _, tc := range []struct {
		tags []string
		want bool
	}{
		{[]string{"highway", "residential"}, true},
		{[]string{"highway", "residential", "motor_vehicle", "no"}, false},
		{[]string{"highway", "service", "access", "no"}, false},
		{[]string{"highway", "unclassified", "vehicle", "no"}, false},
		{[]string{"highway", "unclassified", "motorcar", "no"}, false},
		{[]string{"highway", "track", "motor_vehicle", "agricultural"}, false}, // not drivable anyway
		{[]string{"highway", "unclassified", "motor_vehicle", "agricultural;forestry"}, false},
		// The most specific key decides.
		{[]string{"highway", "residential", "access", "no", "motor_vehicle", "yes"}, true},
		{[]string{"highway", "residential", "motor_vehicle", "yes", "motorcar", "no"}, false},
		{[]string{"highway", "service", "access", "private"}, true},
		{[]string{"highway", "residential", "motor_vehicle", "destination"}, true},
		{[]string{"highway", "service", "area", "yes"}, false},
		{[]string{"highway", "residential", "area", "no"}, true},
		{[]string{"highway", "footway"}, false},
	} {
		w := &osm.Way{ID: 1}
		for i := 0; i+1 < len(tc.tags); i += 2 {
			w.Tags = append(w.Tags, osm.Tag{Key: tc.tags[i], Value: tc.tags[i+1]})
		}
		if got := isHighway(w); got != tc.want {
			t.Errorf("isHighway(%v) = %v, want %v", tc.tags, got, tc.want)
		}
	}
}
