package netbuild

import (
	"reflect"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

func TestParseTurnLaneToken(t *testing.T) {
	cases := []struct {
		in   string
		want []network.TurnCategory
	}{
		{"left", []network.TurnCategory{network.TurnLeft}},
		{"slight_left", []network.TurnCategory{network.TurnLeft}},
		{"sharp_left", []network.TurnCategory{network.TurnLeft}},
		{"merge_to_left", []network.TurnCategory{network.TurnLeft}},
		{"right", []network.TurnCategory{network.TurnRight}},
		{"slight_right", []network.TurnCategory{network.TurnRight}},
		{"sharp_right", []network.TurnCategory{network.TurnRight}},
		{"merge_to_right", []network.TurnCategory{network.TurnRight}},
		{"through", []network.TurnCategory{network.TurnStraight}},
		{"none", []network.TurnCategory{network.TurnStraight}},
		{"", []network.TurnCategory{network.TurnStraight}},
		{"through;right", []network.TurnCategory{network.TurnStraight, network.TurnRight}},
		{"left;through;right", []network.TurnCategory{network.TurnLeft, network.TurnStraight, network.TurnRight}},
		{"reverse", nil}, // dropped — U-turns not modeled
		{"floof", nil},   // dropped — unknown
	}
	for _, c := range cases {
		got := parseTurnLaneSpec(c.in)
		if !reflect.DeepEqual(got, c.want) {
			t.Errorf("parseTurnLaneSpec(%q) = %v, want %v", c.in, got, c.want)
		}
	}
}

func TestParseTurnLanesString(t *testing.T) {
	got := parseTurnLanesString("left|through|through;right")
	want := [][]network.TurnCategory{
		{network.TurnLeft},
		{network.TurnStraight},
		{network.TurnStraight, network.TurnRight},
	}
	if !reflect.DeepEqual(got, want) {
		t.Errorf("got %v, want %v", got, want)
	}
}

func TestParseTurnLanesString_Empty(t *testing.T) {
	if got := parseTurnLanesString(""); got != nil {
		t.Errorf("empty input should return nil, got %v", got)
	}
}

func TestGeometricLaneAssignment(t *testing.T) {
	L, S, R := network.TurnLeft, network.TurnStraight, network.TurnRight

	cases := []struct {
		name     string
		cats     []network.TurnCategory // set of categories present (order ignored)
		numLanes int
		want     [][]network.TurnCategory // per-lane allowed categories
	}{
		{
			name: "one-lane gets everything",
			cats: []network.TurnCategory{L, S, R}, numLanes: 1,
			want: [][]network.TurnCategory{{L, S, R}},
		},
		{
			name: "two-lane with L+S+R",
			cats: []network.TurnCategory{L, S, R}, numLanes: 2,
			want: [][]network.TurnCategory{{R, S}, {L, S}},
		},
		{
			name: "three-lane with L+S+R",
			cats: []network.TurnCategory{L, S, R}, numLanes: 3,
			want: [][]network.TurnCategory{{R}, {S}, {L}},
		},
		{
			name: "four-lane with L+S+R",
			cats: []network.TurnCategory{L, S, R}, numLanes: 4,
			want: [][]network.TurnCategory{{R}, {S}, {S}, {L}},
		},
		{
			name: "two-lane with S+R only",
			cats: []network.TurnCategory{S, R}, numLanes: 2,
			want: [][]network.TurnCategory{{R, S}, {S}},
		},
		{
			name: "two-lane with S+L only",
			cats: []network.TurnCategory{S, L}, numLanes: 2,
			want: [][]network.TurnCategory{{S}, {L, S}},
		},
		{
			name: "two-lane with single category (straight)",
			cats: []network.TurnCategory{S}, numLanes: 2,
			want: [][]network.TurnCategory{{S}, {S}},
		},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			got := assignLanesGeometric(c.cats, c.numLanes)
			if !equalAssignments(got, c.want) {
				t.Errorf("got %v, want %v", got, c.want)
			}
		})
	}
}

// equalAssignments compares two per-lane assignments treating each lane's
// category list as an unordered set.
func equalAssignments(a, b [][]network.TurnCategory) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if !sameSet(a[i], b[i]) {
			return false
		}
	}
	return true
}

func sameSet(a, b []network.TurnCategory) bool {
	if len(a) != len(b) {
		return false
	}
	seen := map[network.TurnCategory]int{}
	for _, x := range a {
		seen[x]++
	}
	for _, x := range b {
		seen[x]--
		if seen[x] < 0 {
			return false
		}
	}
	return true
}
