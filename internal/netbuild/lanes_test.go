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
