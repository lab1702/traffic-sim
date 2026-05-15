package network

import "testing"

// TestClassifyTurn: build a + shaped intersection where edges arrive at
// the center node from four cardinal directions, and verify the turn
// classification for each (incoming, outgoing) pair.
//
//        N
//        |
//   W ---C--- E
//        |
//        S
func TestClassifyTurn(t *testing.T) {
	c := Point{X: 0, Y: 0}
	n := Point{X: 0, Y: 100}
	e := Point{X: 100, Y: 0}
	s := Point{X: 0, Y: -100}
	w := Point{X: -100, Y: 0}

	mkEdge := func(id int, from, to Point) Edge {
		return Edge{ID: EdgeID(id), Geometry: []Point{from, to}, Length: 100}
	}
	net := &Network{
		Edges: []Edge{
			mkEdge(0, n, c), // N->C (vehicle arrives going south)
			mkEdge(1, e, c), // E->C (arrives going west)
			mkEdge(2, s, c), // S->C (arrives going north)
			mkEdge(3, w, c), // W->C (arrives going east)
			mkEdge(4, c, n), // C->N (leaves going north)
			mkEdge(5, c, e), // C->E (leaves going east)
			mkEdge(6, c, s), // C->S (leaves going south)
			mkEdge(7, c, w), // C->W (leaves going west)
		},
	}

	cases := []struct {
		name string
		from EdgeID
		to   EdgeID
		want TurnCategory
	}{
		// From N->C (heading south), then:
		{"N then S = straight", 0, 6, TurnStraight},
		{"N then W = right turn (going right when southbound)", 0, 7, TurnRight},
		{"N then E = left turn (going left when southbound)", 0, 5, TurnLeft},
		{"N then back to N = U-turn", 0, 4, TurnUTurn},

		// From E->C (heading west), then:
		{"E then W = straight", 1, 7, TurnStraight},
		{"E then N = right (clockwise when westbound)", 1, 4, TurnRight},
		{"E then S = left", 1, 6, TurnLeft},
		{"E then E = U-turn", 1, 5, TurnUTurn},

		// From S->C (heading north), then:
		{"S then N = straight", 2, 4, TurnStraight},
		{"S then E = right", 2, 5, TurnRight},
		{"S then W = left", 2, 7, TurnLeft},

		// From W->C (heading east), then:
		{"W then E = straight", 3, 5, TurnStraight},
		{"W then S = right", 3, 6, TurnRight},
		{"W then N = left", 3, 4, TurnLeft},
	}
	for _, tc := range cases {
		got := ClassifyTurn(net, tc.from, tc.to)
		if got != tc.want {
			t.Errorf("%s: got %v, want %v (angle=%.3f rad)",
				tc.name, got, tc.want, TurnAngle(net, tc.from, tc.to))
		}
	}
}
