package render

import (
	"fmt"
	"sort"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
)

// hudLineHeight is the vertical spacing between text lines drawn by
// ebitenutil.DebugPrintAt (its built-in font is roughly 16 px tall).
const hudLineHeight = 16

// hudLineCount is the number of lines DrawHUD renders, used by callers to
// offset content drawn below the HUD.
const hudLineCount = 5

// speedStats is a min/max/mean/median summary of vehicle speeds.
type speedStats struct {
	min, max, mean, median float64
	n                      int
}

// computeSpeedStats summarizes the speed distribution of `vehicles`. For
// an empty input every field is zero. Sort cost is O(N log N) per frame;
// fine for typical N <= a few thousand.
func computeSpeedStats(vehicles []snapshot.VehicleView) speedStats {
	if len(vehicles) == 0 {
		return speedStats{}
	}
	speeds := make([]float64, len(vehicles))
	var sum float64
	for i, v := range vehicles {
		speeds[i] = v.Speed
		sum += v.Speed
	}
	sort.Float64s(speeds)
	s := speedStats{
		min:  speeds[0],
		max:  speeds[len(speeds)-1],
		mean: sum / float64(len(speeds)),
		n:    len(speeds),
	}
	mid := len(speeds) / 2
	if len(speeds)%2 == 1 {
		s.median = speeds[mid]
	} else {
		s.median = (speeds[mid-1] + speeds[mid]) / 2
	}
	return s
}

// DrawHUD renders text overlay (sim time, vehicle count, FPS, view size
// in world meters, vehicle speed stats). viewWidthM/viewHeightM are
// window dimensions divided by the current zoom and indicate how many
// meters of world are visible.
func DrawHUD(screen *ebiten.Image, simTime float64, vehicleCount int, incidentCount int, viewWidthM, viewHeightM float64, stats speedStats) {
	line1 := fmt.Sprintf("sim t=%.1fs  vehicles=%d", simTime, vehicleCount)
	line2 := fmt.Sprintf("FPS=%.1f  TPS=%.1f", ebiten.ActualFPS(), ebiten.ActualTPS())
	line3 := fmt.Sprintf("view: %.0f m wide x %.0f m tall", viewWidthM, viewHeightM)
	line4 := fmt.Sprintf("speed (m/s): min=%.1f  max=%.1f  mean=%.1f  median=%.1f",
		stats.min, stats.max, stats.mean, stats.median)
	line5 := fmt.Sprintf("incidents=%d  (shift+click an edge to cycle)", incidentCount)
	ebitenutil.DebugPrintAt(screen, line1, 8, 8)
	ebitenutil.DebugPrintAt(screen, line2, 8, 8+hudLineHeight)
	ebitenutil.DebugPrintAt(screen, line3, 8, 8+2*hudLineHeight)
	ebitenutil.DebugPrintAt(screen, line4, 8, 8+3*hudLineHeight)
	ebitenutil.DebugPrintAt(screen, line5, 8, 8+4*hudLineHeight)
}

// DrawSelectionPanel renders an info block for the currently selected
// intersection starting at the given Y. Shows static config (signal
// presence, approach counts, banned turns) plus the live signal mode
// pulled from the snapshot. Lines start at x=8 below the standard HUD.
func DrawSelectionPanel(screen *ebiten.Image, net *network.Network, snap snapshot.Snapshot, id network.IntersectionID, startY int) {
	if int(id) >= len(net.Intersections) {
		return
	}
	x := &net.Intersections[id]
	nodePos := net.Nodes[x.NodeID].Pos

	lines := []string{
		fmt.Sprintf("Intersection #%d", id),
		fmt.Sprintf("  node=%d @ (%.1f, %.1f) m", x.NodeID, nodePos.X, nodePos.Y),
		fmt.Sprintf("  approaches: %d in / %d out", len(x.Incoming), len(x.Outgoing)),
	}

	// Signal mode is mutable state; read from the snapshot if signalized.
	if x.HasSignal {
		mode := "normal"
		for _, sv := range snap.Signals {
			if sv.IntersectionID == uint32(id) {
				mode = modeNameOf(sv.Mode)
				break
			}
		}
		lines = append(lines, fmt.Sprintf("  signal: %s", mode))
	} else {
		lines = append(lines, "  signal: none")
	}

	if len(x.BannedTurns) > 0 {
		counts := map[network.TurnCategory]int{}
		for _, tr := range x.BannedTurns {
			c := network.ClassifyTurn(net, tr.From, tr.To)
			counts[c]++
		}
		lines = append(lines, fmt.Sprintf("  banned turns: %d", len(x.BannedTurns)))
		// Iterate in a stable order so the panel doesn't reshuffle.
		for _, c := range []network.TurnCategory{
			network.TurnLeft, network.TurnRight, network.TurnUTurn, network.TurnStraight,
		} {
			if n := counts[c]; n > 0 {
				lines = append(lines, fmt.Sprintf("    no %s: %d", turnCategoryName(c), n))
			}
		}
	} else {
		lines = append(lines, "  banned turns: none")
	}

	for i, line := range lines {
		ebitenutil.DebugPrintAt(screen, line, 8, startY+i*hudLineHeight)
	}
}

func modeNameOf(mode uint8) string {
	switch mode {
	case modeNormal:
		return "normal"
	case modeFlashA:
		return "flash_a"
	case modeFlashB:
		return "flash_b"
	case modeOff:
		return "off"
	}
	return "?"
}

func turnCategoryName(c network.TurnCategory) string {
	switch c {
	case network.TurnLeft:
		return "left_turn"
	case network.TurnRight:
		return "right_turn"
	case network.TurnUTurn:
		return "u_turn"
	case network.TurnStraight:
		return "straight_on"
	}
	return "?"
}
