package render

import (
	"fmt"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
)

// hudLineHeight is the vertical spacing between text lines drawn by
// ebitenutil.DebugPrintAt (its built-in font is roughly 16 px tall).
const hudLineHeight = 16

// DrawHUD renders text overlay (sim time, vehicle count, FPS).
func DrawHUD(screen *ebiten.Image, simTime float64, vehicleCount int) {
	line1 := fmt.Sprintf("sim t=%.1fs  vehicles=%d", simTime, vehicleCount)
	line2 := fmt.Sprintf("FPS=%.1f  TPS=%.1f", ebiten.ActualFPS(), ebiten.ActualTPS())
	ebitenutil.DebugPrintAt(screen, line1, 8, 8)
	ebitenutil.DebugPrintAt(screen, line2, 8, 8+hudLineHeight)
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
