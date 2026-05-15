package render

import (
	"fmt"
	"image/color"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
)

// DrawHUD renders text overlay (sim time, vehicle count, FPS).
func DrawHUD(screen *ebiten.Image, simTime float64, vehicleCount int) {
	line1 := fmt.Sprintf("sim t=%.1fs  vehicles=%d", simTime, vehicleCount)
	line2 := fmt.Sprintf("FPS=%.1f  TPS=%.1f", ebiten.ActualFPS(), ebiten.ActualTPS())
	ebitenutil.DebugPrintAt(screen, line1, 8, 8)
	ebitenutil.DebugPrintAt(screen, line2, 8, 24)
	_ = color.White
}
