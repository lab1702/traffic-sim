// Package snapshot provides a single-producer/single-consumer pointer
// swap of read-only Snapshot values. Writer always writes a new value
// then atomically updates the pointer; readers always see a complete value.
package snapshot

import (
	"sync/atomic"

	"github.com/lab1702/traffic-sim/internal/network"
)

type Snapshot struct {
	Tick     uint64
	SimTime  float64
	Vehicles []VehicleView
	Signals  []SignalView
	Bounds   network.BoundingBox
}

type VehicleView struct {
	ID      uint32
	X, Y    float64
	Heading float64 // radians (atan2(dy, dx))
	Speed   float64
}

type SignalView struct {
	IntersectionID uint32
	X, Y           float64
	IsRed          bool
	IsYellow       bool
	// Mode is the operating mode of the signal (0=normal, 1=flash_a,
	// 2=flash_b, 3=off). The renderer uses this to decide whether to
	// blink (flash modes) or render dark (off mode). Numeric to avoid
	// snapshot depending on the sim package.
	Mode uint8
}

type Buffer struct {
	front atomic.Pointer[Snapshot]
}

func New() *Buffer {
	b := &Buffer{}
	b.front.Store(&Snapshot{})
	return b
}

// Publish swaps in a new snapshot. Caller must not mutate s after.
func (b *Buffer) Publish(s Snapshot) {
	b.front.Store(&s)
}

// Read returns the latest published snapshot. May be the empty snapshot
// if nothing has been published yet.
func (b *Buffer) Read() Snapshot {
	return *b.front.Load()
}
