package snapshot

import (
	"sync/atomic"
	"testing"
	"time"
)

func TestDoubleBuffer_SwapIsAtomic(t *testing.T) {
	b := New()
	var stop atomic.Bool

	// Writer goroutine: fills back-buffer with monotonically increasing Tick.
	go func() {
		var k uint64
		for !stop.Load() {
			s := Snapshot{Tick: k}
			b.Publish(s)
			k++
		}
	}()

	// Reader: reads many times, asserts monotonicity.
	var lastTick uint64
	deadline := time.Now().Add(200 * time.Millisecond)
	for time.Now().Before(deadline) {
		got := b.Read()
		if got.Tick < lastTick {
			t.Fatalf("non-monotonic tick: %d < %d", got.Tick, lastTick)
		}
		lastTick = got.Tick
	}
	stop.Store(true)
}
