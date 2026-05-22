// Package trace defines a binary event stream format for sim playback.
//
// Wire format:
//
//	magic: "TSIM" (4 bytes)
//	version: uint16 (currently 1)
//	then a sequence of events, each:
//	  tick: uint64
//	  simTime: float64
//	  kind: uint8
//	  length: uint16 (bytes of payload that follow)
//	  payload: kind-specific
//
// All integers little-endian.
package trace

type Kind uint8

const (
	KindSimStart         Kind = 1
	KindVehicleSpawn     Kind = 2
	KindVehicleDespawn   Kind = 3
	KindSignalPhase      Kind = 4
	KindMetricsTick      Kind = 5
	KindSimEnd           Kind = 6
	KindSignalModeChange Kind = 7
	// KindTraceDropped records that the writer's backpressure channel
	// overflowed and N events were dropped before this point in the
	// stream. Lets replayers warn that the trace is incomplete.
	KindTraceDropped Kind = 8
	// KindVehicleReroute records that a vehicle replaced the tail of its route
	// at runtime (GPS rerouting around congestion).
	KindVehicleReroute Kind = 9
)

// Event is implemented by every concrete event type.
type Event interface {
	Kind() Kind
}

// Header holds the per-event tick and sim-time metadata.
type Header struct {
	Tick    uint64
	SimTime float64
}

// SimStart is the first event in every trace file.
type SimStart struct {
	SeedHi, SeedLo uint64
	NetHash        uint64 // fingerprint of the network, for replay validation
}

func (*SimStart) Kind() Kind { return KindSimStart }

// VehicleSpawn records the birth of a vehicle.
type VehicleSpawn struct {
	VehicleID  uint32
	OriginNode uint32
	DestNode   uint32
	Route      []uint32
}

func (*VehicleSpawn) Kind() Kind { return KindVehicleSpawn }

// VehicleDespawn records the removal of a vehicle.
type VehicleDespawn struct {
	VehicleID uint32
}

func (*VehicleDespawn) Kind() Kind { return KindVehicleDespawn }

// SignalPhase records a signal state change at an intersection.
type SignalPhase struct {
	IntersectionID uint32
	PhaseIdx       uint8
	IsYellow       bool
}

func (*SignalPhase) Kind() Kind { return KindSignalPhase }

// MetricsTick records aggregate metrics at a given tick.
type MetricsTick struct {
	TotalVehicles uint32
	AvgSpeed      float64
	CongestionIdx float64
}

func (*MetricsTick) Kind() Kind { return KindMetricsTick }

// SimEnd is the last event in a complete trace file.
type SimEnd struct {
	Reason string
}

func (*SimEnd) Kind() Kind { return KindSimEnd }

// SignalModeChange records a change in a signal's operating mode
// (normal/flash_a/flash_b/off). Initial non-normal modes (set via YAML
// config) are emitted as a SignalModeChange at tick 0 so replay sees
// the same starting state.
//
// The Mode field uses the same numeric values as sim.SignalMode:
// 0=normal, 1=flash_a, 2=flash_b, 3=off.
type SignalModeChange struct {
	IntersectionID uint32
	Mode           uint8
}

func (*SignalModeChange) Kind() Kind { return KindSignalModeChange }

// TraceDropped marks that the writer's bounded backpressure channel
// overflowed and Count events were dropped between the previous event in
// the stream and the next one. The (tick, simTime) header points to the
// next surviving event, not the dropped ones. Replayers should warn the
// user that the trace is missing data.
type TraceDropped struct {
	Count uint32
}

func (*TraceDropped) Kind() Kind { return KindTraceDropped }

// VehicleReroute records that a vehicle replaced the tail of its route at
// runtime (GPS rerouting around congestion). AtIndex is the route index of the
// first replaced edge; NewTail is the replacement edge sequence. Replayers
// splice route[:AtIndex] + NewTail to follow the path actually taken.
type VehicleReroute struct {
	VehicleID uint32
	AtIndex   uint32
	NewTail   []uint32
}

func (*VehicleReroute) Kind() Kind { return KindVehicleReroute }

// UnknownEvent is returned by Reader.Next when a trace contains an event
// kind this reader doesn't recognize. The wire format's per-event `length`
// field lets the reader skip over the payload without parsing it, so older
// binaries can still read traces produced by newer ones that introduce
// new kinds. Callers should typically ignore UnknownEvent values.
type UnknownEvent struct {
	KindVal Kind
	Payload []byte
}

func (e *UnknownEvent) Kind() Kind { return e.KindVal }
