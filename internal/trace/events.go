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
	KindSimStart       Kind = 1
	KindVehicleSpawn   Kind = 2
	KindVehicleDespawn Kind = 3
	KindSignalPhase    Kind = 4
	KindMetricsTick    Kind = 5
	KindSimEnd         Kind = 6
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
