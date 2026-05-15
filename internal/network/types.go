// Package network defines the immutable routable graph built from OSM data.
// Sim and renderer both read these types without locks.
package network

type (
	NodeID         uint32
	EdgeID         uint32
	IntersectionID uint32
)

// Point is a projected planar coordinate in meters (not lat/lon).
// Conversion from lat/lon happens during graph build.
type Point struct {
	X, Y float64
}

type Node struct {
	ID  NodeID
	Pos Point
}

type Lane struct {
	Index uint8
	// AllowedTurns lists outgoing edges reachable from this lane at the
	// downstream intersection. Empty means "any outgoing edge."
	AllowedTurns []EdgeID
}

type Edge struct {
	ID         EdgeID
	From, To   NodeID
	Lanes      []Lane
	Length     float64 // meters
	SpeedLimit float64 // m/s
	Geometry   []Point // polyline including endpoints
}

type Intersection struct {
	ID           IntersectionID
	NodeID       NodeID
	Incoming     []EdgeID
	Outgoing     []EdgeID
	HasSignal    bool
}

type Network struct {
	Nodes         []Node
	Edges         []Edge
	Intersections []Intersection
	Grid          *SpatialGrid // populated by netbuild
	Bounds        BoundingBox
}

type BoundingBox struct {
	MinX, MinY, MaxX, MaxY float64
}
