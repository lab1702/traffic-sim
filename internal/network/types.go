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

// Node is a vertex in the routable graph; corresponds to one OSM node.
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

// Edge is a directed road segment between two nodes. Two-way streets produce two Edges.
// SpeedLimit is in m/s; Geometry includes both endpoints.
type Edge struct {
	ID         EdgeID
	From, To   NodeID
	Lanes      []Lane
	Length     float64 // meters
	SpeedLimit float64 // m/s
	Geometry   []Point // polyline including endpoints
}

// TurnRestriction names a banned transition at an intersection. A
// vehicle that has just traversed `From` is not allowed to enter `To`.
// The router skips such transitions during pathfinding.
type TurnRestriction struct {
	From EdgeID // incoming edge ending at the intersection node
	To   EdgeID // outgoing edge starting at the intersection node
}

// Intersection is a node where edges meet. ID indexes into Network.Intersections
// and is unrelated to NodeID — ID lives in IntersectionID-space, NodeID gives
// the spatial position. Incoming and Outgoing list the edges that arrive at and
// depart from NodeID.
type Intersection struct {
	ID        IntersectionID
	NodeID    NodeID
	Incoming  []EdgeID
	Outgoing  []EdgeID
	HasSignal bool
	// BannedTurns lists (from, to) edge transitions that are forbidden
	// at this intersection. Populated at load time from config (or, in
	// future, from OSM `restriction` relations) and read-only thereafter.
	BannedTurns []TurnRestriction
}

// Network is the routable graph built once from OSM data, immutable after construction.
// Sim and renderer read it without locks. Grid is populated by netbuild.
type Network struct {
	Nodes         []Node
	Edges         []Edge
	Intersections []Intersection
	Grid          *SpatialGrid // populated by netbuild
	Bounds        BoundingBox
}

// BoundingBox describes a rectangular region in the same planar frame as Point (meters).
type BoundingBox struct {
	MinX, MinY, MaxX, MaxY float64
}
