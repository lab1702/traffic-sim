// Package network defines the immutable routable graph built from OSM data.
// Sim and renderer both read these types without locks.
package network

// LaneWidthMeters is the assumed per-lane width when an OSM way has no
// explicit `width=*` tag. Used both by netbuild (to estimate Edge.Width)
// and by the renderer (to offset vehicles laterally within their lane).
const LaneWidthMeters = 3.6

// RoadClass names the OSM `highway=*` tier each edge belongs to. The
// renderer maps classes to colors and to draw order (higher tiers render
// on top). Link variants (motorway_link, trunk_link, …) collapse onto
// their parent tier so a flyover ramp shares its mainline's color.
type RoadClass uint8

const (
	ClassUnknown RoadClass = iota
	ClassMotorway
	ClassTrunk
	ClassPrimary
	ClassSecondary
	ClassTertiary
	ClassResidential
	ClassUnclassified
	ClassService
	ClassLivingStreet
)

// Priority returns a draw-order weight: higher tiers draw last (on top).
// Used by the renderer to keep arterials visible where they cross local
// streets, mirroring the convention in standard OSM stylesheets.
func (c RoadClass) Priority() int {
	switch c {
	case ClassMotorway:
		return 9
	case ClassTrunk:
		return 8
	case ClassPrimary:
		return 7
	case ClassSecondary:
		return 6
	case ClassTertiary:
		return 5
	case ClassResidential:
		return 4
	case ClassUnclassified:
		return 3
	case ClassLivingStreet:
		return 2
	case ClassService:
		return 1
	}
	return 0
}

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
//
// Width is the total physical width of the road in meters, taken from the
// OSM `width=*` tag when present or estimated from lane count otherwise.
// For two-way roads, both directions' edges carry the same total width so
// the renderer can paint a single band over the shared centerline. Width
// is for rendering only; the simulation does not consult it.
type Edge struct {
	ID         EdgeID
	From, To   NodeID
	Lanes      []Lane
	Length     float64 // meters
	SpeedLimit float64 // m/s
	Width      float64 // meters (total road width)
	Class      RoadClass
	Geometry   []Point // polyline including endpoints
}

// TurnRestriction names a banned transition at an intersection. A
// vehicle that has just traversed `From` is not allowed to enter `To`.
// The router skips such transitions during pathfinding.
type TurnRestriction struct {
	From EdgeID // incoming edge ending at the intersection node
	To   EdgeID // outgoing edge starting at the intersection node
}

// Control names the right-of-way rule that governs a specific incoming
// approach at an intersection. The values are intentionally ordered so
// that a higher numeric value is a stricter control: a stop is stricter
// than a yield, an all-way stop is stricter still.
type Control uint8

const (
	ControlNone        Control = iota // through-movement, no sign
	ControlYield                      // yield sign — slow, no mandatory stop
	ControlStop                       // stop sign — mandatory dwell, then gap-accept
	ControlAllWayStop                 // all-way stop — Stop + FIFO arbitration
)

// Intersection is a node where edges meet. ID indexes into Network.Intersections
// and is unrelated to NodeID — ID lives in IntersectionID-space, NodeID gives
// the spatial position. Incoming and Outgoing list the edges that arrive at and
// depart from NodeID.
type Intersection struct {
	ID        IntersectionID
	NodeID    NodeID
	Incoming  []EdgeID
	// IncomingControl is parallel to Incoming: IncomingControl[i] is the
	// right-of-way rule for approach Incoming[i]. The two slices have
	// equal length. Populated by netbuild after sortIncomingByPriority.
	IncomingControl []Control
	// Opposing is parallel to Incoming: Opposing[i] is the position of
	// approach i's opposing approach (the same road's other direction),
	// or -1 if none. Populated by netbuild after sortIncomingByPriority.
	// Symmetric: Opposing[Opposing[i]] == i whenever Opposing[i] != -1.
	Opposing []int8
	Outgoing []EdgeID
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
