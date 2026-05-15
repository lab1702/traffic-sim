// Package netbuild constructs an immutable network.Network from parsed
// OSM features. It projects lat/lon to a local planar frame, splits ways
// at intersections, infers lanes/speeds, identifies signal nodes, and
// prunes disconnected components.
package netbuild

import (
	"fmt"
	"log/slog"
	"math"
	"strconv"
	"strings"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/paulmach/osm"
)

type Report struct {
	WaysSkipped         int
	ComponentsDropped   int
	NodesDroppedNoEdges int
}

// Build constructs a Network. Returns the network and a report of what was
// dropped during construction.
func Build(feat *osmload.Features) (*network.Network, Report, error) {
	if len(feat.Ways) == 0 {
		return nil, Report{}, fmt.Errorf("no drivable ways in input")
	}

	// 1. Compute reference point (centroid of node bounding box) for projection.
	refLat, refLon := refPoint(feat)

	// 2. Build node-degree map: how many kept ways touch each node?
	degree := make(map[osm.NodeID]int)
	for _, w := range feat.Ways {
		for i, n := range w.Nodes {
			if i == 0 || i == len(w.Nodes)-1 {
				degree[n.ID] += 1
			} else {
				// Interior nodes count as 1 unless they're endpoints in
				// other ways; we'll add those separately.
				degree[n.ID] += 0
			}
		}
	}
	// Endpoints of multiple ways become intersections.
	endpointCount := make(map[osm.NodeID]int)
	for _, w := range feat.Ways {
		if len(w.Nodes) == 0 {
			continue
		}
		endpointCount[w.Nodes[0].ID]++
		endpointCount[w.Nodes[len(w.Nodes)-1].ID]++
	}

	// A node is an intersection if (a) >=2 ways touch it as endpoint,
	// or (b) it's tagged as a traffic signal, or (c) it appears in
	// the interior of one way and as an endpoint of another. We'll
	// approximate "intersection" as: appears in >=2 ways anywhere.
	usageCount := make(map[osm.NodeID]int)
	for _, w := range feat.Ways {
		seen := make(map[osm.NodeID]bool)
		for _, n := range w.Nodes {
			if !seen[n.ID] {
				usageCount[n.ID]++
				seen[n.ID] = true
			}
		}
	}

	isIntersection := func(id osm.NodeID) bool {
		if usageCount[id] >= 2 {
			return true
		}
		if n, ok := feat.Nodes[id]; ok {
			for _, t := range n.Tags {
				if t.Key == "highway" && t.Value == "traffic_signals" {
					return true
				}
			}
		}
		return false
	}

	// 3. Allocate IDs and build node table for all nodes that will appear
	// in the final graph (intersection nodes + way endpoints + interior
	// shaping nodes).
	osmToNet := make(map[osm.NodeID]network.NodeID)
	var nodes []network.Node
	for _, w := range feat.Ways {
		for _, ref := range w.Nodes {
			if _, ok := osmToNet[ref.ID]; ok {
				continue
			}
			osmNode, ok := feat.Nodes[ref.ID]
			if !ok {
				// Way references a node we didn't load; skip.
				continue
			}
			pt := project(osmNode.Lat, osmNode.Lon, refLat, refLon)
			id := network.NodeID(len(nodes))
			osmToNet[ref.ID] = id
			nodes = append(nodes, network.Node{ID: id, Pos: pt})
		}
	}

	// 4. Split each way at intersection nodes and produce edges.
	var edges []network.Edge
	report := Report{}
	for _, w := range feat.Ways {
		segs := splitAtIntersections(w, isIntersection)
		oneway := isOneway(w)
		hwType := highwayType(w)
		def := defaultsFor(hwType)
		speed := parseSpeed(w, def.SpeedLimit)
		lanesPerDir := parseLanes(w, def.LanesPerDir)

		for _, seg := range segs {
			if len(seg) < 2 {
				report.WaysSkipped++
				continue
			}
			geom := make([]network.Point, 0, len(seg))
			var fromID, toID network.NodeID
			ok := true
			for i, ref := range seg {
				netID, found := osmToNet[ref.ID]
				if !found {
					ok = false
					break
				}
				geom = append(geom, nodes[netID].Pos)
				if i == 0 {
					fromID = netID
				}
				if i == len(seg)-1 {
					toID = netID
				}
			}
			if !ok {
				report.WaysSkipped++
				continue
			}
			length := polylineLength(geom)
			if length < 0.5 { // <0.5m: degenerate
				report.WaysSkipped++
				continue
			}
			lanes := makeLanes(lanesPerDir)
			edges = append(edges, network.Edge{
				ID: network.EdgeID(len(edges)), From: fromID, To: toID,
				Lanes: lanes, Length: length, SpeedLimit: speed, Geometry: geom,
			})
			if !oneway {
				revGeom := reverseGeom(geom)
				edges = append(edges, network.Edge{
					ID: network.EdgeID(len(edges)), From: toID, To: fromID,
					Lanes: lanes, Length: length, SpeedLimit: speed, Geometry: revGeom,
				})
			}
		}
	}

	// 5. Build intersections (one per node with degree>=2 in the final
	// edge set, plus signal nodes even if degree=1).
	intersections := buildIntersections(nodes, edges, feat, osmToNet)

	// 6. Prune to largest connected component.
	nodes, edges, intersections, droppedComponents := keepLargestComponent(nodes, edges, intersections)
	report.ComponentsDropped = droppedComponents

	// 7. Build spatial grid.
	bounds := computeBounds(nodes)
	grid := network.NewSpatialGrid(bounds, 50.0) // 50m cells
	for _, e := range edges {
		for _, p := range e.Geometry {
			grid.Insert(e.ID, p)
		}
	}

	slog.Info("graph build complete",
		"nodes", len(nodes),
		"edges", len(edges),
		"intersections", len(intersections),
		"ways_skipped", report.WaysSkipped,
		"components_dropped", report.ComponentsDropped,
	)

	return &network.Network{
		Nodes: nodes, Edges: edges,
		Intersections: intersections,
		Grid: grid, Bounds: bounds,
	}, report, nil
}

// --- helpers ---

func highwayType(w *osm.Way) string {
	for _, t := range w.Tags {
		if t.Key == "highway" {
			return t.Value
		}
	}
	return ""
}

func isOneway(w *osm.Way) bool {
	for _, t := range w.Tags {
		if t.Key == "oneway" {
			switch t.Value {
			case "yes", "true", "1":
				return true
			}
		}
		// Motorways are implicitly oneway in OSM convention.
		if t.Key == "highway" && t.Value == "motorway" {
			return true
		}
	}
	return false
}

func parseSpeed(w *osm.Way, fallback float64) float64 {
	for _, t := range w.Tags {
		if t.Key != "maxspeed" {
			continue
		}
		// Strip units; assume km/h if numeric, mph if " mph" suffix.
		v := strings.TrimSpace(t.Value)
		if strings.HasSuffix(v, "mph") {
			n, err := strconv.ParseFloat(strings.TrimSpace(strings.TrimSuffix(v, "mph")), 64)
			if err == nil {
				return n * 0.44704
			}
		}
		n, err := strconv.ParseFloat(v, 64)
		if err == nil {
			return n / 3.6 // km/h -> m/s
		}
	}
	return fallback
}

func parseLanes(w *osm.Way, fallback uint8) uint8 {
	for _, t := range w.Tags {
		if t.Key == "lanes" {
			n, err := strconv.Atoi(t.Value)
			if err == nil && n > 0 && n < 16 {
				// "lanes" in OSM is total both directions; we'll halve for
				// non-oneway. parseSpeed/parseLanes callers handle that.
				if isOneway(w) {
					return uint8(n)
				}
				half := n / 2
				if half < 1 {
					half = 1
				}
				return uint8(half)
			}
		}
	}
	return fallback
}

func makeLanes(n uint8) []network.Lane {
	out := make([]network.Lane, n)
	for i := range out {
		out[i] = network.Lane{Index: uint8(i)}
	}
	return out
}

func splitAtIntersections(w *osm.Way, isX func(osm.NodeID) bool) [][]osm.WayNode {
	var segs [][]osm.WayNode
	if len(w.Nodes) == 0 {
		return nil
	}
	start := 0
	for i := 1; i < len(w.Nodes)-1; i++ {
		if isX(w.Nodes[i].ID) {
			segs = append(segs, w.Nodes[start:i+1])
			start = i
		}
	}
	segs = append(segs, w.Nodes[start:])
	return segs
}

// refPoint picks a reference lat/lon for the local planar projection
// (centroid of all loaded node positions).
func refPoint(feat *osmload.Features) (lat, lon float64) {
	var n int
	var sumLat, sumLon float64
	for _, node := range feat.Nodes {
		sumLat += node.Lat
		sumLon += node.Lon
		n++
	}
	if n == 0 {
		return 0, 0
	}
	return sumLat / float64(n), sumLon / float64(n)
}

// project converts (lat, lon) to local planar (x, y) in meters using an
// equirectangular projection around the reference point. Good enough for
// city-scale: error <1% within tens of km.
func project(lat, lon, refLat, refLon float64) network.Point {
	const earthR = 6371000.0
	dLat := (lat - refLat) * math.Pi / 180
	dLon := (lon - refLon) * math.Pi / 180
	y := dLat * earthR
	x := dLon * earthR * math.Cos(refLat*math.Pi/180)
	return network.Point{X: x, Y: y}
}

func polylineLength(geom []network.Point) float64 {
	var d float64
	for i := 1; i < len(geom); i++ {
		dx := geom[i].X - geom[i-1].X
		dy := geom[i].Y - geom[i-1].Y
		d += math.Sqrt(dx*dx + dy*dy)
	}
	return d
}

func reverseGeom(g []network.Point) []network.Point {
	out := make([]network.Point, len(g))
	for i, p := range g {
		out[len(g)-1-i] = p
	}
	return out
}

func computeBounds(nodes []network.Node) network.BoundingBox {
	if len(nodes) == 0 {
		return network.BoundingBox{}
	}
	b := network.BoundingBox{
		MinX: nodes[0].Pos.X, MaxX: nodes[0].Pos.X,
		MinY: nodes[0].Pos.Y, MaxY: nodes[0].Pos.Y,
	}
	for _, n := range nodes {
		if n.Pos.X < b.MinX {
			b.MinX = n.Pos.X
		}
		if n.Pos.X > b.MaxX {
			b.MaxX = n.Pos.X
		}
		if n.Pos.Y < b.MinY {
			b.MinY = n.Pos.Y
		}
		if n.Pos.Y > b.MaxY {
			b.MaxY = n.Pos.Y
		}
	}
	// Pad slightly so points exactly on bounds don't fall outside grid.
	pad := 1.0
	b.MinX -= pad
	b.MinY -= pad
	b.MaxX += pad
	b.MaxY += pad
	return b
}

func buildIntersections(nodes []network.Node, edges []network.Edge,
	feat *osmload.Features, osmToNet map[osm.NodeID]network.NodeID,
) []network.Intersection {
	inc := make(map[network.NodeID][]network.EdgeID)
	out := make(map[network.NodeID][]network.EdgeID)
	for _, e := range edges {
		inc[e.To] = append(inc[e.To], e.ID)
		out[e.From] = append(out[e.From], e.ID)
	}

	signalNodes := make(map[network.NodeID]bool)
	for osmID, n := range feat.Nodes {
		for _, t := range n.Tags {
			if t.Key == "highway" && t.Value == "traffic_signals" {
				if netID, ok := osmToNet[osmID]; ok {
					signalNodes[netID] = true
				}
			}
		}
	}

	var xs []network.Intersection
	for _, n := range nodes {
		incE, outE := inc[n.ID], out[n.ID]
		if len(incE)+len(outE) < 2 && !signalNodes[n.ID] {
			continue
		}
		xs = append(xs, network.Intersection{
			ID:        network.IntersectionID(len(xs)),
			NodeID:    n.ID,
			Incoming:  incE,
			Outgoing:  outE,
			HasSignal: signalNodes[n.ID],
		})
	}
	return xs
}

// keepLargestComponent runs an undirected BFS/Union-Find over edges and
// retains only nodes/edges/intersections in the largest connected set.
// Returns the new slices and the number of dropped components.
func keepLargestComponent(nodes []network.Node, edges []network.Edge,
	xs []network.Intersection,
) ([]network.Node, []network.Edge, []network.Intersection, int) {
	parent := make([]network.NodeID, len(nodes))
	for i := range parent {
		parent[i] = network.NodeID(i)
	}
	var find func(network.NodeID) network.NodeID
	find = func(a network.NodeID) network.NodeID {
		if parent[a] != a {
			parent[a] = find(parent[a])
		}
		return parent[a]
	}
	union := func(a, b network.NodeID) {
		ra, rb := find(a), find(b)
		if ra != rb {
			parent[ra] = rb
		}
	}
	for _, e := range edges {
		union(e.From, e.To)
	}
	// Tally component sizes.
	size := make(map[network.NodeID]int)
	for i := range nodes {
		size[find(network.NodeID(i))]++
	}
	var bestRoot network.NodeID
	best := -1
	for r, s := range size {
		if s > best {
			best, bestRoot = s, r
		}
	}
	keep := func(id network.NodeID) bool { return find(id) == bestRoot }

	// Remap node IDs.
	newNodeID := make(map[network.NodeID]network.NodeID)
	var newNodes []network.Node
	for _, n := range nodes {
		if keep(n.ID) {
			newID := network.NodeID(len(newNodes))
			newNodeID[n.ID] = newID
			n.ID = newID
			newNodes = append(newNodes, n)
		}
	}
	var newEdges []network.Edge
	for _, e := range edges {
		if !keep(e.From) || !keep(e.To) {
			continue
		}
		e.ID = network.EdgeID(len(newEdges))
		e.From = newNodeID[e.From]
		e.To = newNodeID[e.To]
		newEdges = append(newEdges, e)
	}
	// Intersections must be rebuilt because edge IDs changed.
	inc := make(map[network.NodeID][]network.EdgeID)
	out := make(map[network.NodeID][]network.EdgeID)
	for _, e := range newEdges {
		inc[e.To] = append(inc[e.To], e.ID)
		out[e.From] = append(out[e.From], e.ID)
	}
	var newXs []network.Intersection
	for _, x := range xs {
		newNode, ok := newNodeID[x.NodeID]
		if !ok {
			continue
		}
		x.ID = network.IntersectionID(len(newXs))
		x.NodeID = newNode
		x.Incoming = inc[newNode]
		x.Outgoing = out[newNode]
		newXs = append(newXs, x)
	}
	dropped := len(size) - 1
	if dropped < 0 {
		dropped = 0
	}
	return newNodes, newEdges, newXs, dropped
}
