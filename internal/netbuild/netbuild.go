// Package netbuild constructs an immutable network.Network from parsed
// OSM features. It projects lat/lon to a local planar frame, splits ways
// at intersections, infers lanes/speeds, identifies signal nodes, and
// prunes disconnected components.
package netbuild

import (
	"fmt"
	"log/slog"
	"math"
	"sort"
	"strconv"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/paulmach/osm"
)

type Report struct {
	WaysSkipped          int
	ComponentsDropped    int
	RestrictionsApplied  int
	RestrictionsSkipped  int
}

// Build constructs a Network. Returns the network and a report of what was
// dropped during construction.
func Build(feat *osmload.Features) (*network.Network, Report, error) {
	if len(feat.Ways) == 0 {
		return nil, Report{}, fmt.Errorf("no drivable ways in input")
	}

	// 1. Compute reference point (centroid of node bounding box) for projection.
	refLat, refLon := refPoint(feat)

	// 2. Count how many distinct ways reference each node; a node shared
	// by >=2 ways is a real intersection.
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
	// segChains records the full ordered netID chain for each segment so
	// keepLargestComponent can union interior shaping nodes with their edge.
	var segChains [][]network.NodeID
	// osmWayOfEdge tracks which OSM way each edge was derived from, so that
	// restriction relations referencing OSM way IDs can later be resolved
	// to internal EdgeIDs (post-prune).
	var osmWayOfEdge []osm.WayID
	report := Report{}
	for _, w := range feat.Ways {
		segs := splitAtIntersections(w, isIntersection)
		oneway := isOneway(w)
		hwType := highwayType(w)
		def := defaultsFor(hwType)
		// Forward and reverse may have different `maxspeed:forward`/`backward`
		// tags. Resolve both up-front; fall back to the highway-type default.
		speedFwd := parseSpeedForDirection(w, true, def.SpeedLimit)
		speedBwd := parseSpeedForDirection(w, false, def.SpeedLimit)
		lanesPerDir := parseLanes(w, def.LanesPerDir)

		for _, seg := range segs {
			if len(seg) < 2 {
				report.WaysSkipped++
				continue
			}
			geom := make([]network.Point, 0, len(seg))
			chain := make([]network.NodeID, 0, len(seg))
			var fromID, toID network.NodeID
			ok := true
			for i, ref := range seg {
				netID, found := osmToNet[ref.ID]
				if !found {
					ok = false
					break
				}
				geom = append(geom, nodes[netID].Pos)
				chain = append(chain, netID)
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
			segChains = append(segChains, chain)
			edges = append(edges, network.Edge{
				ID: network.EdgeID(len(edges)), From: fromID, To: toID,
				Lanes: makeLanes(lanesPerDir), Length: length, SpeedLimit: speedFwd, Geometry: geom,
			})
			osmWayOfEdge = append(osmWayOfEdge, w.ID)
			if !oneway {
				revGeom := reverseGeom(geom)
				edges = append(edges, network.Edge{
					ID: network.EdgeID(len(edges)), From: toID, To: fromID,
					Lanes: makeLanes(lanesPerDir), Length: length, SpeedLimit: speedBwd, Geometry: revGeom,
				})
				osmWayOfEdge = append(osmWayOfEdge, w.ID)
			}
		}
	}

	// 5. Build intersections (one per node that is a real intersection
	// per usageCount, plus signal nodes).
	realIntersectionNetNodes := make(map[network.NodeID]bool)
	for osmID := range feat.Nodes {
		if isIntersection(osmID) {
			if netID, ok := osmToNet[osmID]; ok {
				realIntersectionNetNodes[netID] = true
			}
		}
	}
	intersections := buildIntersections(nodes, edges, feat, osmToNet, realIntersectionNetNodes)

	// 6. Prune to largest connected component.
	nodes, edges, intersections, osmWayOfEdge, droppedComponents := keepLargestComponent(nodes, edges, intersections, segChains, osmWayOfEdge)
	report.ComponentsDropped = droppedComponents

	// 6b. Resolve OSM turn restriction relations to BannedTurns on the
	// intersections (writes through pointers into the slice).
	report.RestrictionsApplied, report.RestrictionsSkipped =
		applyOSMRestrictions(intersections, edges, osmWayOfEdge, osmToNet, feat.Restrictions)

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
		"restrictions_applied", report.RestrictionsApplied,
		"restrictions_skipped", report.RestrictionsSkipped,
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
// (centroid of all loaded node positions). Iterates nodes in sorted ID
// order so that floating-point summation is deterministic across runs.
func refPoint(feat *osmload.Features) (lat, lon float64) {
	if len(feat.Nodes) == 0 {
		return 0, 0
	}
	ids := make([]osm.NodeID, 0, len(feat.Nodes))
	for id := range feat.Nodes {
		ids = append(ids, id)
	}
	sort.Slice(ids, func(i, j int) bool { return ids[i] < ids[j] })
	var sumLat, sumLon float64
	for _, id := range ids {
		n := feat.Nodes[id]
		sumLat += n.Lat
		sumLon += n.Lon
	}
	n := float64(len(ids))
	return sumLat / n, sumLon / n
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
	realIntersectionNetNodes map[network.NodeID]bool,
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
		isReal := realIntersectionNetNodes[n.ID]
		if !isReal && !signalNodes[n.ID] {
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
// segChains provides the full ordered node ID sequences for each segment so
// interior shaping nodes (not From/To endpoints) are correctly grouped with
// their segment's component.
// Returns the new slices and the number of dropped components.
func keepLargestComponent(nodes []network.Node, edges []network.Edge,
	xs []network.Intersection, segChains [][]network.NodeID, osmWayOfEdge []osm.WayID,
) ([]network.Node, []network.Edge, []network.Intersection, []osm.WayID, int) {
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
	// Union all consecutive nodes in each segment chain so that interior
	// shaping nodes are connected to their segment's component.
	for _, chain := range segChains {
		for i := 1; i < len(chain); i++ {
			union(chain[i-1], chain[i])
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
	var newOsmWayOf []osm.WayID
	for i, e := range edges {
		if !keep(e.From) || !keep(e.To) {
			continue
		}
		e.ID = network.EdgeID(len(newEdges))
		e.From = newNodeID[e.From]
		e.To = newNodeID[e.To]
		newEdges = append(newEdges, e)
		if i < len(osmWayOfEdge) {
			newOsmWayOf = append(newOsmWayOf, osmWayOfEdge[i])
		}
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
	return newNodes, newEdges, newXs, newOsmWayOf, dropped
}
