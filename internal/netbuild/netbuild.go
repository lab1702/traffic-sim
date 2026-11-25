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
	// selfJunction marks nodes a single way passes through twice, other than
	// the first/last node of a closed ring: a lollipop cul-de-sac drawn as one
	// way [A, B, C, D, E, B] meets itself at B, a real junction.
	selfJunction := make(map[osm.NodeID]bool)
	for _, w := range feat.Ways {
		seen := make(map[osm.NodeID]bool)
		last := len(w.Nodes) - 1
		for i, n := range w.Nodes {
			if !seen[n.ID] {
				usageCount[n.ID]++
				seen[n.ID] = true
			} else if i != last || n.ID != w.Nodes[0].ID {
				selfJunction[n.ID] = true
			}
		}
	}

	// A node is treated as a real intersection if it is shared by ≥2
	// distinct ways, met twice by one way (selfJunction), OR it carries the
	// `highway=traffic_signals` tag.
	//
	// Note: nodes tagged `highway=stop` or `highway=give_way` on a SINGLE
	// way are NOT promoted here — they live as interior shaping nodes and
	// are honored by applyInteriorNodeSign, which walks each approach to
	// find the closest interior sign. Promoting them would split the way
	// and bypass that walking logic.
	isIntersection := func(id osm.NodeID) bool {
		if usageCount[id] >= 2 || selfJunction[id] {
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
	var edgeIsForward []bool
	// edgeSpan[i] holds the positions in its way's node list of edge i's From
	// and To nodes. A closed way lists its first node twice, so searching the
	// list for an edge's end node finds the wrong occurrence for the segment
	// that closes the ring; the sign and restriction resolvers read this
	// instead.
	var edgeSpan [][2]int
	report := Report{}
	for _, w := range feat.Ways {
		spans := splitAtIntersections(w, isIntersection)
		dir := onewayDirection(w)
		// A real roundabout is always one-way; only flag circulating segments
		// when the way actually built one-way. This guards malformed data
		// (junction=roundabout + explicit oneway=no) from getting ring
		// priority in both directions — it degrades to a normal two-way road.
		rab := isRoundabout(w) && dir != onewayTwoWay
		hwType := highwayType(w)
		def := defaultsFor(hwType)
		class := classOf(hwType)
		// Forward and reverse may have different `maxspeed:forward`/`backward`
		// tags. Resolve both up-front; fall back to the highway-type default.
		// Same for lane counts.
		speedFwd := parseSpeedForDirection(w, true, def.SpeedLimit)
		speedBwd := parseSpeedForDirection(w, false, def.SpeedLimit)
		lanesFwd, lanesBwd := parseLanesPerDirection(w, def.LanesPerDir)
		width := wayWidthMeters(w, dir, lanesFwd, lanesBwd)

		for _, span := range spans {
			seg := w.Nodes[span[0] : span[1]+1]
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
			switch dir {
			case onewayForward:
				edges = append(edges, network.Edge{
					ID: network.EdgeID(len(edges)), From: fromID, To: toID,
					Lanes: makeLanes(lanesFwd), Length: length, SpeedLimit: speedFwd,
					Width: width, Class: class, Roundabout: rab, Geometry: geom,
				})
				osmWayOfEdge = append(osmWayOfEdge, w.ID)
				edgeIsForward = append(edgeIsForward, true)
				edgeSpan = append(edgeSpan, span)
			case onewayReverse:
				// oneway=-1 / reverse: the way's tagged direction is opposite
				// to its node order. Emit exactly one edge, but flipped, and
				// use the `backward` direction speed/lanes since that's the
				// actual direction of traffic flow.
				revGeom := reverseGeom(geom)
				edges = append(edges, network.Edge{
					ID: network.EdgeID(len(edges)), From: toID, To: fromID,
					Lanes: makeLanes(lanesBwd), Length: length, SpeedLimit: speedBwd,
					Width: width, Class: class, Roundabout: rab, Geometry: revGeom,
				})
				osmWayOfEdge = append(osmWayOfEdge, w.ID)
				// Tag as the way's "forward" for turn-lane purposes (the
				// reversed-traffic direction is what the way's natural
				// `turn:lanes` tag describes, since OSM tags follow the
				// physical traffic flow, not the node order). Empirically
				// uncommon; treating it as forward is conservative.
				edgeIsForward = append(edgeIsForward, true)
				edgeSpan = append(edgeSpan, [2]int{span[1], span[0]})
			case onewayTwoWay:
				edges = append(edges, network.Edge{
					ID: network.EdgeID(len(edges)), From: fromID, To: toID,
					Lanes: makeLanes(lanesFwd), Length: length, SpeedLimit: speedFwd,
					Width: width, Class: class, Roundabout: rab, Geometry: geom,
				})
				osmWayOfEdge = append(osmWayOfEdge, w.ID)
				edgeIsForward = append(edgeIsForward, true)
				edgeSpan = append(edgeSpan, span)
				revGeom := reverseGeom(geom)
				edges = append(edges, network.Edge{
					ID: network.EdgeID(len(edges)), From: toID, To: fromID,
					Lanes: makeLanes(lanesBwd), Length: length, SpeedLimit: speedBwd,
					Width: width, Class: class, Roundabout: rab, Geometry: revGeom,
				})
				osmWayOfEdge = append(osmWayOfEdge, w.ID)
				edgeIsForward = append(edgeIsForward, false)
				edgeSpan = append(edgeSpan, [2]int{span[1], span[0]})
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
	nodes, edges, intersections, osmWayOfEdge, edgeIsForward, edgeSpan, newNodeID, droppedComponents :=
		keepLargestComponent(nodes, edges, intersections, segChains, osmWayOfEdge, edgeIsForward, edgeSpan)
	report.ComponentsDropped = droppedComponents

	// Pruning renumbered the surviving nodes. Every later OSM-node lookup
	// (resolveControls' node tags and approach directions, restriction via
	// nodes, pickEdgeAt) reads osmToNet, so remap it to the new IDs and drop
	// pruned nodes. Without this, any extract whose pruned components were
	// numbered before kept nodes reads signs and restrictions from the wrong
	// OSM node. realIntersectionNetNodes and segChains also hold pre-prune
	// IDs but are consumed only by buildIntersections / keepLargestComponent
	// above; nothing after this point reads them.
	osmToNet = remapOSMToNet(osmToNet, newNodeID)

	// 6a-bis. Sort Intersection.Incoming by source-OSM-way highway priority
	// so the sim's "lower index = higher priority" yield rule maps to road
	// class. Without this, a residential side street can outrank a main
	// arterial purely because of edge-ordering accident.
	sortIncomingByPriority(intersections, osmWayOfEdge, feat)

	// Resolve opposing approaches (which approaches form a through road).
	// Runs BEFORE resolveControls so the class-based fallback can tell a
	// through road from a terminating stem. Needs edge geometry; build a
	// partial *Network containing just edges.
	partialNet := &network.Network{Edges: edges}
	resolveOpposing(intersections, partialNet)

	// Resolve per-approach right-of-way controls. Runs after the priority
	// sort (so each IncomingControl[i] aligns with the final sorted position
	// of Incoming[i]) and after resolveOpposing (so the fallback can read
	// x.Opposing).
	osmToNetReverse := make(map[network.NodeID]osm.NodeID, len(osmToNet))
	for k, v := range osmToNet {
		osmToNetReverse[v] = k
	}
	osmNodeOf := func(nid network.NodeID) (osm.NodeID, bool) {
		o, ok := osmToNetReverse[nid]
		return o, ok
	}
	resolveControls(intersections, feat, osmWayOfEdge, edgeSpan, osmNodeOf, edges)

	// 6b. Resolve OSM turn restriction relations to BannedTurns on the
	// intersections (writes through pointers into the slice).
	report.RestrictionsApplied, report.RestrictionsSkipped =
		applyOSMRestrictions(intersections, edges, osmWayOfEdge, edgeSpan, osmToNet,
			feat, osmNodeOf, feat.Restrictions)

	// 6c. Populate Lane.AllowedTurns per the turn-aware-lane-choice design.
	// Done after restrictions so BannedTurns is authoritative.
	tmpNet := &network.Network{Nodes: nodes, Edges: edges, Intersections: intersections}
	populateAllowedTurns(tmpNet, feat, osmWayOfEdge, edgeIsForward, edgeSpan)

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

// onewayDir describes a way's directionality. onewayTwoWay produces two
// edges; onewayForward/Reverse produce exactly one (with reverse flipping
// node order so the edge always flows in the direction of traffic).
type onewayDir uint8

const (
	onewayTwoWay onewayDir = iota
	onewayForward
	onewayReverse
)

// isRoundabout reports whether a way is a circulating roundabout ring.
// OSM tags these junction=roundabout (and the rarer junction=circular).
// Such ways are implicitly one-way even with no oneway tag.
func isRoundabout(w *osm.Way) bool {
	for _, t := range w.Tags {
		if t.Key == "junction" && (t.Value == "roundabout" || t.Value == "circular") {
			return true
		}
	}
	return false
}

// onewayDirection inspects the OSM way's tags and returns its directionality.
// Recognizes the full OSM convention including `oneway=-1` and `oneway=reverse`
// (way is one-way but traffic flows opposite to node order), plus the
// motorway and junction=roundabout/circular implicit-oneway rules.
func onewayDirection(w *osm.Way) onewayDir {
	for _, t := range w.Tags {
		if t.Key == "oneway" {
			switch t.Value {
			case "yes", "true", "1":
				return onewayForward
			case "-1", "reverse":
				return onewayReverse
			case "no", "false", "0":
				return onewayTwoWay
			}
		}
	}
	// junction=roundabout/circular is implicitly one-way (forward) unless an
	// explicit oneway tag above already decided. Checked before the motorway
	// rule; both are implicit-oneway sources.
	if isRoundabout(w) {
		return onewayForward
	}
	for _, t := range w.Tags {
		if t.Key == "highway" && t.Value == "motorway" {
			return onewayForward
		}
	}
	return onewayTwoWay
}

// parseLanesPerDirection returns (forwardLanes, backwardLanes). It honors
// the OSM convention:
//
//   - `lanes:forward` / `lanes:backward` are direction-specific overrides.
//   - `lanes` is total across both directions (halved for two-way). When a
//     two-way way also tags one direction, the other direction has the rest
//     of the total, less any centre lanes both directions share
//     (`lanes:both_ways`): lanes=4 + lanes:forward=3 is three lanes forward
//     and one backward, not half of four.
//
// For one-way ways the irrelevant-direction count is the same as the
// relevant one (callers ignore it). Falls back to `fallback` for either
// direction whose tag is missing.
func parseLanesPerDirection(w *osm.Way, fallback uint8) (uint8, uint8) {
	dir := onewayDirection(w)
	var fwdSpecific, bwdSpecific uint8
	var hasFwd, hasBwd bool
	var totalLanes, bothWays int
	var hasTotal bool
	for _, t := range w.Tags {
		switch t.Key {
		case "lanes":
			if n, err := strconv.Atoi(t.Value); err == nil && n > 0 && n < 16 {
				totalLanes = n
				hasTotal = true
			}
		case "lanes:forward":
			if n, err := strconv.Atoi(t.Value); err == nil && n > 0 && n < 16 {
				fwdSpecific = uint8(n)
				hasFwd = true
			}
		case "lanes:backward":
			if n, err := strconv.Atoi(t.Value); err == nil && n > 0 && n < 16 {
				bwdSpecific = uint8(n)
				hasBwd = true
			}
		case "lanes:both_ways":
			if n, err := strconv.Atoi(t.Value); err == nil && n > 0 && n < 16 {
				bothWays = n
			}
		}
	}
	fwd, bwd := fallback, fallback
	if hasTotal {
		if dir == onewayTwoWay {
			half := totalLanes / 2
			if half < 1 {
				half = 1
			}
			fwd, bwd = uint8(half), uint8(half)
		} else {
			fwd, bwd = uint8(totalLanes), uint8(totalLanes)
		}
	}
	if hasFwd {
		fwd = fwdSpecific
	}
	if hasBwd {
		bwd = bwdSpecific
	}
	if dir == onewayTwoWay && hasTotal && hasFwd != hasBwd {
		tagged := fwdSpecific
		if hasBwd {
			tagged = bwdSpecific
		}
		rest := totalLanes - bothWays - int(tagged)
		if rest < 1 {
			rest = 1 // inconsistent tags: keep the road two-way
		}
		if hasFwd {
			bwd = uint8(rest)
		} else {
			fwd = uint8(rest)
		}
	}
	return fwd, bwd
}

func makeLanes(n uint8) []network.Lane {
	out := make([]network.Lane, n)
	for i := range out {
		out[i] = network.Lane{Index: uint8(i)}
	}
	return out
}

// splitAtIntersections splits a way at its interior intersection nodes and
// returns each segment as the positions of its first and last node in
// w.Nodes (inclusive).
func splitAtIntersections(w *osm.Way, isX func(osm.NodeID) bool) [][2]int {
	if len(w.Nodes) == 0 {
		return nil
	}
	var segs [][2]int
	start := 0
	for i := 1; i < len(w.Nodes)-1; i++ {
		if isX(w.Nodes[i].ID) {
			segs = append(segs, [2]int{start, i})
			start = i
		}
	}
	return append(segs, [2]int{start, len(w.Nodes) - 1})
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
		ctrl := make([]network.Control, len(incE))
		opp := make([]int8, len(incE))
		for k := range opp {
			opp[k] = -1
		}
		// Defaults: ControlNone for every approach; Opposing[i] = -1.
		// Real values are assigned later in resolveControls and
		// resolveOpposing (both called after sortIncomingByPriority).
		xs = append(xs, network.Intersection{
			ID:              network.IntersectionID(len(xs)),
			NodeID:          n.ID,
			Incoming:        incE,
			IncomingControl: ctrl,
			Opposing:        opp,
			Outgoing:        outE,
			HasSignal:       signalNodes[n.ID],
		})
	}
	return xs
}

// remapOSMToNet rewrites an OSM→network node map through a pre→post-prune
// node renumbering. OSM nodes whose network node was pruned are dropped.
func remapOSMToNet(osmToNet map[osm.NodeID]network.NodeID,
	newNodeID map[network.NodeID]network.NodeID,
) map[osm.NodeID]network.NodeID {
	out := make(map[osm.NodeID]network.NodeID, len(newNodeID))
	for osmID, oldID := range osmToNet {
		if newID, ok := newNodeID[oldID]; ok {
			out[osmID] = newID
		}
	}
	return out
}

// keepLargestComponent runs an undirected BFS/Union-Find over edges and
// retains only nodes/edges/intersections in the largest connected set.
// segChains provides the full ordered node ID sequences for each segment so
// interior shaping nodes (not From/To endpoints) are correctly grouped with
// their segment's component.
//
// When several components tie for largest, the one containing the lowest
// node ID wins (node IDs follow input way order, so that is the component
// listed first). This keeps the built network — and its hash — identical
// across runs instead of depending on map iteration order.
//
// Returns the new slices, the old→new node ID map for surviving nodes (pruned
// nodes are absent), and the number of dropped components.
func keepLargestComponent(nodes []network.Node, edges []network.Edge,
	xs []network.Intersection, segChains [][]network.NodeID,
	osmWayOfEdge []osm.WayID, edgeIsForward []bool, edgeSpan [][2]int,
) ([]network.Node, []network.Edge, []network.Intersection, []osm.WayID, []bool, [][2]int, map[network.NodeID]network.NodeID, int) {
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
	// Scan in node-ID order (not map order) and require a strictly larger
	// size to replace the incumbent, so ties go to the component holding the
	// lowest node ID.
	var bestRoot network.NodeID
	best := -1
	for i := range nodes {
		r := find(network.NodeID(i))
		if s := size[r]; s > best {
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
	var newEdgeIsForward []bool
	var newEdgeSpan [][2]int
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
		if i < len(edgeIsForward) {
			newEdgeIsForward = append(newEdgeIsForward, edgeIsForward[i])
		}
		if i < len(edgeSpan) {
			newEdgeSpan = append(newEdgeSpan, edgeSpan[i])
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
		// Resize IncomingControl and Opposing to match the new Incoming
		// length. Pruning can drop incoming edges from surviving
		// intersections (e.g. an isolated spur), and the downstream
		// sort/resolve passes rely on len(IncomingControl) == len(Incoming).
		// Reset to defaults (ControlNone / -1); real values are filled later.
		x.IncomingControl = make([]network.Control, len(x.Incoming))
		x.Opposing = make([]int8, len(x.Incoming))
		for k := range x.Opposing {
			x.Opposing[k] = -1
		}
		newXs = append(newXs, x)
	}
	dropped := len(size) - 1
	if dropped < 0 {
		dropped = 0
	}
	return newNodes, newEdges, newXs, newOsmWayOf, newEdgeIsForward, newEdgeSpan, newNodeID, dropped
}
