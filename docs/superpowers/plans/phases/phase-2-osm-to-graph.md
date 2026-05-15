# Phase 2 — OSM → Routable Graph

**Milestone:** `trafficsim load <file.osm | file.osm.pbf>` parses the file, builds the graph, and prints stats (`nodes=N edges=M intersections=K signals=S largest_component=...`).

---

### Task 2.1: OSM loader interface

**Files:**
- Create: `internal/osmload/osmload.go`

- [ ] **Step 1: Write the loader file**

Write `internal/osmload/osmload.go`:
```go
// Package osmload parses .osm XML and .osm.pbf files and emits the raw OSM
// features (nodes, ways, relations) needed by netbuild. Format is detected
// from the file extension.
package osmload

import (
	"context"
	"errors"
	"fmt"
	"io"
	"os"
	"strings"

	"github.com/paulmach/osm"
	"github.com/paulmach/osm/osmpbf"
	"github.com/paulmach/osm/osmxml"
)

// Features is what netbuild consumes. Only highway-tagged ways are kept;
// nodes are kept if referenced by a kept way OR tagged with traffic_signals.
type Features struct {
	Nodes map[osm.NodeID]*osm.Node
	Ways  []*osm.Way
}

// Load reads an OSM file and returns the filtered Features set.
func Load(path string) (*Features, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, fmt.Errorf("open %s: %w", path, err)
	}
	defer f.Close()

	scanner, err := scannerFor(path, f)
	if err != nil {
		return nil, err
	}
	defer scanner.Close()

	return collect(scanner)
}

type scanner interface {
	Scan() bool
	Object() osm.Object
	Err() error
	Close() error
}

func scannerFor(path string, r io.Reader) (scanner, error) {
	switch {
	case strings.HasSuffix(path, ".osm.pbf"), strings.HasSuffix(path, ".pbf"):
		return osmpbf.New(context.Background(), r, 4), nil
	case strings.HasSuffix(path, ".osm"), strings.HasSuffix(path, ".xml"):
		return osmxml.New(context.Background(), r), nil
	default:
		return nil, fmt.Errorf("unrecognized extension on %s (want .osm, .osm.pbf)", path)
	}
}

func collect(s scanner) (*Features, error) {
	feat := &Features{Nodes: make(map[osm.NodeID]*osm.Node)}
	var allNodes []*osm.Node

	for s.Scan() {
		switch o := s.Object().(type) {
		case *osm.Node:
			allNodes = append(allNodes, o)
		case *osm.Way:
			if isHighway(o) {
				feat.Ways = append(feat.Ways, o)
			}
		}
	}
	if err := s.Err(); err != nil && !errors.Is(err, io.EOF) {
		return nil, fmt.Errorf("scan: %w", err)
	}

	// Keep only nodes that are referenced by a kept way OR are signals.
	want := make(map[osm.NodeID]bool)
	for _, w := range feat.Ways {
		for _, n := range w.Nodes {
			want[n.ID] = true
		}
	}
	for _, n := range allNodes {
		if want[n.ID] || hasTag(n.Tags, "highway", "traffic_signals") {
			feat.Nodes[n.ID] = n
		}
	}
	return feat, nil
}

func isHighway(w *osm.Way) bool {
	for _, t := range w.Tags {
		if t.Key == "highway" && drivableHighway(t.Value) {
			return true
		}
	}
	return false
}

// drivableHighway returns true for highway= values that carry motor traffic.
// Excludes footways, cycleways, paths, steps, etc.
func drivableHighway(v string) bool {
	switch v {
	case "motorway", "trunk", "primary", "secondary", "tertiary",
		"unclassified", "residential", "service", "living_street",
		"motorway_link", "trunk_link", "primary_link",
		"secondary_link", "tertiary_link":
		return true
	}
	return false
}

func hasTag(tags osm.Tags, key, value string) bool {
	for _, t := range tags {
		if t.Key == key && t.Value == value {
			return true
		}
	}
	return false
}
```

- [ ] **Step 2: Verify build**

Run: `go build ./internal/osmload/`
Expected: no output, exit 0.

- [ ] **Step 3: Commit**

```bash
git add internal/osmload/
git commit -m "feat(osmload): parse .osm and .osm.pbf via paulmach/osm"
```

---

### Task 2.2: OSM loader test fixture + tests

**Files:**
- Create: `internal/osmload/testdata/tiny.osm`
- Create: `internal/osmload/osmload_test.go`

- [ ] **Step 1: Write a tiny XML fixture**

Write `internal/osmload/testdata/tiny.osm`:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6">
  <node id="1" lat="40.0000" lon="-74.0000"/>
  <node id="2" lat="40.0010" lon="-74.0000"/>
  <node id="3" lat="40.0010" lon="-74.0010"/>
  <node id="4" lat="40.0000" lon="-74.0010">
    <tag k="highway" v="traffic_signals"/>
  </node>
  <node id="99" lat="40.0500" lon="-74.0500"/>
  <way id="100">
    <nd ref="1"/>
    <nd ref="2"/>
    <nd ref="3"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="200">
    <nd ref="99"/>
    <tag k="highway" v="footway"/>
  </way>
</osm>
```

(Way 100 is a drivable residential way; way 200 is a footway that must be filtered out. Node 4 has no way reference but is a signal so it should be retained. Node 99 is only on the footway, so should be dropped along with way 200.)

- [ ] **Step 2: Write the failing test**

Write `internal/osmload/osmload_test.go`:
```go
package osmload

import (
	"testing"

	"github.com/paulmach/osm"
)

func TestLoad_XML_FiltersAndKeepsSignalNodes(t *testing.T) {
	f, err := Load("testdata/tiny.osm")
	if err != nil {
		t.Fatalf("Load: %v", err)
	}

	if len(f.Ways) != 1 {
		t.Errorf("want 1 way (residential only), got %d", len(f.Ways))
	}
	if f.Ways[0].ID != osm.WayID(100) {
		t.Errorf("want way 100, got %d", f.Ways[0].ID)
	}

	for _, id := range []osm.NodeID{1, 2, 3, 4} {
		if _, ok := f.Nodes[id]; !ok {
			t.Errorf("want node %d kept (referenced or signal), missing", id)
		}
	}
	if _, ok := f.Nodes[99]; ok {
		t.Errorf("node 99 should be dropped (only on filtered footway)")
	}
}

func TestLoad_UnknownExtension(t *testing.T) {
	_, err := Load("testdata/notarealfile.dat")
	if err == nil {
		t.Fatalf("want error for unknown extension")
	}
}
```

- [ ] **Step 3: Run test**

Run: `go test ./internal/osmload/ -v`
Expected: PASS on both.

- [ ] **Step 4: Commit**

```bash
git add internal/osmload/
git commit -m "test(osmload): cover XML filtering and signal-node retention"
```

---

### Task 2.3: Highway defaults table

**Files:**
- Create: `internal/netbuild/defaults.go`

- [ ] **Step 1: Write the defaults**

Write `internal/netbuild/defaults.go`:
```go
package netbuild

// Defaults for OSM highway types when explicit tags are missing.
// Speeds are in m/s. Lanes are *per direction* (so a default of 1 on a
// two-way street produces two 1-lane edges).

type highwayDefaults struct {
	SpeedLimit   float64 // m/s
	LanesPerDir  uint8
}

var defaults = map[string]highwayDefaults{
	"motorway":       {31.3, 2}, // ~70 mph
	"trunk":          {26.8, 2}, // ~60 mph
	"primary":        {22.4, 1}, // ~50 mph
	"secondary":      {17.9, 1}, // ~40 mph
	"tertiary":       {13.4, 1}, // ~30 mph
	"unclassified":   {11.2, 1},
	"residential":    {11.2, 1}, // ~25 mph
	"service":        {6.7, 1},
	"living_street":  {4.5, 1},
	"motorway_link":  {17.9, 1},
	"trunk_link":     {15.6, 1},
	"primary_link":   {13.4, 1},
	"secondary_link": {11.2, 1},
	"tertiary_link":  {11.2, 1},
}

func defaultsFor(highway string) highwayDefaults {
	if d, ok := defaults[highway]; ok {
		return d
	}
	return highwayDefaults{SpeedLimit: 11.2, LanesPerDir: 1}
}
```

- [ ] **Step 2: Verify build**

Run: `go build ./internal/netbuild/`
Expected: exit 0.

- [ ] **Step 3: Commit**

```bash
git add internal/netbuild/defaults.go
git commit -m "feat(netbuild): add highway type defaults table"
```

---

### Task 2.4: Build graph from features — basic skeleton

**Files:**
- Create: `internal/netbuild/netbuild.go`

- [ ] **Step 1: Write the build function**

Write `internal/netbuild/netbuild.go`:
```go
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
```

- [ ] **Step 2: Write supporting helpers in same package**

Append to `internal/netbuild/netbuild.go`:
```go
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
```

- [ ] **Step 3: Add intersection builder**

Append to `internal/netbuild/netbuild.go`:
```go
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
```

- [ ] **Step 4: Add largest-component pruning**

Append to `internal/netbuild/netbuild.go`:
```go
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
```

- [ ] **Step 5: Verify build**

Run: `go build ./...`
Expected: exit 0. Address any compile errors that surface (e.g., unused imports — `slog` is used; `strconv` is used).

- [ ] **Step 6: Commit**

```bash
git add internal/netbuild/netbuild.go
git commit -m "feat(netbuild): build routable graph from OSM features"
```

---

### Task 2.5: netbuild tests

**Files:**
- Create: `internal/netbuild/netbuild_test.go`

- [ ] **Step 1: Write the test**

Write `internal/netbuild/netbuild_test.go`:
```go
package netbuild

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/paulmach/osm"
)

func mkNode(id int64, lat, lon float64, tags ...string) *osm.Node {
	n := &osm.Node{ID: osm.NodeID(id), Lat: lat, Lon: lon}
	for i := 0; i+1 < len(tags); i += 2 {
		n.Tags = append(n.Tags, osm.Tag{Key: tags[i], Value: tags[i+1]})
	}
	return n
}

func mkWay(id int64, highway string, oneway bool, nodes ...int64) *osm.Way {
	w := &osm.Way{ID: osm.WayID(id)}
	w.Tags = append(w.Tags, osm.Tag{Key: "highway", Value: highway})
	if oneway {
		w.Tags = append(w.Tags, osm.Tag{Key: "oneway", Value: "yes"})
	}
	for _, n := range nodes {
		w.Nodes = append(w.Nodes, osm.WayNode{ID: osm.NodeID(n)})
	}
	return w
}

// Builds a + shape:
//   2
//   |
// 1-X-3      X is the intersection at node 5
//   |
//   4
func TestBuild_PlusShape_SplitsAtIntersection(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0, 0.0),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0, -74.0005),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "residential", false, 1, 5, 3),
		mkWay(20, "residential", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	// 5 nodes kept, 8 edges (4 segments * 2 directions), 1 intersection at node 5.
	if len(net.Nodes) != 5 {
		t.Errorf("want 5 nodes, got %d", len(net.Nodes))
	}
	if len(net.Edges) != 8 {
		t.Errorf("want 8 edges (4 segs * 2 dirs), got %d", len(net.Edges))
	}
	if len(net.Intersections) != 1 {
		t.Errorf("want 1 intersection, got %d", len(net.Intersections))
	}
}

func TestBuild_DropsDisconnectedComponent(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0),
		2: mkNode(2, 40.0001, -74.0),
		3: mkNode(3, 40.0002, -74.0),
		// disconnected:
		10: mkNode(10, 40.5, -74.0),
		11: mkNode(11, 40.5001, -74.0),
	}}
	feat.Ways = []*osm.Way{
		mkWay(100, "residential", false, 1, 2, 3),
		mkWay(200, "residential", false, 10, 11),
	}
	net, rpt, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if rpt.ComponentsDropped != 1 {
		t.Errorf("want 1 dropped component, got %d", rpt.ComponentsDropped)
	}
	if len(net.Nodes) != 3 {
		t.Errorf("want 3 nodes after pruning, got %d", len(net.Nodes))
	}
}

func TestBuild_OnewayProducesSingleEdge(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0),
		2: mkNode(2, 40.0001, -74.0),
	}}
	feat.Ways = []*osm.Way{mkWay(100, "primary", true, 1, 2)}
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Edges) != 1 {
		t.Errorf("oneway should produce 1 edge, got %d", len(net.Edges))
	}
}
```

- [ ] **Step 2: Run tests**

Run: `go test ./internal/netbuild/ -v`
Expected: all three PASS.

- [ ] **Step 3: Commit**

```bash
git add internal/netbuild/netbuild_test.go
git commit -m "test(netbuild): cover split-at-intersection, oneway, and component pruning"
```

---

### Task 2.6: `trafficsim load` subcommand

**Files:**
- Create: `cmd/trafficsim/main.go`

- [ ] **Step 1: Write the CLI entry point**

Write `cmd/trafficsim/main.go`:
```go
// Command trafficsim is the live simulator + viewer binary.
//
// For now (Phase 2 milestone) it only supports the `load` subcommand,
// which parses an OSM file, builds the graph, and prints stats.
package main

import (
	"flag"
	"fmt"
	"log/slog"
	"os"

	"github.com/lab1702/traffic-sim/internal/netbuild"
	"github.com/lab1702/traffic-sim/internal/osmload"
)

func main() {
	if len(os.Args) < 2 {
		usage()
		os.Exit(2)
	}
	switch os.Args[1] {
	case "load":
		runLoad(os.Args[2:])
	default:
		usage()
		os.Exit(2)
	}
}

func usage() {
	fmt.Fprintln(os.Stderr, "usage: trafficsim <subcommand> [flags]")
	fmt.Fprintln(os.Stderr, "subcommands:")
	fmt.Fprintln(os.Stderr, "  load <path-to-osm>   parse and print graph stats")
}

func runLoad(args []string) {
	fs := flag.NewFlagSet("load", flag.ExitOnError)
	if err := fs.Parse(args); err != nil {
		os.Exit(2)
	}
	if fs.NArg() != 1 {
		fmt.Fprintln(os.Stderr, "load: need exactly one OSM path")
		os.Exit(2)
	}
	path := fs.Arg(0)

	feat, err := osmload.Load(path)
	if err != nil {
		slog.Error("load failed", "err", err)
		os.Exit(1)
	}
	net, rpt, err := netbuild.Build(feat)
	if err != nil {
		slog.Error("build failed", "err", err)
		os.Exit(1)
	}
	fmt.Printf("nodes=%d edges=%d intersections=%d signals=%d\n",
		len(net.Nodes), len(net.Edges), len(net.Intersections), countSignals(net.Intersections))
	fmt.Printf("ways_skipped=%d components_dropped=%d\n",
		rpt.WaysSkipped, rpt.ComponentsDropped)
	fmt.Printf("bounds=(%.1f,%.1f)-(%.1f,%.1f) m\n",
		net.Bounds.MinX, net.Bounds.MinY, net.Bounds.MaxX, net.Bounds.MaxY)
}

func countSignals(xs []network.Intersection) int {
	n := 0
	for _, x := range xs {
		if x.HasSignal {
			n++
		}
	}
	return n
}
```

- [ ] **Step 2: Fix the import**

The function `countSignals` references `network.Intersection`. Add the import. Replace the import block in `cmd/trafficsim/main.go` with:
```go
import (
	"flag"
	"fmt"
	"log/slog"
	"os"

	"github.com/lab1702/traffic-sim/internal/netbuild"
	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
)
```

- [ ] **Step 3: Build the binary**

Run: `go build ./cmd/trafficsim/`
Expected: produces `trafficsim` (or `trafficsim.exe` on Windows), exit 0.

- [ ] **Step 4: Smoke-test against the tiny fixture**

Run (Windows PowerShell):
```powershell
.\trafficsim.exe load .\internal\osmload\testdata\tiny.osm
```
Expected output (numbers may vary slightly):
```
nodes=3 edges=2 intersections=0 signals=0
ways_skipped=0 components_dropped=0
bounds=(...) m
```
(Tiny fixture has 1 residential way of 3 nodes → 1 segment, 2 edges. No intersections because node 5 isn't shared. The signal-tagged node 4 is dropped during largest-component pruning because it's not connected.)

- [ ] **Step 5: Commit**

```bash
git add cmd/trafficsim/
git commit -m "feat(cli): add trafficsim load subcommand for graph stats"
```

---

**Phase 2 done when:**
- `go test ./...` is green.
- `trafficsim load <file>` prints stats for both `.osm` and `.osm.pbf` files (manual smoke test with a real download is recommended but not required for green CI).
