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
//
//	2
//	|
//
// 1-X-3      X is the intersection at node 5
//
//	|
//	4
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
