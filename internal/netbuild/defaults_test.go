package netbuild

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/paulmach/osm"
)

func TestClassOf(t *testing.T) {
	cases := []struct {
		in   string
		want network.RoadClass
	}{
		{"motorway", network.ClassMotorway},
		{"motorway_link", network.ClassMotorway},
		{"trunk", network.ClassTrunk},
		{"trunk_link", network.ClassTrunk},
		{"primary", network.ClassPrimary},
		{"secondary_link", network.ClassSecondary},
		{"tertiary", network.ClassTertiary},
		{"residential", network.ClassResidential},
		{"unclassified", network.ClassUnclassified},
		{"road", network.ClassUnclassified}, // OSM placeholder
		{"service", network.ClassService},
		{"living_street", network.ClassLivingStreet},
		{"", network.ClassUnknown},
		{"footway", network.ClassUnknown}, // not in drivable set, but classOf is total
	}
	for _, c := range cases {
		if got := classOf(c.in); got != c.want {
			t.Errorf("classOf(%q) = %d, want %d", c.in, got, c.want)
		}
	}
}

// TestBuild_PropagatesRoadClass: the OSM highway tag should set
// Edge.Class for both directions of a two-way street, and the priority
// ordering should let an arterial outrank a residential street.
func TestBuild_PropagatesRoadClass(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0),
		2: mkNode(2, 40.0001, -74.0),
	}}
	feat.Ways = []*osm.Way{mkWay(100, "primary", false, 1, 2)}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Edges) != 2 {
		t.Fatalf("want 2 edges (fwd+rev), got %d", len(net.Edges))
	}
	for i, e := range net.Edges {
		if e.Class != network.ClassPrimary {
			t.Errorf("edge %d Class = %d, want ClassPrimary (%d)", i, e.Class, network.ClassPrimary)
		}
	}
	// Priority sanity: primary outranks residential, motorway outranks primary.
	if network.ClassResidential.Priority() >= network.ClassPrimary.Priority() {
		t.Error("ClassPrimary should outrank ClassResidential in draw order")
	}
	if network.ClassPrimary.Priority() >= network.ClassMotorway.Priority() {
		t.Error("ClassMotorway should outrank ClassPrimary in draw order")
	}
}

// TestBuild_LinkCollapsesToParentClass: a motorway_link way must get
// ClassMotorway so a ramp paints in the mainline's color rather than
// dropping to a default gray.
func TestBuild_LinkCollapsesToParentClass(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0),
		2: mkNode(2, 40.0001, -74.0),
	}}
	feat.Ways = []*osm.Way{mkWay(100, "motorway_link", true, 1, 2)}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Edges) != 1 {
		t.Fatalf("oneway link should produce 1 edge, got %d", len(net.Edges))
	}
	if net.Edges[0].Class != network.ClassMotorway {
		t.Errorf("motorway_link Class = %d, want ClassMotorway (%d)",
			net.Edges[0].Class, network.ClassMotorway)
	}
}
