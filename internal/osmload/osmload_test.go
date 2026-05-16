package osmload

import (
	"strings"
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

func TestLoad_RetainsTurnRestrictionRelations(t *testing.T) {
	f, err := Load("testdata/with_restriction.osm")
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if len(f.Restrictions) != 1 {
		t.Fatalf("want 1 retained restriction relation, got %d", len(f.Restrictions))
	}
	r := f.Restrictions[0]
	if r.ID != osm.RelationID(500) {
		t.Errorf("want relation ID 500, got %d", r.ID)
	}
	// Verify the tag values made it through.
	var typeTag, restrictionTag string
	for _, tg := range r.Tags {
		switch tg.Key {
		case "type":
			typeTag = tg.Value
		case "restriction":
			restrictionTag = tg.Value
		}
	}
	if typeTag != "restriction" {
		t.Errorf("relation type tag: want 'restriction', got %q", typeTag)
	}
	if restrictionTag != "no_left_turn" {
		t.Errorf("restriction tag: want 'no_left_turn', got %q", restrictionTag)
	}
	if len(r.Members) != 3 {
		t.Errorf("want 3 members, got %d", len(r.Members))
	}
}

func TestLoad_UnknownExtension(t *testing.T) {
	_, err := Load("testdata/notarealfile.dat")
	if err == nil {
		t.Fatalf("want error for unknown extension")
	}
	if !strings.Contains(err.Error(), "notarealfile.dat") &&
		!strings.Contains(err.Error(), "unrecognized extension") {
		t.Errorf("error should mention filename or be about extension, got: %v", err)
	}
}

func TestLoad_RetainsSignNodes(t *testing.T) {
	f, err := Load("testdata/sign_nodes.osm")
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	// Nodes 1-3 referenced by the kept way; nodes 10-13 retained only
	// because of sign tags; node 99 not referenced and not signed.
	for _, id := range []osm.NodeID{10, 11, 12, 13} {
		if _, ok := f.Nodes[id]; !ok {
			t.Errorf("node %d should be retained (carries sign tag), missing", id)
		}
	}
	if _, ok := f.Nodes[99]; ok {
		t.Errorf("node 99 should be dropped (unreferenced, untagged)")
	}
}
