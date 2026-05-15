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
