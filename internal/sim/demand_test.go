package sim

import (
	"reflect"
	"testing"
)

func TestRandomOD_DeterministicForSeed(t *testing.T) {
	net := buildLineGraph()
	s1 := NewRandomOD(net, 42, 5.0)
	s2 := NewRandomOD(net, 42, 5.0)
	var r1, r2 []SpawnRequest
	for i := 0; i < 20; i++ {
		r1 = append(r1, s1.Tick(float64(i)*0.05, 0.05)...)
		r2 = append(r2, s2.Tick(float64(i)*0.05, 0.05)...)
	}
	if !reflect.DeepEqual(r1, r2) {
		t.Fatalf("same seed should produce identical request streams\n r1=%v\n r2=%v", r1, r2)
	}
	if len(r1) == 0 {
		t.Errorf("expected at least one spawn request over 1s @ 5/s")
	}
}
