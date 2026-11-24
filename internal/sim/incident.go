package sim

import (
	"log/slog"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/trace"
)

// Severity is the kind/intensity of a road incident on an edge. Values match
// the snapshot.Sev* constants (guarded by TestSeverityConstantsMatchSnapshot).
type Severity uint8

const (
	SeverityNone Severity = 0 // no incident / clear
	Slowdown     Severity = 1
	LaneClose    Severity = 2
	FullClose    Severity = 3
)

// IncidentEvent is a UI->sim command to set (or clear, with SeverityNone) the
// incident on an edge. Delivered over World.IncidentControl, mirroring
// ControlEvent / World.Control.
type IncidentEvent struct {
	EdgeID   network.EdgeID
	Severity Severity
}

const (
	// incidentSlowdownFactor caps desired speed on a Slowdown edge to this
	// fraction of its limit (a hazard crawl).
	incidentSlowdownFactor = 0.3

	// Routing-cost penalties used by edgeCost. Slowdown/LaneClose multiply the
	// base (congestion) cost so GPS reroutes promptly without waiting for the
	// EWMA; FullClose uses a large finite cost so the edge is avoided but still
	// selectable as a last resort (mirrors congestion.go's minEdgeSpeed floor).
	incidentSlowdownCostMul  = 1.5
	incidentLaneCloseCostMul = 3.0
	incidentFullCloseCost    = 1e9
)

// edgeCost is the routing cost for an edge: congestion travel time, adjusted
// for any incident. Used by both spawn-time routing and rerouting.
func (w *World) edgeCost(eid network.EdgeID) float64 {
	if w.edgeFullyBlocked(eid) {
		return incidentFullCloseCost
	}
	base := w.Cong.Cost(w.Net, eid)
	switch w.Incidents[eid] {
	case Slowdown:
		return base * incidentSlowdownCostMul
	case LaneClose:
		return base * incidentLaneCloseCostMul
	default:
		return base
	}
}

// edgeFullyBlocked reports whether nothing can get past edge eid: a full
// closure, or a lane closure on a road with only one lane. Closing the only
// lane closes the road, so it must block entry, cost like a full closure in
// routing, and trigger GPS diversion just as FullClose does. Before this,
// single-lane LaneClose only placed the obstacle: cars kept driving in,
// stopped at it, and were despawned as stuck.
func (w *World) edgeFullyBlocked(eid network.EdgeID) bool {
	switch w.Incidents[eid] {
	case FullClose:
		return true
	case LaneClose:
		return len(w.Net.Edges[eid].Lanes) <= 1
	}
	return false
}

// closedLaneFor returns the closed lane index and true when the edge has a
// LaneClose incident. v1 always closes the curb lane (index 0). FullClose is
// handled by lineStop (all lanes), not here.
func (w *World) closedLaneFor(eid network.EdgeID) (uint8, bool) {
	if w.Incidents[eid] != LaneClose {
		return 0, false
	}
	if len(w.Net.Edges[eid].Lanes) == 0 {
		return 0, false
	}
	return 0, true
}

// applyIncident sets or clears the incident on an edge and records it. Out-of-
// range edge ids are ignored (defensive, like applyControl).
func (w *World) applyIncident(ev IncidentEvent) {
	// Ignore out-of-range edge ids (defensive, mirrors applyControl). EdgeID is
	// uint32 so the < 0 arm never fires; the upper bound is the real guard.
	if int(ev.EdgeID) < 0 || int(ev.EdgeID) >= len(w.Net.Edges) {
		return
	}
	switch ev.Severity {
	case SeverityNone, Slowdown, LaneClose, FullClose:
		// valid
	default:
		slog.Warn("applyIncident: unknown Severity; ignoring",
			"edge_id", uint32(ev.EdgeID), "severity", uint8(ev.Severity))
		return
	}
	// An incident is a property of the physical road: apply it to the clicked
	// edge and, for a two-way road, its reverse twin, so both directions are
	// affected (a closed road is closed both ways). One-way edges have no twin.
	w.setIncidentEdge(ev.EdgeID, ev.Severity)
	if twin, ok := w.reverseEdge[ev.EdgeID]; ok {
		w.setIncidentEdge(twin, ev.Severity)
	}
}

// setIncidentEdge sets or clears the incident on a single directed edge and
// records it in the trace.
func (w *World) setIncidentEdge(eid network.EdgeID, sev Severity) {
	if sev == SeverityNone {
		delete(w.Incidents, eid)
	} else {
		w.Incidents[eid] = sev
	}
	w.EmitTrace(w.Tick, w.SimTime, &trace.IncidentSet{
		EdgeID:   uint32(eid),
		Severity: uint8(sev),
	})
}

// buildReverseEdges indexes each directed edge to its opposite-direction twin
// (same node pair, swapped From/To), when one exists. Used so an incident marks
// both directions of a two-way road. One-way edges are absent from the result.
func buildReverseEdges(net *network.Network) map[network.EdgeID]network.EdgeID {
	byEnds := make(map[[2]network.NodeID]network.EdgeID, len(net.Edges))
	for i := range net.Edges {
		e := &net.Edges[i]
		byEnds[[2]network.NodeID{e.From, e.To}] = network.EdgeID(i)
	}
	rev := make(map[network.EdgeID]network.EdgeID)
	for i := range net.Edges {
		e := &net.Edges[i]
		if twin, ok := byEnds[[2]network.NodeID{e.To, e.From}]; ok {
			rev[network.EdgeID(i)] = twin
		}
	}
	return rev
}
