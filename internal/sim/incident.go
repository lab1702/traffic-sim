package sim

import (
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
	base := w.Cong.Cost(w.Net, eid)
	switch w.Incidents[eid] {
	case Slowdown:
		return base * incidentSlowdownCostMul
	case LaneClose:
		return base * incidentLaneCloseCostMul
	case FullClose:
		return incidentFullCloseCost
	default:
		return base
	}
}

// closedLaneFor returns the closed lane index and true when the edge has a
// LaneClose incident. v1 always closes the curb lane (index 0). FullClose is
// handled by incidentStopDistance (all lanes), not here.
func (w *World) closedLaneFor(eid network.EdgeID) (uint8, bool) {
	if w.Incidents[eid] != LaneClose {
		return 0, false
	}
	if len(w.Net.Edges[eid].Lanes) == 0 {
		return 0, false
	}
	return 0, true
}

// incidentStopDistance returns (distance from the vehicle's front bumper to
// the incident obstacle, true) when the vehicle is blocked by an incident on
// its current edge — a full closure (all lanes) or a lane closure of the
// vehicle's lane. The obstacle sits at the edge's downstream end. Mirrors
// stopDistanceForRed's shape so Step can fold it into the virtual-leader set.
func (w *World) incidentStopDistance(v *Vehicle) (float64, bool) {
	sev := w.Incidents[v.Edge]
	blocked := sev == FullClose
	if sev == LaneClose {
		if cl, ok := w.closedLaneFor(v.Edge); ok && v.Lane == cl {
			blocked = true
		}
	}
	if !blocked {
		return 0, false
	}
	d := w.Net.Edges[v.Edge].Length - v.S
	if d < 0 {
		d = 0
	}
	return d, true
}

// applyIncident sets or clears the incident on an edge and records it. Out-of-
// range edge ids are ignored (defensive, like applyControl).
func (w *World) applyIncident(ev IncidentEvent) {
	if int(ev.EdgeID) < 0 || int(ev.EdgeID) >= len(w.Net.Edges) {
		return
	}
	if ev.Severity == SeverityNone {
		delete(w.Incidents, ev.EdgeID)
	} else {
		w.Incidents[ev.EdgeID] = ev.Severity
	}
	w.EmitTrace(w.Tick, w.SimTime, &trace.IncidentSet{
		EdgeID:   uint32(ev.EdgeID),
		Severity: uint8(ev.Severity),
	})
}
