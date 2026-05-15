package sim

import (
	"container/heap"
	"errors"
	"math"

	"github.com/lab1702/traffic-sim/internal/network"
)

// noEdge is the sentinel "arrived from nowhere" value used at the source
// of a search. Vehicles at their origin can take any outgoing edge.
const noEdge = network.EdgeID(math.MaxUint32)

// Router computes shortest-path routes over a Network. Cost = edge length
// / speed limit (free-flow travel time). When a node has TurnRestriction
// entries on its Intersection, the router uses edge-expanded A* — state
// is (node, arriving-edge) — so a transition that violates a restriction
// is never relaxed.
type Router struct {
	net    *network.Network
	adjOut map[network.NodeID][]network.EdgeID
	// banned[node][from] is the set of forbidden outgoing edges given an
	// incoming edge. Built once at NewRouter; nil entries are common.
	banned map[network.NodeID]map[network.EdgeID]map[network.EdgeID]bool
}

func NewRouter(net *network.Network) *Router {
	adj := make(map[network.NodeID][]network.EdgeID, len(net.Nodes))
	for i := range net.Edges {
		e := &net.Edges[i]
		adj[e.From] = append(adj[e.From], e.ID)
	}
	banned := make(map[network.NodeID]map[network.EdgeID]map[network.EdgeID]bool)
	for i := range net.Intersections {
		x := &net.Intersections[i]
		if len(x.BannedTurns) == 0 {
			continue
		}
		nodeMap := make(map[network.EdgeID]map[network.EdgeID]bool)
		for _, tr := range x.BannedTurns {
			set := nodeMap[tr.From]
			if set == nil {
				set = make(map[network.EdgeID]bool)
				nodeMap[tr.From] = set
			}
			set[tr.To] = true
		}
		banned[x.NodeID] = nodeMap
	}
	return &Router{net: net, adjOut: adj, banned: banned}
}

var ErrNoRoute = errors.New("no route between nodes")

// searchState pairs the current node with the edge we arrived on, so the
// search can consult turn restrictions at the node's intersection.
type searchState struct {
	Node       network.NodeID
	ArrivedVia network.EdgeID
}

// Route returns the edge IDs to traverse from src to dst, respecting any
// turn restrictions on the intermediate intersections.
func (r *Router) Route(src, dst network.NodeID) ([]network.EdgeID, error) {
	if src == dst {
		return nil, nil
	}

	dstPos := r.net.Nodes[dst].Pos
	heuristic := func(n network.NodeID) float64 {
		p := r.net.Nodes[n].Pos
		dx, dy := p.X-dstPos.X, p.Y-dstPos.Y
		return math.Sqrt(dx*dx+dy*dy) / 31.3 // admissible against max speed
	}

	gScore := make(map[searchState]float64)
	cameFrom := make(map[searchState]searchState)
	closed := make(map[searchState]bool)

	start := searchState{Node: src, ArrivedVia: noEdge}
	gScore[start] = 0

	open := &pq{}
	heap.Init(open)
	heap.Push(open, &pqItem{state: start, f: heuristic(src)})

	for open.Len() > 0 {
		cur := heap.Pop(open).(*pqItem)
		if cur.state.Node == dst {
			return reconstruct(cameFrom, start, cur.state), nil
		}
		if closed[cur.state] {
			continue
		}
		closed[cur.state] = true

		// Skip outgoing edges that are banned given our arrival edge.
		nodeBans := r.banned[cur.state.Node]
		var fromBans map[network.EdgeID]bool
		if nodeBans != nil && cur.state.ArrivedVia != noEdge {
			fromBans = nodeBans[cur.state.ArrivedVia]
		}

		// Prohibit U-turns at intermediate nodes UNLESS the U-turn is
		// the only non-banned option (e.g., dead-end streets). Vehicles
		// at the origin have ArrivedVia=noEdge and can pick any outgoing
		// edge — a true U-turn requires an arrival edge to flip from.
		uTurnsAllowed := cur.state.ArrivedVia == noEdge
		if !uTurnsAllowed {
			uTurnsAllowed = true
			for _, eid := range r.adjOut[cur.state.Node] {
				if fromBans != nil && fromBans[eid] {
					continue
				}
				if network.ClassifyTurn(r.net, cur.state.ArrivedVia, eid) != network.TurnUTurn {
					uTurnsAllowed = false
					break
				}
			}
		}

		for _, eid := range r.adjOut[cur.state.Node] {
			if fromBans != nil && fromBans[eid] {
				continue
			}
			if !uTurnsAllowed &&
				network.ClassifyTurn(r.net, cur.state.ArrivedVia, eid) == network.TurnUTurn {
				continue
			}
			e := &r.net.Edges[eid]
			speed := e.SpeedLimit
			if speed < 0.1 {
				speed = 0.1
			}
			next := searchState{Node: e.To, ArrivedVia: eid}
			tentative := gScore[cur.state] + e.Length/speed
			if existing, ok := gScore[next]; ok && tentative >= existing {
				continue
			}
			gScore[next] = tentative
			cameFrom[next] = cur.state
			heap.Push(open, &pqItem{state: next, f: tentative + heuristic(next.Node)})
		}
	}
	return nil, ErrNoRoute
}

func reconstruct(cameFrom map[searchState]searchState, start, end searchState) []network.EdgeID {
	var out []network.EdgeID
	cur := end
	for cur != start {
		out = append(out, cur.ArrivedVia)
		prev, ok := cameFrom[cur]
		if !ok {
			return nil
		}
		cur = prev
	}
	// Reverse in place.
	for i, j := 0, len(out)-1; i < j; i, j = i+1, j-1 {
		out[i], out[j] = out[j], out[i]
	}
	return out
}

// --- priority queue ---

type pqItem struct {
	state searchState
	f     float64
	idx   int
}

type pq []*pqItem

func (p pq) Len() int           { return len(p) }
func (p pq) Less(i, j int) bool { return p[i].f < p[j].f }
func (p pq) Swap(i, j int)      { p[i], p[j] = p[j], p[i]; p[i].idx, p[j].idx = i, j }
func (p *pq) Push(x any)        { it := x.(*pqItem); it.idx = len(*p); *p = append(*p, it) }
func (p *pq) Pop() any          { o := *p; n := len(o); it := o[n-1]; *p = o[:n-1]; return it }
