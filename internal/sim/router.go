package sim

import (
	"container/heap"
	"errors"
	"math"

	"github.com/lab1702/traffic-sim/internal/network"
)

// Router computes shortest-path routes over a Network.
// Cost = edge length / speed limit (i.e., free-flow travel time).
type Router struct {
	net    *network.Network
	adjOut map[network.NodeID][]network.EdgeID
}

func NewRouter(net *network.Network) *Router {
	adj := make(map[network.NodeID][]network.EdgeID, len(net.Nodes))
	for i := range net.Edges {
		e := &net.Edges[i]
		adj[e.From] = append(adj[e.From], e.ID)
	}
	return &Router{net: net, adjOut: adj}
}

var ErrNoRoute = errors.New("no route between nodes")

// Route returns the edge IDs to traverse from src to dst.
func (r *Router) Route(src, dst network.NodeID) ([]network.EdgeID, error) {
	if src == dst {
		return nil, nil
	}
	gScore := make(map[network.NodeID]float64)
	cameFromEdge := make(map[network.NodeID]network.EdgeID)
	cameFromNode := make(map[network.NodeID]network.NodeID)
	gScore[src] = 0

	dstPos := r.net.Nodes[dst].Pos
	heuristic := func(n network.NodeID) float64 {
		p := r.net.Nodes[n].Pos
		dx, dy := p.X-dstPos.X, p.Y-dstPos.Y
		return math.Sqrt(dx*dx+dy*dy) / 31.3 // assume max speed
	}

	open := &pq{}
	heap.Init(open)
	heap.Push(open, &pqItem{node: src, f: heuristic(src)})
	closed := make(map[network.NodeID]bool)

	for open.Len() > 0 {
		cur := heap.Pop(open).(*pqItem)
		if cur.node == dst {
			return reconstruct(cameFromEdge, cameFromNode, src, dst), nil
		}
		if closed[cur.node] {
			continue
		}
		closed[cur.node] = true
		for _, eid := range r.adjOut[cur.node] {
			e := &r.net.Edges[eid]
			speed := e.SpeedLimit
			if speed < 0.1 {
				speed = 0.1
			}
			tentative := gScore[cur.node] + e.Length/speed
			if existing, ok := gScore[e.To]; ok && tentative >= existing {
				continue
			}
			gScore[e.To] = tentative
			cameFromEdge[e.To] = eid
			cameFromNode[e.To] = cur.node
			heap.Push(open, &pqItem{node: e.To, f: tentative + heuristic(e.To)})
		}
	}
	return nil, ErrNoRoute
}

func reconstruct(edgeBy map[network.NodeID]network.EdgeID,
	nodeBy map[network.NodeID]network.NodeID,
	src, dst network.NodeID,
) []network.EdgeID {
	var out []network.EdgeID
	cur := dst
	for cur != src {
		eid, ok := edgeBy[cur]
		if !ok {
			return nil
		}
		out = append(out, eid)
		cur = nodeBy[cur]
	}
	// Reverse in place.
	for i, j := 0, len(out)-1; i < j; i, j = i+1, j-1 {
		out[i], out[j] = out[j], out[i]
	}
	return out
}

// --- priority queue ---

type pqItem struct {
	node network.NodeID
	f    float64
	idx  int
}

type pq []*pqItem

func (p pq) Len() int            { return len(p) }
func (p pq) Less(i, j int) bool  { return p[i].f < p[j].f }
func (p pq) Swap(i, j int)       { p[i], p[j] = p[j], p[i]; p[i].idx, p[j].idx = i, j }
func (p *pq) Push(x any)         { it := x.(*pqItem); it.idx = len(*p); *p = append(*p, it) }
func (p *pq) Pop() any           { o := *p; n := len(o); it := o[n-1]; *p = o[:n-1]; return it }
