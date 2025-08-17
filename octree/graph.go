package octree

import (
	"github.com/o0olele/octree-go/geometry"
	"github.com/o0olele/octree-go/math32"
)

// PathNode is the path node, corresponding to the empty leaf node in the octree
type PathNode struct {
	ID         int32
	Center     math32.Vector3
	Bounds     geometry.AABB
	Edges      []*PathEdge
	OctreeNode *OctreeNode // The corresponding octree node

	// A* algorithm related fields
	GCost     float32
	HCost     float32
	FCost     float32
	Parent    *PathNode
	HeapIndex int
}

func (n *PathNode) CalculateFCost() {
	n.FCost = n.GCost + n.HCost
}

// PathEdge is the path edge, connecting adjacent empty nodes
type PathEdge struct {
	NodeA *PathNode
	NodeB *PathNode
	Cost  float32
}

// PathGraph is the path graph
type PathGraph struct {
	Nodes  map[*OctreeNode]*PathNode
	Edges  []*PathEdge
	nextID int32
}

// NewPathGraph creates a new path graph
func NewPathGraph() *PathGraph {
	return &PathGraph{
		Nodes:  make(map[*OctreeNode]*PathNode),
		Edges:  make([]*PathEdge, 0),
		nextID: 0,
	}
}

// AddNode adds a new node to the path graph
func (g *PathGraph) AddNode(octreeNode *OctreeNode) *PathNode {
	if node, exists := g.Nodes[octreeNode]; exists {
		return node
	}

	node := &PathNode{
		ID:         g.nextID,
		Center:     octreeNode.Bounds.Center(),
		Bounds:     octreeNode.Bounds,
		Edges:      make([]*PathEdge, 0),
		OctreeNode: octreeNode,
	}
	g.nextID++

	g.Nodes[octreeNode] = node
	return node
}

// AddEdge adds a new edge to the path graph
func (g *PathGraph) AddEdge(nodeA, nodeB *PathNode) {
	// Calculate the cost of the edge (distance)
	cost := nodeA.Center.Distance(nodeB.Center)

	edge := &PathEdge{
		NodeA: nodeA,
		NodeB: nodeB,
		Cost:  cost,
	}

	g.Edges = append(g.Edges, edge)
	nodeA.Edges = append(nodeA.Edges, edge)
	nodeB.Edges = append(nodeB.Edges, edge)
}

// PathNodeHeap is the priority queue for the A* algorithm
type PathNodeHeap []*PathNode

func (h PathNodeHeap) Len() int           { return len(h) }
func (h PathNodeHeap) Less(i, j int) bool { return h[i].FCost < h[j].FCost }
func (h PathNodeHeap) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].HeapIndex = i
	h[j].HeapIndex = j
}

// Push pushes a new node to the priority queue
func (h *PathNodeHeap) Push(x interface{}) {
	node := x.(*PathNode)
	node.HeapIndex = len(*h)
	*h = append(*h, node)
}

// Pop pops a node from the priority queue
func (h *PathNodeHeap) Pop() interface{} {
	old := *h
	n := len(old)
	node := old[n-1]
	node.HeapIndex = -1
	*h = old[0 : n-1]
	return node
}
