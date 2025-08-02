package octree

import (
	"github.com/o0olele/octree-go/geometry"
	"github.com/o0olele/octree-go/math32"
)

// PathNode 寻路节点，对应八叉树中的空白叶子节点
type PathNode struct {
	ID         int32
	Center     math32.Vector3
	Bounds     geometry.AABB
	Edges      []*PathEdge
	OctreeNode *OctreeNode // 对应的八叉树节点

	// A*算法相关字段
	GCost     float32
	HCost     float32
	FCost     float32
	Parent    *PathNode
	HeapIndex int
}

func (n *PathNode) CalculateFCost() {
	n.FCost = n.GCost + n.HCost
}

// PathEdge 寻路边，连接相邻的空白节点
type PathEdge struct {
	NodeA *PathNode
	NodeB *PathNode
	Cost  float32
}

// PathGraph 寻路图
type PathGraph struct {
	Nodes  map[*OctreeNode]*PathNode
	Edges  []*PathEdge
	nextID int32
}

func NewPathGraph() *PathGraph {
	return &PathGraph{
		Nodes:  make(map[*OctreeNode]*PathNode),
		Edges:  make([]*PathEdge, 0),
		nextID: 0,
	}
}

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

func (g *PathGraph) AddEdge(nodeA, nodeB *PathNode) {
	// 计算边的代价（距离）
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

// PathNodeHeap A*算法的优先队列
type PathNodeHeap []*PathNode

func (h PathNodeHeap) Len() int           { return len(h) }
func (h PathNodeHeap) Less(i, j int) bool { return h[i].FCost < h[j].FCost }
func (h PathNodeHeap) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].HeapIndex = i
	h[j].HeapIndex = j
}

func (h *PathNodeHeap) Push(x interface{}) {
	node := x.(*PathNode)
	node.HeapIndex = len(*h)
	*h = append(*h, node)
}

func (h *PathNodeHeap) Pop() interface{} {
	old := *h
	n := len(old)
	node := old[n-1]
	node.HeapIndex = -1
	*h = old[0 : n-1]
	return node
}
