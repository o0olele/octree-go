package query

import "sync"

// heapNode is the data structure for the A* algorithm
type heapNode struct {
	nodeID int32
	fScore float32
	index  int
}

// nodeHeap is the heap for the A* algorithm
type nodeHeap []*heapNode

func (h nodeHeap) Len() int           { return len(h) }
func (h nodeHeap) Less(i, j int) bool { return h[i].fScore < h[j].fScore }
func (h nodeHeap) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].index = i
	h[j].index = j
}

// Push pushes a new node to the heap
func (h *nodeHeap) Push(x interface{}) {
	n := len(*h)
	item := x.(*heapNode)
	item.index = n
	*h = append(*h, item)
}

// Pop pops a node from the heap
func (h *nodeHeap) Pop() interface{} {
	old := *h
	n := len(old)
	item := old[n-1]
	old[n-1] = nil
	item.index = -1
	*h = old[0 : n-1]
	return item
}

// Clear clears the heap
func (h *nodeHeap) Clear() {
	for _, node := range *h {
		heapNodePool.Put(node)
	}
	*h = (*h)[:0]
}

// heapNodePool is the pool for the heap
var heapNodePool = sync.Pool{
	New: func() interface{} {
		return &heapNode{
			nodeID: 0,
			fScore: 0,
			index:  -1,
		}
	},
}

// newHeapNode creates a new heap node
func newHeapNode(nodeID int32, fScore float32) *heapNode {
	node := heapNodePool.Get().(*heapNode)
	node.nodeID = nodeID
	node.fScore = fScore
	node.index = -1
	return node
}
