package query

// 堆数据结构用于A*算法
type heapNode struct {
	nodeID int32
	fScore float32
	index  int
}

type nodeHeap []*heapNode

func (h nodeHeap) Len() int           { return len(h) }
func (h nodeHeap) Less(i, j int) bool { return h[i].fScore < h[j].fScore }
func (h nodeHeap) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].index = i
	h[j].index = j
}

func (h *nodeHeap) Push(x interface{}) {
	n := len(*h)
	item := x.(*heapNode)
	item.index = n
	*h = append(*h, item)
}

func (h *nodeHeap) Pop() interface{} {
	old := *h
	n := len(old)
	item := old[n-1]
	old[n-1] = nil
	item.index = -1
	*h = old[0 : n-1]
	return item
}
