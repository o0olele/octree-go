package octree

import (
	"container/heap"
	"math"
)

// 优化的节点结构，使用整数坐标
type OptimizedJPSNode struct {
	X, Y, Z   int
	Parent    *OptimizedJPSNode
	GCost     float64
	HCost     float64
	FCost     float64
	HeapIndex int // 用于堆操作
}

func (n *OptimizedJPSNode) CalculateFCost() {
	n.FCost = n.GCost + n.HCost
}

func (n *OptimizedJPSNode) ToVector3(stepSize float64, origin Vector3) Vector3 {
	return Vector3{
		X: origin.X + float64(n.X)*stepSize,
		Y: origin.Y + float64(n.Y)*stepSize,
		Z: origin.Z + float64(n.Z)*stepSize,
	}
}

// 优先队列实现
type NodeHeap []*OptimizedJPSNode

func (h NodeHeap) Len() int           { return len(h) }
func (h NodeHeap) Less(i, j int) bool { return h[i].FCost < h[j].FCost }
func (h NodeHeap) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].HeapIndex = i
	h[j].HeapIndex = j
}

func (h *NodeHeap) Push(x interface{}) {
	node := x.(*OptimizedJPSNode)
	node.HeapIndex = len(*h)
	*h = append(*h, node)
}

func (h *NodeHeap) Pop() interface{} {
	old := *h
	n := len(old)
	node := old[n-1]
	node.HeapIndex = -1
	*h = old[0 : n-1]
	return node
}

// 方向常量
var directions = []struct{ dx, dy, dz int }{
	// 6个主要方向（减少搜索空间）
	{1, 0, 0}, {-1, 0, 0}, // X轴
	{0, 1, 0}, {0, -1, 0}, // Y轴
	{0, 0, 1}, {0, 0, -1}, // Z轴
	// 12个对角线方向
	{1, 1, 0}, {1, -1, 0}, {-1, 1, 0}, {-1, -1, 0},
	{1, 0, 1}, {1, 0, -1}, {-1, 0, 1}, {-1, 0, -1},
	{0, 1, 1}, {0, 1, -1}, {0, -1, 1}, {0, -1, -1},
}

// 优化的JPS寻路器
type OptimizedJPSPathfinder struct {
	octree      *Octree
	stepSize    float64
	origin      Vector3 // 坐标系原点
	openHeap    *NodeHeap
	closedSet   map[int64]bool // 使用int64作为键
	nodeCache   map[int64]*OptimizedJPSNode
	maxJumpDist int // 最大跳跃距离
}

func NewOptimizedJPSPathfinder(octree *Octree, stepSize float64) *OptimizedJPSPathfinder {
	heapNodes := &NodeHeap{}
	heap.Init(heapNodes)

	return &OptimizedJPSPathfinder{
		octree:      octree,
		stepSize:    stepSize,
		origin:      octree.Root.Bounds.Min, // 使用八叉树边界作为原点
		openHeap:    heapNodes,
		closedSet:   make(map[int64]bool),
		nodeCache:   make(map[int64]*OptimizedJPSNode),
		maxJumpDist: 5, // 减少最大跳跃距离，避免过度搜索
	}
}

func (jps *OptimizedJPSPathfinder) SetStepSize(stepSize float64) {
	jps.stepSize = stepSize
}

// 将Vector3转换为整数坐标
func (jps *OptimizedJPSPathfinder) toGridCoord(pos Vector3) (int, int, int) {
	return int(math.Round((pos.X - jps.origin.X) / jps.stepSize)),
		int(math.Round((pos.Y - jps.origin.Y) / jps.stepSize)),
		int(math.Round((pos.Z - jps.origin.Z) / jps.stepSize))
}

// 整数坐标哈希
func (jps *OptimizedJPSPathfinder) hash(x, y, z int) int64 {
	// 使用位移和异或来创建唯一哈希
	return int64(x)<<32 | int64(y)<<16 | int64(z)
}

// 检查位置是否被占用（添加边界检查）
func (jps *OptimizedJPSPathfinder) isOccupied(x, y, z int) bool {
	pos := Vector3{
		X: jps.origin.X + float64(x)*jps.stepSize,
		Y: jps.origin.Y + float64(y)*jps.stepSize,
		Z: jps.origin.Z + float64(z)*jps.stepSize,
	}

	// 边界检查
	bounds := jps.octree.Root.Bounds
	if pos.X < bounds.Min.X || pos.X > bounds.Max.X ||
		pos.Y < bounds.Min.Y || pos.Y > bounds.Max.Y ||
		pos.Z < bounds.Min.Z || pos.Z > bounds.Max.Z {
		return true // 超出边界视为占用
	}

	return jps.octree.IsOccupied(pos)
}

// 完全非递归的跳点搜索
func (jps *OptimizedJPSPathfinder) jumpPoint(startX, startY, startZ, dx, dy, dz, endX, endY, endZ int) *OptimizedJPSNode {
	x, y, z := startX, startY, startZ

	for i := 0; i < jps.maxJumpDist; i++ {
		x, y, z = x+dx, y+dy, z+dz

		// 检查边界和障碍物
		if jps.isOccupied(x, y, z) {
			return nil
		}

		// 到达目标
		if x == endX && y == endY && z == endZ {
			return &OptimizedJPSNode{X: x, Y: y, Z: z}
		}

		// 检查是否是跳点（强制邻居检测）
		if jps.hasForced邻居(x, y, z, dx, dy, dz) {
			return &OptimizedJPSNode{X: x, Y: y, Z: z}
		}

		// 对角线移动时，检查主轴方向是否有跳点
		// 简化版本：如果是对角线移动且移动了足够距离，就作为跳点
		if (dx != 0 && dy != 0) || (dx != 0 && dz != 0) || (dy != 0 && dz != 0) {
			if i > 0 && i%3 == 0 { // 每3步检查一次
				return &OptimizedJPSNode{X: x, Y: y, Z: z}
			}
		}
	}

	// 达到最大跳跃距离，返回当前位置作为跳点
	return &OptimizedJPSNode{X: x, Y: y, Z: z}
}

// 简化的强制邻居检测
func (jps *OptimizedJPSPathfinder) hasForced邻居(x, y, z, dx, dy, dz int) bool {
	// 检查主轴方向的强制邻居
	if dx != 0 {
		// X轴移动，检查Y和Z方向的邻居
		return (jps.isOccupied(x-dx, y-1, z) && !jps.isOccupied(x, y-1, z)) ||
			(jps.isOccupied(x-dx, y+1, z) && !jps.isOccupied(x, y+1, z)) ||
			(jps.isOccupied(x-dx, y, z-1) && !jps.isOccupied(x, y, z-1)) ||
			(jps.isOccupied(x-dx, y, z+1) && !jps.isOccupied(x, y, z+1))
	}
	if dy != 0 {
		// Y轴移动，检查X和Z方向的邻居
		return (jps.isOccupied(x-1, y-dy, z) && !jps.isOccupied(x-1, y, z)) ||
			(jps.isOccupied(x+1, y-dy, z) && !jps.isOccupied(x+1, y, z)) ||
			(jps.isOccupied(x, y-dy, z-1) && !jps.isOccupied(x, y, z-1)) ||
			(jps.isOccupied(x, y-dy, z+1) && !jps.isOccupied(x, y, z+1))
	}
	if dz != 0 {
		// Z轴移动，检查X和Y方向的邻居
		return (jps.isOccupied(x-1, y, z-dz) && !jps.isOccupied(x-1, y, z)) ||
			(jps.isOccupied(x+1, y, z-dz) && !jps.isOccupied(x+1, y, z)) ||
			(jps.isOccupied(x, y-1, z-dz) && !jps.isOccupied(x, y-1, z)) ||
			(jps.isOccupied(x, y+1, z-dz) && !jps.isOccupied(x, y+1, z))
	}
	return false
}

// 计算距离（曼哈顿距离，更快）
func (jps *OptimizedJPSPathfinder) heuristic(x1, y1, z1, x2, y2, z2 int) float64 {
	dx := math.Abs(float64(x1 - x2))
	dy := math.Abs(float64(y1 - y2))
	dz := math.Abs(float64(z1 - z2))
	return (dx + dy + dz) * jps.stepSize
}

// 优化的寻路主函数
func (jps *OptimizedJPSPathfinder) FindPath(start, end Vector3) []Vector3 {
	// 重置状态
	jps.openHeap = &NodeHeap{}
	heap.Init(jps.openHeap)
	jps.closedSet = make(map[int64]bool)
	jps.nodeCache = make(map[int64]*OptimizedJPSNode)

	startX, startY, startZ := jps.toGridCoord(start)
	endX, endY, endZ := jps.toGridCoord(end)

	// 检查起点和终点是否有效
	if jps.isOccupied(startX, startY, startZ) || jps.isOccupied(endX, endY, endZ) {
		return nil
	}

	startNode := &OptimizedJPSNode{
		X:      startX,
		Y:      startY,
		Z:      startZ,
		Parent: nil,
		GCost:  0,
		HCost:  jps.heuristic(startX, startY, startZ, endX, endY, endZ),
	}
	startNode.CalculateFCost()

	heap.Push(jps.openHeap, startNode)
	startHash := jps.hash(startX, startY, startZ)
	jps.nodeCache[startHash] = startNode

	maxIterations := 1000 // 进一步减少最大迭代次数
	iterations := 0

	for jps.openHeap.Len() > 0 && iterations < maxIterations {
		iterations++

		current := heap.Pop(jps.openHeap).(*OptimizedJPSNode)
		currentHash := jps.hash(current.X, current.Y, current.Z)
		jps.closedSet[currentHash] = true

		// 到达目标
		if current.X == endX && current.Y == endY && current.Z == endZ {
			return jps.reconstructPath(current)
		}

		// 搜索所有方向的跳点
		for _, dir := range directions {
			jumpNode := jps.jumpPoint(current.X, current.Y, current.Z,
				dir.dx, dir.dy, dir.dz, endX, endY, endZ)

			if jumpNode == nil {
				continue
			}

			jumpHash := jps.hash(jumpNode.X, jumpNode.Y, jumpNode.Z)

			// 跳过已关闭的节点
			if jps.closedSet[jumpHash] {
				continue
			}

			// 计算代价
			dx := float64(jumpNode.X - current.X)
			dy := float64(jumpNode.Y - current.Y)
			dz := float64(jumpNode.Z - current.Z)
			gCost := current.GCost + math.Sqrt(dx*dx+dy*dy+dz*dz)*jps.stepSize

			// 检查是否已存在更好的路径
			if existingNode, exists := jps.nodeCache[jumpHash]; exists {
				if gCost >= existingNode.GCost {
					continue
				}
				// 更新现有节点
				existingNode.Parent = current
				existingNode.GCost = gCost
				existingNode.CalculateFCost()
				heap.Fix(jps.openHeap, existingNode.HeapIndex)
			} else {
				// 创建新节点
				jumpNode.Parent = current
				jumpNode.GCost = gCost
				jumpNode.HCost = jps.heuristic(jumpNode.X, jumpNode.Y, jumpNode.Z, endX, endY, endZ)
				jumpNode.CalculateFCost()

				heap.Push(jps.openHeap, jumpNode)
				jps.nodeCache[jumpHash] = jumpNode
			}
		}
	}

	return nil // 未找到路径
}

// 重构路径
func (jps *OptimizedJPSPathfinder) reconstructPath(node *OptimizedJPSNode) []Vector3 {
	path := make([]Vector3, 0)
	current := node

	for current != nil {
		pos := current.ToVector3(jps.stepSize, jps.origin)
		path = append([]Vector3{pos}, path...)
		current = current.Parent
	}

	return path
}

// 路径平滑优化
func (jps *OptimizedJPSPathfinder) SmoothPath(path []Vector3) []Vector3 {
	if len(path) <= 2 {
		return path
	}

	smoothed := []Vector3{path[0]}

	for i := 1; i < len(path)-1; i++ {
		// 检查是否可以直线连接
		if !jps.hasObstacleBetween(smoothed[len(smoothed)-1], path[i+1]) {
			continue // 跳过中间点
		}
		smoothed = append(smoothed, path[i])
	}

	smoothed = append(smoothed, path[len(path)-1])
	return smoothed
}

// 检查两点间是否有障碍物
func (jps *OptimizedJPSPathfinder) hasObstacleBetween(start, end Vector3) bool {
	steps := int(start.Distance(end) / jps.stepSize)
	if steps <= 1 {
		return false
	}

	for i := 1; i < steps; i++ {
		t := float64(i) / float64(steps)
		pos := Vector3{
			X: start.X + (end.X-start.X)*t,
			Y: start.Y + (end.Y-start.Y)*t,
			Z: start.Z + (end.Z-start.Z)*t,
		}
		if jps.octree.IsOccupied(pos) {
			return true
		}
	}
	return false
}
