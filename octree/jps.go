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

// 方向常量 - 减少方向数量，专注于主要方向
var directions = []struct{ dx, dy, dz int }{
	// 6个主要方向
	{1, 0, 0}, {-1, 0, 0}, // X轴
	{0, 1, 0}, {0, -1, 0}, // Y轴
	{0, 0, 1}, {0, 0, -1}, // Z轴
	// 重要的对角线方向（水平面）
	{1, 0, 1}, {1, 0, -1}, {-1, 0, 1}, {-1, 0, -1},
}

// 优化的JPS寻路器
type OptimizedJPSPathfinder struct {
	octree      *Octree
	agent       *Agent
	stepSize    float64
	origin      Vector3
	openHeap    *NodeHeap
	closedSet   map[int64]bool
	nodeCache   map[int64]*OptimizedJPSNode
	maxJumpDist int     // 动态调整的最大跳跃距离
	minSize     float64 // octree的最小单元格大小
}

func NewOptimizedJPSPathfinder(octree *Octree, stepSize float64) *OptimizedJPSPathfinder {
	heapNodes := &NodeHeap{}
	heap.Init(heapNodes)

	// 为复杂场景大幅减少跳跃距离，确保不跳过狭窄通道
	maxJumpDist := int(math.Max(1, octree.MinSize/stepSize/2)) // 除以2，减少跳跃距离
	if maxJumpDist > 5 {
		maxJumpDist = 5 // 进一步限制最大值为5
	}
	if maxJumpDist < 2 {
		maxJumpDist = 2 // 确保至少有一些跳跃能力
	}

	return &OptimizedJPSPathfinder{
		octree:      octree,
		agent:       nil,
		stepSize:    stepSize,
		origin:      octree.Root.Bounds.Min,
		openHeap:    heapNodes,
		closedSet:   make(map[int64]bool),
		nodeCache:   make(map[int64]*OptimizedJPSNode),
		maxJumpDist: maxJumpDist,
		minSize:     octree.MinSize,
	}
}

// SetAgent 设置寻路Agent
func (jps *OptimizedJPSPathfinder) SetAgent(agent *Agent) {
	jps.agent = agent
}

// GetAgent 获取当前Agent
func (jps *OptimizedJPSPathfinder) GetAgent() *Agent {
	return jps.agent
}

func (jps *OptimizedJPSPathfinder) GetStepSize() float64 {
	return jps.stepSize
}

func (jps *OptimizedJPSPathfinder) SetStepSize(stepSize float64) {
	jps.stepSize = stepSize
	// 重新计算最大跳跃距离（保持与构造函数一致的逻辑）
	jps.maxJumpDist = int(math.Max(1, jps.minSize/stepSize/2))
	if jps.maxJumpDist > 5 {
		jps.maxJumpDist = 5
	}
	if jps.maxJumpDist < 2 {
		jps.maxJumpDist = 2
	}
}

// GetMaxJumpDist 获取最大跳跃距离
func (jps *OptimizedJPSPathfinder) GetMaxJumpDist() int {
	return jps.maxJumpDist
}

// ToGridCoord 公共方法：将Vector3转换为网格坐标
func (jps *OptimizedJPSPathfinder) ToGridCoord(pos Vector3) (int, int, int) {
	return jps.toGridCoord(pos)
}

// 将Vector3转换为整数坐标
func (jps *OptimizedJPSPathfinder) toGridCoord(pos Vector3) (int, int, int) {
	return int(math.Round((pos.X - jps.origin.X) / jps.stepSize)),
		int(math.Round((pos.Y - jps.origin.Y) / jps.stepSize)),
		int(math.Round((pos.Z - jps.origin.Z) / jps.stepSize))
}

// 整数坐标哈希
func (jps *OptimizedJPSPathfinder) hash(x, y, z int) int64 {
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
	if jps.agent != nil {
		// 如果有Agent，考虑Agent的边界
		agentBounds := jps.agent.GetBounds(pos)
		if agentBounds.Min.X < bounds.Min.X || agentBounds.Max.X > bounds.Max.X ||
			agentBounds.Min.Y < bounds.Min.Y || agentBounds.Max.Y > bounds.Max.Y ||
			agentBounds.Min.Z < bounds.Min.Z || agentBounds.Max.Z > bounds.Max.Z {
			return true // 超出边界视为占用
		}
		return jps.octree.IsAgentOccupied(jps.agent, pos)
	} else {
		// 没有Agent时使用点检测
		if pos.X < bounds.Min.X || pos.X > bounds.Max.X ||
			pos.Y < bounds.Min.Y || pos.Y > bounds.Max.Y ||
			pos.Z < bounds.Min.Z || pos.Z > bounds.Max.Z {
			return true // 超出边界视为占用
		}
		return jps.octree.IsOccupied(pos)
	}
}

// 超精确路径碰撞检测 - 专门针对Recast dungeon等复杂狭窄通道优化
func (jps *OptimizedJPSPathfinder) hasObstacleInPath(startX, startY, startZ, endX, endY, endZ int) bool {
	dx := endX - startX
	dy := endY - startY
	dz := endZ - startZ

	// 计算实际3D距离
	distance := math.Sqrt(float64(dx*dx + dy*dy + dz*dz))

	if distance <= 1 {
		return false
	}

	// 为狭窄通道使用极高密度采样 - 每个stepSize至少4个检测点
	samplesPerStep := 4
	totalSamples := int(math.Ceil(distance * float64(samplesPerStep)))
	if totalSamples < 20 {
		totalSamples = 20 // 最少20个检测点
	}
	if totalSamples > 200 {
		totalSamples = 200 // 限制最大采样数，避免性能问题
	}

	// 沿路径进行高密度检测
	for i := 1; i <= totalSamples; i++ {
		t := float64(i) / float64(totalSamples)

		// 使用浮点插值然后四舍五入，提高精度
		fx := float64(startX) + float64(dx)*t
		fy := float64(startY) + float64(dy)*t
		fz := float64(startZ) + float64(dz)*t

		x := int(math.Round(fx))
		y := int(math.Round(fy))
		z := int(math.Round(fz))

		// 基本碰撞检测
		if jps.isOccupied(x, y, z) {
			return true
		}

		// 如果有Agent，进行Agent体积检测
		if jps.agent != nil {
			pos := Vector3{
				X: jps.origin.X + float64(x)*jps.stepSize,
				Y: jps.origin.Y + float64(y)*jps.stepSize,
				Z: jps.origin.Z + float64(z)*jps.stepSize,
			}

			// 使用八叉树的Agent碰撞检测（更精确）
			if jps.octree.IsAgentOccupied(jps.agent, pos) {
				return true
			}
		}

		// 为了保险起见，额外检查相邻的格子点
		// 这对于处理三角形网格边界很重要
		neighbors := []struct{ dx, dy, dz int }{
			{0, 0, 0},             // 当前点
			{1, 0, 0}, {-1, 0, 0}, // X方向
			{0, 1, 0}, {0, -1, 0}, // Y方向
			{0, 0, 1}, {0, 0, -1}, // Z方向
		}

		for _, n := range neighbors {
			nx, ny, nz := x+n.dx, y+n.dy, z+n.dz
			if jps.isOccupied(nx, ny, nz) {
				// 检查这个障碍物是否真的阻挡了路径
				// 使用更精确的线段-体素相交测试
				if jps.lineIntersectsVoxel(fx, fy, fz, float64(nx), float64(ny), float64(nz)) {
					return true
				}
			}
		}
	}

	return false
}

// 检查线段是否与体素相交的辅助函数
func (jps *OptimizedJPSPathfinder) lineIntersectsVoxel(lineX, lineY, lineZ, voxelX, voxelY, voxelZ float64) bool {
	// 简化的相交测试：检查线段点是否在体素的扩展边界内
	threshold := 0.3 // 体素影响范围
	return math.Abs(lineX-voxelX) < threshold &&
		math.Abs(lineY-voxelY) < threshold &&
		math.Abs(lineZ-voxelZ) < threshold
}

// 改进的跳点搜索 - 添加路径碰撞检测和直接终点检测
func (jps *OptimizedJPSPathfinder) jumpPoint(startX, startY, startZ, dx, dy, dz, endX, endY, endZ int) *OptimizedJPSNode {
	x, y, z := startX, startY, startZ

	// 首先检查是否可以直接到达终点（不受跳跃距离限制）
	if dx != 0 || dy != 0 || dz != 0 {
		// 计算到达终点需要的步数
		var stepsToEnd int
		if dx != 0 {
			stepsToEnd = (endX - startX) / dx
		} else if dy != 0 {
			stepsToEnd = (endY - startY) / dy
		} else if dz != 0 {
			stepsToEnd = (endZ - startZ) / dz
		}

		// 检查是否能按当前方向直接到达终点
		if stepsToEnd > 0 && stepsToEnd <= jps.maxJumpDist*3 { // 允许更长的直接终点跳跃
			targetX := startX + dx*stepsToEnd
			targetY := startY + dy*stepsToEnd
			targetZ := startZ + dz*stepsToEnd

			// 检查是否正好到达终点
			if targetX == endX && targetY == endY && targetZ == endZ {
				// 检查路径上是否有障碍物
				if !jps.hasObstacleInPath(startX, startY, startZ, endX, endY, endZ) {
					return &OptimizedJPSNode{X: endX, Y: endY, Z: endZ}
				}
			}
		}
	}

	// 限制跳跃距离，避免过大的跳跃
	for i := 0; i < jps.maxJumpDist; i++ {
		nextX, nextY, nextZ := x+dx, y+dy, z+dz

		// 检查下一步是否有障碍物
		if jps.isOccupied(nextX, nextY, nextZ) {
			return nil
		}

		// 检查从起点到当前点的路径是否有障碍物
		if i > 0 && jps.hasObstacleInPath(startX, startY, startZ, nextX, nextY, nextZ) {
			return nil
		}

		x, y, z = nextX, nextY, nextZ

		// 到达目标
		if x == endX && y == endY && z == endZ {
			return &OptimizedJPSNode{X: x, Y: y, Z: z}
		}

		// 检查是否是跳点（简化的强制邻居检测）
		if jps.hasForced邻居(x, y, z, dx, dy, dz) {
			return &OptimizedJPSNode{X: x, Y: y, Z: z}
		}

		// 对于狭窄通道，每一步都可能是关键跳点
		// 增加跳点密度，特别是对角线移动时
		if (dx != 0 && dy != 0) || (dx != 0 && dz != 0) || (dy != 0 && dz != 0) {
			if i > 0 { // 每一步都创建跳点，确保不遗漏狭窄通道
				return &OptimizedJPSNode{X: x, Y: y, Z: z}
			}
		} else {
			// 即使是轴向移动，在复杂场景中也要频繁创建跳点
			if i > 0 && i%2 == 0 {
				return &OptimizedJPSNode{X: x, Y: y, Z: z}
			}
		}
	}

	// 检查最终路径
	if !jps.hasObstacleInPath(startX, startY, startZ, x, y, z) {
		return &OptimizedJPSNode{X: x, Y: y, Z: z}
	}

	return nil
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

// 计算距离（欧几里得距离，更准确）
func (jps *OptimizedJPSPathfinder) heuristic(x1, y1, z1, x2, y2, z2 int) float64 {
	dx := float64(x1 - x2)
	dy := float64(y1 - y2)
	dz := float64(z1 - z2)
	return math.Sqrt(dx*dx+dy*dy+dz*dz) * jps.stepSize
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

	// 根据场景大小动态调整最大迭代次数
	sceneSize := jps.octree.Root.Bounds.Max.Sub(jps.octree.Root.Bounds.Min)
	maxDimension := math.Max(math.Max(sceneSize.X, sceneSize.Y), sceneSize.Z)
	maxIterations := int(maxDimension / jps.stepSize)
	if maxIterations > 5000 {
		maxIterations = 5000 // 设置上限，避免无限循环
	}
	if maxIterations < 500 {
		maxIterations = 500 // 设置下限，确保足够的搜索空间
	}

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

		// 首先尝试直接连接到终点
		directDistance := math.Sqrt(float64((endX-current.X)*(endX-current.X) +
			(endY-current.Y)*(endY-current.Y) + (endZ-current.Z)*(endZ-current.Z)))

		// 如果距离足够近，尝试直接连接
		if directDistance <= float64(jps.maxJumpDist*2) {
			if !jps.hasObstacleInPath(current.X, current.Y, current.Z, endX, endY, endZ) {
				// 可以直接到达终点
				finalNode := &OptimizedJPSNode{
					X:      endX,
					Y:      endY,
					Z:      endZ,
					Parent: current,
					GCost:  current.GCost + directDistance*jps.stepSize,
					HCost:  0,
				}
				finalNode.CalculateFCost()
				return jps.reconstructPath(finalNode)
			}
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

// 重构路径并添加插值点
func (jps *OptimizedJPSPathfinder) reconstructPath(node *OptimizedJPSNode) []Vector3 {
	path := make([]Vector3, 0)
	current := node

	// 收集跳点
	jumpPoints := make([]Vector3, 0)
	for current != nil {
		pos := current.ToVector3(jps.stepSize, jps.origin)
		jumpPoints = append([]Vector3{pos}, jumpPoints...)
		current = current.Parent
	}

	// 在跳点之间插值，避免穿越障碍物
	for i := 0; i < len(jumpPoints)-1; i++ {
		start := jumpPoints[i]
		end := jumpPoints[i+1]

		// 添加起点
		path = append(path, start)

		// 计算需要的插值点数
		distance := start.Distance(end)
		steps := int(math.Ceil(distance / jps.stepSize))

		// 添加插值点
		for j := 1; j < steps; j++ {
			t := float64(j) / float64(steps)
			interpolated := Vector3{
				X: start.X + (end.X-start.X)*t,
				Y: start.Y + (end.Y-start.Y)*t,
				Z: start.Z + (end.Z-start.Z)*t,
			}
			path = append(path, interpolated)
		}
	}

	// 添加终点
	if len(jumpPoints) > 0 {
		path = append(path, jumpPoints[len(jumpPoints)-1])
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

// 改进的障碍物检测
func (jps *OptimizedJPSPathfinder) hasObstacleBetween(start, end Vector3) bool {
	distance := start.Distance(end)
	steps := int(math.Ceil(distance / jps.stepSize))

	if steps <= 1 {
		return false
	}

	// 使用更细粒度的检测
	for i := 1; i < steps; i++ {
		t := float64(i) / float64(steps)
		pos := Vector3{
			X: start.X + (end.X-start.X)*t,
			Y: start.Y + (end.Y-start.Y)*t,
			Z: start.Z + (end.Z-start.Z)*t,
		}

		// 使用Agent检测或点检测
		if jps.agent != nil {
			if jps.octree.IsAgentOccupied(jps.agent, pos) {
				return true
			}
		} else {
			if jps.octree.IsOccupied(pos) {
				return true
			}
		}
	}
	return false
}
