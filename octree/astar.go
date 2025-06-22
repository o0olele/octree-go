package octree

import (
	"container/heap"
	"math"
)

// A*节点结构
type AStarNode struct {
	X, Y, Z   int
	Parent    *AStarNode
	GCost     float64 // 从起点到当前节点的实际代价
	HCost     float64 // 从当前节点到终点的启发式代价
	FCost     float64 // 总代价 F = G + H
	HeapIndex int     // 用于堆操作
}

func (n *AStarNode) CalculateFCost() {
	n.FCost = n.GCost + n.HCost
}

func (n *AStarNode) ToVector3(stepSize float64, origin Vector3) Vector3 {
	return Vector3{
		X: origin.X + float64(n.X)*stepSize,
		Y: origin.Y + float64(n.Y)*stepSize,
		Z: origin.Z + float64(n.Z)*stepSize,
	}
}

// A*优先队列实现
type AStarNodeHeap []*AStarNode

func (h AStarNodeHeap) Len() int           { return len(h) }
func (h AStarNodeHeap) Less(i, j int) bool { return h[i].FCost < h[j].FCost }
func (h AStarNodeHeap) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].HeapIndex = i
	h[j].HeapIndex = j
}

func (h *AStarNodeHeap) Push(x interface{}) {
	node := x.(*AStarNode)
	node.HeapIndex = len(*h)
	*h = append(*h, node)
}

func (h *AStarNodeHeap) Pop() interface{} {
	old := *h
	n := len(old)
	node := old[n-1]
	node.HeapIndex = -1
	*h = old[0 : n-1]
	return node
}

// A*寻路器
type AStarPathfinder struct {
	octree    *Octree
	agent     *Agent
	stepSize  float64
	origin    Vector3
	openHeap  *AStarNodeHeap
	closedSet map[int64]bool
	nodeCache map[int64]*AStarNode
}

// 6个基本方向 + 有限的对角线方向（更保守的移动方式）
var astarDirections = []struct{ dx, dy, dz int }{
	// 6个面方向（优先级最高）
	{1, 0, 0}, {-1, 0, 0}, // X轴
	{0, 1, 0}, {0, -1, 0}, // Y轴
	{0, 0, 1}, {0, 0, -1}, // Z轴

	// 水平面对角线方向（Y轴保持不变）
	{1, 0, 1}, {1, 0, -1}, {-1, 0, 1}, {-1, 0, -1}, // XZ平面

	// 减少复杂的三维对角线移动以避免穿墙
}

// NewAStarPathfinder 创建新的A*寻路器
func NewAStarPathfinder(octree *Octree, stepSize float64) *AStarPathfinder {
	heapNodes := &AStarNodeHeap{}
	heap.Init(heapNodes)

	return &AStarPathfinder{
		octree:    octree,
		agent:     nil,
		stepSize:  stepSize,
		origin:    octree.Root.Bounds.Min,
		openHeap:  heapNodes,
		closedSet: make(map[int64]bool),
		nodeCache: make(map[int64]*AStarNode),
	}
}

// SetAgent 设置寻路Agent
func (astar *AStarPathfinder) SetAgent(agent *Agent) {
	astar.agent = agent
}

// GetAgent 获取当前Agent
func (astar *AStarPathfinder) GetAgent() *Agent {
	return astar.agent
}

// GetStepSize 获取步长
func (astar *AStarPathfinder) GetStepSize() float64 {
	return astar.stepSize
}

// SetStepSize 设置步长
func (astar *AStarPathfinder) SetStepSize(stepSize float64) {
	astar.stepSize = stepSize
}

// ToGridCoord 将Vector3转换为网格坐标
func (astar *AStarPathfinder) ToGridCoord(pos Vector3) (int, int, int) {
	return astar.toGridCoord(pos)
}

// 将Vector3转换为整数坐标
func (astar *AStarPathfinder) toGridCoord(pos Vector3) (int, int, int) {
	return int(math.Round((pos.X - astar.origin.X) / astar.stepSize)),
		int(math.Round((pos.Y - astar.origin.Y) / astar.stepSize)),
		int(math.Round((pos.Z - astar.origin.Z) / astar.stepSize))
}

// 整数坐标哈希
func (astar *AStarPathfinder) hash(x, y, z int) int64 {
	return int64(x)<<32 | int64(y)<<16 | int64(z)
}

// 检查位置是否被占用
func (astar *AStarPathfinder) isOccupied(x, y, z int) bool {
	pos := Vector3{
		X: astar.origin.X + float64(x)*astar.stepSize,
		Y: astar.origin.Y + float64(y)*astar.stepSize,
		Z: astar.origin.Z + float64(z)*astar.stepSize,
	}

	// 边界检查
	bounds := astar.octree.Root.Bounds
	if astar.agent != nil {
		// 如果有Agent，考虑Agent的边界
		agentBounds := astar.agent.GetBounds(pos)
		if agentBounds.Min.X < bounds.Min.X || agentBounds.Max.X > bounds.Max.X ||
			agentBounds.Min.Y < bounds.Min.Y || agentBounds.Max.Y > bounds.Max.Y ||
			agentBounds.Min.Z < bounds.Min.Z || agentBounds.Max.Z > bounds.Max.Z {
			return true // 超出边界视为占用
		}

		// 使用更严格的Agent碰撞检测
		return astar.isAgentOccupiedStrict(pos)
	} else {
		// 没有Agent时使用点检测
		if pos.X < bounds.Min.X || pos.X > bounds.Max.X ||
			pos.Y < bounds.Min.Y || pos.Y > bounds.Max.Y ||
			pos.Z < bounds.Min.Z || pos.Z > bounds.Max.Z {
			return true // 超出边界视为占用
		}
		return astar.octree.IsOccupied(pos)
	}
}

// 更严格的Agent占用检测
func (astar *AStarPathfinder) isAgentOccupiedStrict(pos Vector3) bool {
	// 基本的Agent碰撞检测
	if astar.octree.IsAgentOccupied(astar.agent, pos) {
		return true
	}

	// 额外检查Agent周围的空间
	radius := astar.agent.Radius

	// 检查Agent周围的关键点
	checkPoints := []Vector3{
		{pos.X + radius, pos.Y, pos.Z},
		{pos.X - radius, pos.Y, pos.Z},
		{pos.X, pos.Y, pos.Z + radius},
		{pos.X, pos.Y, pos.Z - radius},
		{pos.X, pos.Y + astar.agent.Height/2, pos.Z},
		{pos.X, pos.Y - astar.agent.Height/2, pos.Z},
	}

	for _, checkPos := range checkPoints {
		if astar.octree.IsOccupied(checkPos) {
			return true
		}
	}

	return false
}

// 计算两点间的启发式距离（欧几里得距离）
func (astar *AStarPathfinder) heuristic(x1, y1, z1, x2, y2, z2 int) float64 {
	dx := float64(x2 - x1)
	dy := float64(y2 - y1)
	dz := float64(z2 - z1)
	return math.Sqrt(dx*dx+dy*dy+dz*dz) * astar.stepSize
}

// 计算移动代价
func (astar *AStarPathfinder) moveCost(dx, dy, dz int) float64 {
	// 对角线移动的代价更高
	distance := math.Sqrt(float64(dx*dx + dy*dy + dz*dz))
	return distance * astar.stepSize
}

// 检查路径是否有障碍物（用于胶囊体碰撞检测）
func (astar *AStarPathfinder) hasObstacleInPath(startX, startY, startZ, endX, endY, endZ int) bool {
	dx := endX - startX
	dy := endY - startY
	dz := endZ - startZ

	// 计算实际3D距离
	distance := math.Sqrt(float64(dx*dx + dy*dy + dz*dz))
	if distance <= 1 {
		return false
	}

	// 超高密度采样检测路径 - 确保不会遗漏任何障碍物
	samples := int(math.Ceil(distance * 5)) // 每个单位距离5个采样点
	if samples < 10 {
		samples = 10
	}

	for i := 1; i < samples; i++ {
		t := float64(i) / float64(samples)
		checkX := startX + int(math.Round(float64(dx)*t))
		checkY := startY + int(math.Round(float64(dy)*t))
		checkZ := startZ + int(math.Round(float64(dz)*t))

		if astar.isOccupied(checkX, checkY, checkZ) {
			return true
		}
	}

	return false
}

// FindPath 使用A*算法寻找路径
func (astar *AStarPathfinder) FindPath(start, end Vector3) []Vector3 {
	// 清理之前的状态
	astar.openHeap = &AStarNodeHeap{}
	heap.Init(astar.openHeap)
	astar.closedSet = make(map[int64]bool)
	astar.nodeCache = make(map[int64]*AStarNode)

	// 转换为网格坐标
	startX, startY, startZ := astar.toGridCoord(start)
	endX, endY, endZ := astar.toGridCoord(end)

	// 检查起点和终点是否有效
	if astar.isOccupied(startX, startY, startZ) || astar.isOccupied(endX, endY, endZ) {
		return nil
	}

	// 创建起始节点
	startNode := &AStarNode{
		X:      startX,
		Y:      startY,
		Z:      startZ,
		Parent: nil,
		GCost:  0,
		HCost:  astar.heuristic(startX, startY, startZ, endX, endY, endZ),
	}
	startNode.CalculateFCost()

	heap.Push(astar.openHeap, startNode)
	astar.nodeCache[astar.hash(startX, startY, startZ)] = startNode

	maxIterations := 50000 // 增加最大迭代次数
	iterations := 0

	for astar.openHeap.Len() > 0 && iterations < maxIterations {
		iterations++

		// 获取F代价最小的节点
		currentNode := heap.Pop(astar.openHeap).(*AStarNode)
		currentHash := astar.hash(currentNode.X, currentNode.Y, currentNode.Z)

		// 添加到已访问集合
		astar.closedSet[currentHash] = true

		// 检查是否到达目标
		if currentNode.X == endX && currentNode.Y == endY && currentNode.Z == endZ {
			return astar.reconstructPath(currentNode)
		}

		// 遍历所有邻居
		for _, dir := range astarDirections {
			neighborX := currentNode.X + dir.dx
			neighborY := currentNode.Y + dir.dy
			neighborZ := currentNode.Z + dir.dz

			neighborHash := astar.hash(neighborX, neighborY, neighborZ)

			// 跳过已访问的节点
			if astar.closedSet[neighborHash] {
				continue
			}

			// 检查邻居是否被占用
			if astar.isOccupied(neighborX, neighborY, neighborZ) {
				continue
			}

			// 对于对角线移动，检查相邻的轴向是否也可通行（避免切角）
			if dir.dx != 0 && dir.dz != 0 {
				// 检查X轴和Z轴方向是否可通行
				if astar.isOccupied(currentNode.X+dir.dx, currentNode.Y, currentNode.Z) ||
					astar.isOccupied(currentNode.X, currentNode.Y, currentNode.Z+dir.dz) {
					continue
				}
			}

			// 检查路径是否有障碍物（对于胶囊体Agent）
			if astar.hasObstacleInPath(currentNode.X, currentNode.Y, currentNode.Z, neighborX, neighborY, neighborZ) {
				continue
			}

			// 计算新的G代价
			tentativeGCost := currentNode.GCost + astar.moveCost(dir.dx, dir.dy, dir.dz)

			// 检查是否已经在开放列表中
			existingNode, exists := astar.nodeCache[neighborHash]
			if exists {
				// 如果新路径更好，更新节点
				if tentativeGCost < existingNode.GCost {
					existingNode.Parent = currentNode
					existingNode.GCost = tentativeGCost
					existingNode.CalculateFCost()
					heap.Fix(astar.openHeap, existingNode.HeapIndex)
				}
			} else {
				// 创建新节点
				neighborNode := &AStarNode{
					X:      neighborX,
					Y:      neighborY,
					Z:      neighborZ,
					Parent: currentNode,
					GCost:  tentativeGCost,
					HCost:  astar.heuristic(neighborX, neighborY, neighborZ, endX, endY, endZ),
				}
				neighborNode.CalculateFCost()

				heap.Push(astar.openHeap, neighborNode)
				astar.nodeCache[neighborHash] = neighborNode
			}
		}
	}

	// 没有找到路径
	return nil
}

// 重构路径
func (astar *AStarPathfinder) reconstructPath(node *AStarNode) []Vector3 {
	var path []Vector3
	current := node

	for current != nil {
		pos := current.ToVector3(astar.stepSize, astar.origin)
		path = append([]Vector3{pos}, path...) // 前插入以保持正确顺序
		current = current.Parent
	}

	// 平滑路径
	return astar.SmoothPath(path)
}

// SmoothPath 路径平滑处理 - 更保守的平滑策略
func (astar *AStarPathfinder) SmoothPath(path []Vector3) []Vector3 {
	if len(path) <= 2 {
		return path
	}

	// 对于复杂场景，减少平滑处理以避免穿墙
	if astar.agent != nil {
		// 有Agent时采用更保守的平滑策略
		return astar.conservativeSmooth(path)
	}

	smoothed := []Vector3{path[0]}
	i := 0

	for i < len(path)-1 {
		// 尝试找到最远的可直达点
		farthest := i + 1
		for j := i + 2; j < len(path) && j <= i+3; j++ { // 限制跳跃距离
			if !astar.hasObstacleBetween(path[i], path[j]) {
				farthest = j
			} else {
				break
			}
		}

		smoothed = append(smoothed, path[farthest])
		i = farthest
	}

	return smoothed
}

// 保守的路径平滑策略
func (astar *AStarPathfinder) conservativeSmooth(path []Vector3) []Vector3 {
	if len(path) <= 2 {
		return path
	}

	smoothed := []Vector3{path[0]}
	i := 0

	for i < len(path)-1 {
		// 只尝试跳过一个中间点
		if i+2 < len(path) && !astar.hasObstacleBetween(path[i], path[i+2]) {
			smoothed = append(smoothed, path[i+2])
			i += 2
		} else {
			smoothed = append(smoothed, path[i+1])
			i++
		}
	}

	return smoothed
}

// 检查两点间是否有障碍物 - 使用更严格的检测
func (astar *AStarPathfinder) hasObstacleBetween(start, end Vector3) bool {
	// 转换为网格坐标
	startX, startY, startZ := astar.toGridCoord(start)
	endX, endY, endZ := astar.toGridCoord(end)

	dx := endX - startX
	dy := endY - startY
	dz := endZ - startZ

	distance := math.Sqrt(float64(dx*dx + dy*dy + dz*dz))
	if distance <= 1 {
		return false
	}

	// 极高密度采样 - 确保不会穿过任何障碍物
	samples := int(math.Ceil(distance * 8)) // 每个单位距离8个采样点
	if samples < 20 {
		samples = 20
	}

	for i := 1; i < samples; i++ {
		t := float64(i) / float64(samples)
		checkX := startX + int(math.Round(float64(dx)*t))
		checkY := startY + int(math.Round(float64(dy)*t))
		checkZ := startZ + int(math.Round(float64(dz)*t))

		if astar.isOccupied(checkX, checkY, checkZ) {
			return true
		}
	}

	return false
}
