package octree

import (
	"container/heap"
	"fmt"
	"math"
	"sort"
	"time"
)

// NavigationQuery 导航查询器，类似于Detour，只负责运行时查询
type NavigationQuery struct {
	navData  *NavigationData
	agent    *Agent
	stepSize float64

	// 缓存的几何体数据（用于碰撞检测）
	geometries []Geometry

	// 空间查询优化
	spatialCache map[Vector3]int // 位置到最近节点的缓存

	// 漏斗算法
	funnelAlgorithm *FunnelAlgorithm
}

// NewNavigationQuery 创建新的导航查询器
func NewNavigationQuery(navData *NavigationData) (*NavigationQuery, error) {
	if err := navData.Validate(); err != nil {
		return nil, fmt.Errorf("invalid navigation data: %v", err)
	}

	// 预先构建索引以提高查询性能
	navData.BuildIndexes()

	// 反序列化几何体数据
	geometries, err := navData.DeserializeGeometries()
	if err != nil {
		return nil, fmt.Errorf("failed to deserialize geometries: %v", err)
	}

	nq := &NavigationQuery{
		navData:         navData,
		stepSize:        navData.StepSize,
		geometries:      geometries,
		spatialCache:    make(map[Vector3]int),
		funnelAlgorithm: NewFunnelAlgorithm(0.5), // 默认Agent半径
	}

	return nq, nil
}

// SetAgent 设置代理
func (nq *NavigationQuery) SetAgent(agent *Agent) {
	nq.agent = agent
	// 更新漏斗算法的Agent半径
	if agent != nil {
		nq.funnelAlgorithm = NewFunnelAlgorithm(agent.Radius)
	} else {
		nq.funnelAlgorithm = NewFunnelAlgorithm(0.5) // 默认半径
	}
}

// GetAgent 获取代理
func (nq *NavigationQuery) GetAgent() *Agent {
	return nq.agent
}

// SetStepSize 设置步长
func (nq *NavigationQuery) SetStepSize(stepSize float64) {
	nq.stepSize = stepSize
}

// GetStepSize 获取步长
func (nq *NavigationQuery) GetStepSize() float64 {
	return nq.stepSize
}

// FindPath 查找路径
func (nq *NavigationQuery) FindPath(start, end Vector3) []Vector3 {
	// 1. 找到起点和终点最近的节点
	startNodeID := nq.findClosestNode(start)
	endNodeID := nq.findClosestNode(end)

	if startNodeID == -1 || endNodeID == -1 {
		return nil
	}

	startTime := time.Now()
	// 2. 使用A*算法查找路径
	nodePath := nq.astar(startNodeID, endNodeID)
	if nodePath == nil {
		return nil
	}
	fmt.Printf("A* algorithm took %v\n", time.Since(startTime))

	// 3. 转换为世界坐标路径
	path := nq.convertToWorldPath(nodePath, start, end)
	fmt.Printf("Path length: %d, took %v\n", len(path), time.Since(startTime))
	return path
}

// findClosestNode 查找最近的节点
func (nq *NavigationQuery) findClosestNode(pos Vector3) int {
	// 检查缓存
	if nodeID, exists := nq.spatialCache[pos]; exists {
		return nodeID
	}

	// 使用Morton索引进行空间查询
	nodeID := nq.findClosestNodeMorton(pos)

	// 缓存结果
	nq.spatialCache[pos] = nodeID

	return nodeID
}

// findClosestNodeMorton 使用Morton编码查找最近节点
func (nq *NavigationQuery) findClosestNodeMorton(pos Vector3) int {
	if len(nq.navData.MortonIndex) == 0 {
		return nq.findClosestNodeBruteForce(pos)
	}

	// 计算查询位置的Morton编码
	queryMorton := Vector3ToMorton(pos, nq.navData.Bounds, nq.navData.MortonResolution)

	// 在Morton索引中进行二分查找
	targetIndex := sort.Search(len(nq.navData.MortonIndex), func(i int) bool {
		nodeID := nq.navData.MortonIndex[i]
		node := &nq.navData.Nodes[nodeID]
		nodeMorton := Vector3ToMorton(node.Center, nq.navData.Bounds, nq.navData.MortonResolution)
		return nodeMorton >= queryMorton
	})

	// 检查周围的节点
	candidates := make([]int, 0, 16)
	searchRadius := 8 // 检查前后8个节点

	for i := targetIndex - searchRadius; i <= targetIndex+searchRadius; i++ {
		if i >= 0 && i < len(nq.navData.MortonIndex) {
			candidates = append(candidates, nq.navData.MortonIndex[i])
		}
	}

	// 找到距离最近的节点
	bestNodeID := -1
	bestDistance := math.MaxFloat64

	for _, nodeID := range candidates {
		node := &nq.navData.Nodes[nodeID]
		distance := pos.Distance(node.Center)

		if distance < bestDistance {
			bestDistance = distance
			bestNodeID = nodeID
		}
	}

	return bestNodeID
}

// findClosestNodeBruteForce 暴力查找最近节点
func (nq *NavigationQuery) findClosestNodeBruteForce(pos Vector3) int {
	bestNodeID := -1
	bestDistance := math.MaxFloat64

	for i, node := range nq.navData.Nodes {
		distance := pos.Distance(node.Center)
		if distance < bestDistance {
			bestDistance = distance
			bestNodeID = i
		}
	}

	return bestNodeID
}

// astar A*寻路算法
func (nq *NavigationQuery) astar(startNodeID, endNodeID int) []int {
	if startNodeID == endNodeID {
		return []int{startNodeID}
	}

	// 初始化开放列表和关闭列表
	openSet := &nodeHeap{}
	heap.Init(openSet)

	closedSet := make(map[int]bool)
	gScore := make(map[int]float64)
	fScore := make(map[int]float64)
	cameFrom := make(map[int]int)
	inOpenSet := make(map[int]*heapNode) // 优化：使用map快速查找开放列表中的节点

	// 起点
	startNode := &nq.navData.Nodes[startNodeID]
	endNode := &nq.navData.Nodes[endNodeID]

	gScore[startNodeID] = 0
	fScore[startNodeID] = nq.heuristic(startNode.Center, endNode.Center)

	startHeapNode := &heapNode{
		nodeID: startNodeID,
		fScore: fScore[startNodeID],
	}
	heap.Push(openSet, startHeapNode)
	inOpenSet[startNodeID] = startHeapNode

	// 添加最大迭代限制，防止无限循环
	maxIterations := 20000
	iterations := 0

	for openSet.Len() > 0 && iterations < maxIterations {
		iterations++

		current := heap.Pop(openSet).(*heapNode)
		currentNodeID := current.nodeID
		delete(inOpenSet, currentNodeID) // 从开放列表映射中删除

		if currentNodeID == endNodeID {
			// 重构路径
			path := make([]int, 0)
			nodeID := endNodeID
			for {
				path = append([]int{nodeID}, path...)
				if nodeID == startNodeID {
					break
				}
				nodeID = cameFrom[nodeID]
			}
			return path
		}

		closedSet[currentNodeID] = true

		// 检查邻居
		neighbors := nq.navData.GetNeighbors(currentNodeID)
		for _, neighborID := range neighbors {
			if closedSet[neighborID] {
				continue
			}

			currentNode := &nq.navData.Nodes[currentNodeID]
			neighborNode := &nq.navData.Nodes[neighborID]

			// 计算移动成本
			tentativeG := gScore[currentNodeID] + currentNode.Center.Distance(neighborNode.Center)

			if existingG, exists := gScore[neighborID]; !exists || tentativeG < existingG {
				cameFrom[neighborID] = currentNodeID
				gScore[neighborID] = tentativeG
				fScore[neighborID] = tentativeG + nq.heuristic(neighborNode.Center, endNode.Center)

				// 优化：使用map快速检查是否已在开放列表中
				if existingHeapNode, exists := inOpenSet[neighborID]; exists {
					// 更新现有节点的fScore并修复堆
					existingHeapNode.fScore = fScore[neighborID]
					heap.Fix(openSet, existingHeapNode.index)
				} else {
					// 添加新节点到开放列表
					newHeapNode := &heapNode{
						nodeID: neighborID,
						fScore: fScore[neighborID],
					}
					heap.Push(openSet, newHeapNode)
					inOpenSet[neighborID] = newHeapNode
				}
			}
		}
	}

	return nil // 没有找到路径
}

// heuristic 启发式函数
func (nq *NavigationQuery) heuristic(a, b Vector3) float64 {
	return a.Distance(b)
}

// convertToWorldPath 将节点路径转换为世界坐标路径
func (nq *NavigationQuery) convertToWorldPath(nodePath []int, start, end Vector3) []Vector3 {
	if len(nodePath) == 0 {
		return nil
	}

	// 将节点ID转换为PathNode结构
	pathNodes := make([]*PathNode, len(nodePath))
	for i, nodeID := range nodePath {
		node := &nq.navData.Nodes[nodeID]
		pathNodes[i] = &PathNode{
			ID:     nodeID,
			Center: node.Center,
			Bounds: node.Bounds,
		}
	}

	// 使用漏斗算法进行路径平滑
	smoothedPath := nq.funnelAlgorithm.SmoothPath(pathNodes)

	// 如果漏斗算法返回空路径，回退到简单方法
	if len(smoothedPath) == 0 {
		path := make([]Vector3, 0, len(nodePath)+2)
		path = append(path, start)
		for _, nodeID := range nodePath {
			node := &nq.navData.Nodes[nodeID]
			path = append(path, node.Center)
		}
		path = append(path, end)
		return path
	}

	// 确保起点和终点正确
	if len(smoothedPath) > 0 {
		smoothedPath[0] = start
		smoothedPath[len(smoothedPath)-1] = end
	}

	return smoothedPath
}

// smoothPath 简单的路径平滑（保留作为备用）
func (nq *NavigationQuery) smoothPath(path []Vector3) []Vector3 {
	if len(path) <= 2 {
		return path
	}

	smoothed := make([]Vector3, 0, len(path))
	smoothed = append(smoothed, path[0])

	i := 0
	for i < len(path)-1 {
		// 尝试跳过中间点
		j := i + 2
		for j < len(path) {
			if nq.isPathClear(smoothed[len(smoothed)-1], path[j]) {
				j++
			} else {
				break
			}
		}

		// 添加最后一个可达点
		smoothed = append(smoothed, path[j-1])
		i = j - 1
	}

	return smoothed
}

// isPathClear 检查路径是否畅通
func (nq *NavigationQuery) isPathClear(start, end Vector3) bool {
	// 简化的路径检查，实际应该根据代理大小进行检查
	steps := int(start.Distance(end) / nq.stepSize)
	if steps <= 1 {
		return true
	}

	direction := end.Sub(start).Scale(1.0 / float64(steps))

	for i := 1; i < steps; i++ {
		point := start.Add(direction.Scale(float64(i)))

		if nq.agent != nil {
			// 检查代理碰撞
			if nq.isAgentOccupied(point) {
				return false
			}
		} else {
			// 检查点碰撞
			if nq.isOccupied(point) {
				return false
			}
		}
	}

	return true
}

// isOccupied 检查点是否被占用
func (nq *NavigationQuery) isOccupied(point Vector3) bool {
	for _, geom := range nq.geometries {
		if geom.ContainsPoint(point) {
			return true
		}
	}
	return false
}

// isAgentOccupied 检查代理是否与障碍物碰撞
func (nq *NavigationQuery) isAgentOccupied(position Vector3) bool {
	if nq.agent == nil {
		return nq.isOccupied(position)
	}

	agentCapsule := nq.agent.GetCapsule(position)

	for _, geom := range nq.geometries {
		if nq.capsuleIntersectsGeometry(agentCapsule, geom) {
			return true
		}
	}

	return false
}

// capsuleIntersectsGeometry 检查胶囊体与几何体的碰撞
func (nq *NavigationQuery) capsuleIntersectsGeometry(capsule Capsule, geom Geometry) bool {
	switch g := geom.(type) {
	case Triangle:
		return nq.capsuleIntersectsTriangle(capsule, g)
	case Box:
		return nq.capsuleIntersectsBox(capsule, g)
	case Capsule:
		return nq.capsuleIntersectsCapsule(capsule, g)
	case ConvexMesh:
		return nq.capsuleIntersectsConvexMesh(capsule, g)
	}
	return false
}

// 这些碰撞检测方法从原来的octree.go中复制过来
func (nq *NavigationQuery) capsuleIntersectsTriangle(capsule Capsule, triangle Triangle) bool {
	// 简化实现：检查胶囊体轴线与三角形的距离
	minDist := pointToLineSegmentDistance(triangle.A, capsule.Start, capsule.End)
	minDist = math.Min(minDist, pointToLineSegmentDistance(triangle.B, capsule.Start, capsule.End))
	minDist = math.Min(minDist, pointToLineSegmentDistance(triangle.C, capsule.Start, capsule.End))

	return minDist <= capsule.Radius
}

func (nq *NavigationQuery) capsuleIntersectsBox(capsule Capsule, box Box) bool {
	// 简化实现：检查胶囊体轴线与盒子的距离
	boxAABB := box.GetBounds()

	// 检查胶囊体轴线的起点和终点是否在扩展的盒子内
	expandedAABB := AABB{
		Min: boxAABB.Min.Sub(Vector3{capsule.Radius, capsule.Radius, capsule.Radius}),
		Max: boxAABB.Max.Add(Vector3{capsule.Radius, capsule.Radius, capsule.Radius}),
	}

	return nq.lineSegmentIntersectsAABB(capsule.Start, capsule.End, expandedAABB)
}

func (nq *NavigationQuery) capsuleIntersectsCapsule(capsule1, capsule2 Capsule) bool {
	// 计算两个胶囊体轴线之间的最短距离
	distance := nq.lineSegmentDistance(capsule1.Start, capsule1.End, capsule2.Start, capsule2.End)
	return distance <= (capsule1.Radius + capsule2.Radius)
}

func (nq *NavigationQuery) capsuleIntersectsConvexMesh(capsule Capsule, mesh ConvexMesh) bool {
	// 简化实现：检查胶囊体是否与凸网格的任何面相交
	for _, face := range mesh.Faces {
		if len(face) >= 3 {
			// 创建三角形并检查相交
			triangle := Triangle{
				A: mesh.Vertices[face[0]],
				B: mesh.Vertices[face[1]],
				C: mesh.Vertices[face[2]],
			}
			if nq.capsuleIntersectsTriangle(capsule, triangle) {
				return true
			}
		}
	}
	return false
}

// 辅助函数：线段与AABB的相交检测
func (nq *NavigationQuery) lineSegmentIntersectsAABB(start, end Vector3, aabb AABB) bool {
	// 简化实现：检查线段的起点或终点是否在AABB内
	if aabb.Contains(start) || aabb.Contains(end) {
		return true
	}

	// 检查线段是否与AABB的任何面相交
	// 这里使用简化算法
	direction := end.Sub(start)
	length := direction.Length()
	if length == 0 {
		return false
	}

	direction = direction.Scale(1.0 / length)

	// 检查与每个轴的交点
	for t := 0.0; t <= length; t += length / 10 {
		point := start.Add(direction.Scale(t))
		if aabb.Contains(point) {
			return true
		}
	}

	return false
}

// 辅助函数：计算两个线段之间的最短距离
func (nq *NavigationQuery) lineSegmentDistance(seg1Start, seg1End, seg2Start, seg2End Vector3) float64 {
	// 简化实现：计算两个线段之间的近似距离
	d1 := pointToLineSegmentDistance(seg1Start, seg2Start, seg2End)
	d2 := pointToLineSegmentDistance(seg1End, seg2Start, seg2End)
	d3 := pointToLineSegmentDistance(seg2Start, seg1Start, seg1End)
	d4 := pointToLineSegmentDistance(seg2End, seg1Start, seg1End)

	return math.Min(math.Min(d1, d2), math.Min(d3, d4))
}

// ToGridCoord 转换为网格坐标（兼容接口）
func (nq *NavigationQuery) ToGridCoord(pos Vector3) (int, int, int) {
	// 简化实现，实际可以根据需要优化
	return int(pos.X / nq.stepSize), int(pos.Y / nq.stepSize), int(pos.Z / nq.stepSize)
}

// GetNavData 获取导航数据
func (nq *NavigationQuery) GetNavData() *NavigationData {
	return nq.navData
}

// ClearCache 清除空间缓存
func (nq *NavigationQuery) ClearCache() {
	nq.spatialCache = make(map[Vector3]int)
}

// SmoothPath 使用漏斗算法平滑路径（兼容接口）
func (nq *NavigationQuery) SmoothPath(path []Vector3) []Vector3 {
	if len(path) <= 2 {
		return path
	}

	// 将Vector3路径转换为PathNode结构以便使用漏斗算法
	pathNodes := make([]*PathNode, len(path))
	for i, point := range path {
		// 为每个点创建一个虚拟的PathNode
		pathNodes[i] = &PathNode{
			ID:     i,
			Center: point,
			Bounds: AABB{
				Min: point.Sub(Vector3{nq.stepSize / 2, nq.stepSize / 2, nq.stepSize / 2}),
				Max: point.Add(Vector3{nq.stepSize / 2, nq.stepSize / 2, nq.stepSize / 2}),
			},
		}
	}

	// 使用漏斗算法平滑路径
	smoothedPath := nq.funnelAlgorithm.SmoothPath(pathNodes)

	// 如果漏斗算法失败，回退到简单平滑
	if len(smoothedPath) == 0 {
		return nq.smoothPath(path)
	}

	return smoothedPath
}

// GetStats 获取统计信息
func (nq *NavigationQuery) GetStats() NavigationStats {
	return NavigationStats{
		NodeCount:     len(nq.navData.Nodes),
		EdgeCount:     len(nq.navData.Edges),
		GeometryCount: len(nq.geometries),
		CacheSize:     len(nq.spatialCache),
		DataSize:      nq.navData.GetDataSize(),
	}
}

// NavigationStats 导航统计信息
type NavigationStats struct {
	NodeCount     int // 节点数量
	EdgeCount     int // 边数量
	GeometryCount int // 几何体数量
	CacheSize     int // 缓存大小
	DataSize      int // 数据大小（字节）
}

// 堆数据结构用于A*算法
type heapNode struct {
	nodeID int
	fScore float64
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
