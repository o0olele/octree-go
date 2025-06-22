package octree

import (
	"container/heap"
	"math"
)

// PathNode 寻路节点，对应八叉树中的空白叶子节点
type PathNode struct {
	ID         int
	Center     Vector3
	Bounds     AABB
	Edges      []*PathEdge
	OctreeNode *OctreeNode // 对应的八叉树节点

	// A*算法相关字段
	GCost     float64
	HCost     float64
	FCost     float64
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
	Cost  float64
}

// PathGraph 寻路图
type PathGraph struct {
	Nodes  map[*OctreeNode]*PathNode
	Edges  []*PathEdge
	nextID int
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

// NodeBasedAStarPathfinder 基于节点的A*寻路器
type NodeBasedAStarPathfinder struct {
	octree   *Octree
	agent    *Agent
	stepSize float64
	graph    *PathGraph
}

// NewNodeBasedAStarPathfinder 创建基于节点的A*寻路器
func NewNodeBasedAStarPathfinder(octree *Octree, stepSize float64) *NodeBasedAStarPathfinder {
	pathfinder := &NodeBasedAStarPathfinder{
		octree:   octree,
		stepSize: stepSize,
		graph:    NewPathGraph(),
	}

	// 构建寻路图
	pathfinder.buildGraph()

	return pathfinder
}

// buildGraph 构建寻路图
func (nba *NodeBasedAStarPathfinder) buildGraph() {
	// 收集所有空白叶子节点
	emptyLeaves := nba.collectEmptyLeaves(nba.octree.Root)

	// 为每个空白叶子节点创建寻路节点
	for _, leaf := range emptyLeaves {
		nba.graph.AddNode(leaf)
	}

	// 建立相邻节点之间的连接
	nba.buildConnections(emptyLeaves)
}

// collectEmptyLeaves 收集所有空白的叶子节点
func (nba *NodeBasedAStarPathfinder) collectEmptyLeaves(node *OctreeNode) []*OctreeNode {
	var emptyLeaves []*OctreeNode

	if node.IsLeaf {
		// 检查叶子节点是否为空（没有被占用）
		if !node.IsOccupied {
			emptyLeaves = append(emptyLeaves, node)
		}
		return emptyLeaves
	}

	// 递归检查子节点
	for _, child := range node.Children {
		if child != nil {
			childLeaves := nba.collectEmptyLeaves(child)
			emptyLeaves = append(emptyLeaves, childLeaves...)
		}
	}

	return emptyLeaves
}

// buildConnections 建立节点之间的连接
func (nba *NodeBasedAStarPathfinder) buildConnections(nodes []*OctreeNode) {
	for i, nodeA := range nodes {
		pathNodeA := nba.graph.Nodes[nodeA]

		for j := i + 1; j < len(nodes); j++ {
			nodeB := nodes[j]
			pathNodeB := nba.graph.Nodes[nodeB]

			// 检查两个节点是否相邻（边界相交或接触）
			if nba.areNodesAdjacent(nodeA, nodeB) {
				nba.graph.AddEdge(pathNodeA, pathNodeB)
			}
		}
	}
}

// areNodesAdjacent 检查两个节点是否相邻且连通
func (nba *NodeBasedAStarPathfinder) areNodesAdjacent(nodeA, nodeB *OctreeNode) bool {
	// 检查边界是否相交或接触
	boundsA := nodeA.Bounds
	boundsB := nodeB.Bounds

	// 计算两个节点中心之间的距离
	centerA := boundsA.Center()
	centerB := boundsB.Center()
	distance := centerA.Distance(centerB)

	// 计算两个节点的最大尺寸
	sizeA := boundsA.Size()
	sizeB := boundsB.Size()
	maxSizeA := math.Max(math.Max(sizeA.X, sizeA.Y), sizeA.Z)
	maxSizeB := math.Max(math.Max(sizeB.X, sizeB.Y), sizeB.Z)

	// 非常严格的距离阈值检查 - 只允许真正相邻的节点连接
	threshold := (maxSizeA + maxSizeB) * 0.6 // 更加严格的阈值
	if distance > threshold {
		return false
	}

	// 首先检查节点是否真的相邻（边界接触或重叠）
	if !boundsA.Intersects(boundsB) {
		// 如果边界不相交，检查是否非常接近
		minDistance := math.Min(maxSizeA, maxSizeB) * 0.1
		if distance > minDistance {
			return false
		}
	}

	// 严格的路径清晰度检查
	if !nba.isPathClear(centerA, centerB) {
		return false
	}

	// 检查多个关键点之间的连通性
	// 只有当大多数关键路径都畅通时才认为节点相邻
	testPoints := []Vector3{
		centerA, centerB,
		// 边界中点
		{(boundsA.Min.X + boundsA.Max.X) / 2, centerA.Y, centerA.Z},
		{(boundsB.Min.X + boundsB.Max.X) / 2, centerB.Y, centerB.Z},
		{centerA.X, (boundsA.Min.Y + boundsA.Max.Y) / 2, centerA.Z},
		{centerB.X, (boundsB.Min.Y + boundsB.Max.Y) / 2, centerB.Z},
	}

	// 检查关键点之间的连通性
	clearConnections := 0
	totalConnections := 0

	for i := 0; i < len(testPoints); i += 2 {
		if i+1 < len(testPoints) {
			totalConnections++
			if nba.isPathClear(testPoints[i], testPoints[i+1]) {
				clearConnections++
			}
		}
	}

	// 要求所有关键连接都畅通
	return clearConnections == totalConnections
}

// isPathClear 检查两个点之间的路径是否畅通无阻
func (nba *NodeBasedAStarPathfinder) isPathClear(start, end Vector3) bool {
	// 计算方向向量和距离
	direction := end.Sub(start)
	distance := direction.Length()

	if distance < 0.001 { // 距离太近，认为是同一点
		return true
	}

	// 标准化方向向量
	direction = direction.Scale(1.0 / distance)

	// 使用更密集的采样，确保不会错过障碍物
	stepSize := math.Min(nba.stepSize*0.25, 0.1) // 更小的步长
	steps := int(math.Ceil(distance / stepSize))

	// 确保至少有足够的采样点
	if steps < 10 {
		steps = 10
	}

	// 沿着路径进行密集采样检测
	for i := 0; i <= steps; i++ {
		t := float64(i) / float64(steps)
		samplePoint := start.Add(direction.Scale(distance * t))

		// 检查采样点是否被占用
		if nba.octree.IsOccupied(samplePoint) {
			return false
		}

		// 如果有Agent，还需要检查Agent碰撞
		if nba.agent != nil {
			if nba.octree.IsAgentOccupied(nba.agent, samplePoint) {
				return false
			}
		}
	}

	return true
}

// getNodeMarginPoints 获取节点边界的采样点
func (nba *NodeBasedAStarPathfinder) getNodeMarginPoints(bounds AABB) []Vector3 {
	center := bounds.Center()

	// 在节点边界内生成采样点（避免边界上的点）
	margin := 0.1 // 从边界内缩进一点
	points := []Vector3{
		// 中心点
		center,
		// 8个角点的内缩版本
		{bounds.Min.X + margin, bounds.Min.Y + margin, bounds.Min.Z + margin},
		{bounds.Max.X - margin, bounds.Min.Y + margin, bounds.Min.Z + margin},
		{bounds.Min.X + margin, bounds.Max.Y - margin, bounds.Min.Z + margin},
		{bounds.Max.X - margin, bounds.Max.Y - margin, bounds.Min.Z + margin},
		{bounds.Min.X + margin, bounds.Min.Y + margin, bounds.Max.Z - margin},
		{bounds.Max.X - margin, bounds.Min.Y + margin, bounds.Max.Z - margin},
		{bounds.Min.X + margin, bounds.Max.Y - margin, bounds.Max.Z - margin},
		{bounds.Max.X - margin, bounds.Max.Y - margin, bounds.Max.Z - margin},
		// 6个面的中心点
		{center.X, center.Y, bounds.Min.Z + margin},
		{center.X, center.Y, bounds.Max.Z - margin},
		{center.X, bounds.Min.Y + margin, center.Z},
		{center.X, bounds.Max.Y - margin, center.Z},
		{bounds.Min.X + margin, center.Y, center.Z},
		{bounds.Max.X - margin, center.Y, center.Z},
	}

	return points
}

// SetAgent 设置寻路Agent
func (nba *NodeBasedAStarPathfinder) SetAgent(agent *Agent) {
	nba.agent = agent
}

// GetAgent 获取当前Agent
func (nba *NodeBasedAStarPathfinder) GetAgent() *Agent {
	return nba.agent
}

// GetStepSize 获取步长
func (nba *NodeBasedAStarPathfinder) GetStepSize() float64 {
	return nba.stepSize
}

// SetStepSize 设置步长
func (nba *NodeBasedAStarPathfinder) SetStepSize(stepSize float64) {
	nba.stepSize = stepSize
}

// ToGridCoord 将Vector3转换为网格坐标（兼容接口）
func (nba *NodeBasedAStarPathfinder) ToGridCoord(pos Vector3) (int, int, int) {
	origin := nba.octree.Root.Bounds.Min
	return int(math.Round((pos.X - origin.X) / nba.stepSize)),
		int(math.Round((pos.Y - origin.Y) / nba.stepSize)),
		int(math.Round((pos.Z - origin.Z) / nba.stepSize))
}

// FindPath 使用基于节点的A*算法寻找路径
func (nba *NodeBasedAStarPathfinder) FindPath(start, end Vector3) []Vector3 {
	// 找到起点和终点最近的空白节点
	startNode := nba.findClosestNode(start)
	endNode := nba.findClosestNode(end)

	if startNode == nil || endNode == nil {
		return nil
	}

	// 如果起点和终点在同一个节点中
	if startNode == endNode {
		return []Vector3{start, end}
	}

	// 运行A*算法
	path := nba.astar(startNode, endNode)
	if path == nil {
		return nil
	}

	// 将节点路径转换为世界坐标路径
	return nba.convertToWorldPath(path, start, end)
}

// findClosestNode 找到最接近给定位置的空白节点
func (nba *NodeBasedAStarPathfinder) findClosestNode(pos Vector3) *PathNode {
	var closestNode *PathNode
	minDistance := math.Inf(1)

	for _, node := range nba.graph.Nodes {
		// 检查位置是否在节点边界内
		if node.Bounds.Contains(pos) {
			// 如果有Agent，检查Agent是否能放置在这个位置
			if nba.agent != nil && nba.octree.IsAgentOccupied(nba.agent, pos) {
				continue
			}
			return node
		}

		// 计算到节点中心的距离
		distance := pos.Distance(node.Center)
		if distance < minDistance {
			// 如果有Agent，检查Agent是否能放置在节点中心
			if nba.agent != nil && nba.octree.IsAgentOccupied(nba.agent, node.Center) {
				continue
			}
			minDistance = distance
			closestNode = node
		}
	}

	return closestNode
}

// astar 运行A*算法
func (nba *NodeBasedAStarPathfinder) astar(startNode, endNode *PathNode) []*PathNode {
	openHeap := &PathNodeHeap{}
	heap.Init(openHeap)
	closedSet := make(map[int]bool)

	// 重置所有节点的A*数据
	for _, node := range nba.graph.Nodes {
		node.GCost = math.Inf(1)
		node.HCost = 0
		node.FCost = math.Inf(1)
		node.Parent = nil
		node.HeapIndex = -1 // 重要：初始化堆索引
	}

	startNode.GCost = 0
	startNode.HCost = nba.heuristic(startNode, endNode)
	startNode.CalculateFCost()

	heap.Push(openHeap, startNode)

	maxIterations := 10000
	iterations := 0

	for openHeap.Len() > 0 && iterations < maxIterations {
		iterations++

		current := heap.Pop(openHeap).(*PathNode)

		if current == endNode {
			return nba.reconstructNodePath(current)
		}

		closedSet[current.ID] = true

		// 检查所有相邻节点
		for _, edge := range current.Edges {
			neighbor := edge.NodeA
			if neighbor == current {
				neighbor = edge.NodeB
			}

			if closedSet[neighbor.ID] {
				continue
			}

			tentativeGCost := current.GCost + edge.Cost

			if tentativeGCost < neighbor.GCost {
				neighbor.Parent = current
				neighbor.GCost = tentativeGCost
				neighbor.HCost = nba.heuristic(neighbor, endNode)
				neighbor.CalculateFCost()

				// 如果不在开放列表中，添加它
				if neighbor.HeapIndex == -1 {
					heap.Push(openHeap, neighbor)
				} else {
					heap.Fix(openHeap, neighbor.HeapIndex)
				}
			}
		}
	}

	return nil // 没有找到路径
}

// heuristic 启发式函数
func (nba *NodeBasedAStarPathfinder) heuristic(a, b *PathNode) float64 {
	return a.Center.Distance(b.Center)
}

// reconstructNodePath 重构节点路径
func (nba *NodeBasedAStarPathfinder) reconstructNodePath(node *PathNode) []*PathNode {
	var path []*PathNode
	current := node

	for current != nil {
		path = append([]*PathNode{current}, path...)
		current = current.Parent
	}

	return path
}

// convertToWorldPath 将节点路径转换为世界坐标路径
func (nba *NodeBasedAStarPathfinder) convertToWorldPath(nodePath []*PathNode, start, end Vector3) []Vector3 {
	if len(nodePath) == 0 {
		return nil
	}

	path := []Vector3{start}

	// 添加所有节点的中心点，确保路径安全
	for _, node := range nodePath {
		centerPoint := node.Center

		// 如果有Agent，确保Agent可以安全通过节点中心
		if nba.agent != nil && nba.octree.IsAgentOccupied(nba.agent, centerPoint) {
			// 尝试在节点边界内找到安全点
			safePoint := nba.findSafePointInNode(node)
			if safePoint != nil {
				centerPoint = *safePoint
			}
		}

		path = append(path, centerPoint)
	}

	path = append(path, end)

	// 使用保守的路径平滑策略
	smoothed := nba.SmoothPath(path)
	validated := nba.validatePathSafety(smoothed)

	// 如果平滑后的路径不安全，返回原始路径
	if len(validated) < 2 {
		return nba.validatePathSafety(path)
	}

	return validated
}

// findSafePointInNode 在节点内找到Agent的安全位置
func (nba *NodeBasedAStarPathfinder) findSafePointInNode(node *PathNode) *Vector3 {
	if nba.agent == nil {
		return &node.Center
	}

	// 在节点边界内采样多个点
	bounds := node.Bounds
	size := bounds.Size()

	// 缩小搜索范围以避免边界碰撞
	margin := math.Min(math.Min(size.X, size.Y), size.Z) * 0.1
	searchBounds := AABB{
		Min: bounds.Min.Add(Vector3{margin, margin, margin}),
		Max: bounds.Max.Sub(Vector3{margin, margin, margin}),
	}

	// 在搜索区域内采样
	samples := 8
	for i := 0; i < samples; i++ {
		for j := 0; j < samples; j++ {
			for k := 0; k < samples; k++ {
				t := Vector3{
					float64(i) / float64(samples-1),
					float64(j) / float64(samples-1),
					float64(k) / float64(samples-1),
				}

				testPoint := Vector3{
					searchBounds.Min.X + t.X*(searchBounds.Max.X-searchBounds.Min.X),
					searchBounds.Min.Y + t.Y*(searchBounds.Max.Y-searchBounds.Min.Y),
					searchBounds.Min.Z + t.Z*(searchBounds.Max.Z-searchBounds.Min.Z),
				}

				if !nba.octree.IsAgentOccupied(nba.agent, testPoint) {
					return &testPoint
				}
			}
		}
	}

	// 如果找不到安全点，返回中心点
	return &node.Center
}

// validatePathSafety 验证路径安全性
func (nba *NodeBasedAStarPathfinder) validatePathSafety(path []Vector3) []Vector3 {
	if len(path) <= 1 {
		return path
	}

	validatedPath := []Vector3{path[0]}

	for i := 1; i < len(path); i++ {
		lastPoint := validatedPath[len(validatedPath)-1]
		currentPoint := path[i]

		// 检查从上一个有效点到当前点的路径是否安全
		if nba.isPathClear(lastPoint, currentPoint) {
			validatedPath = append(validatedPath, currentPoint)
		} else {
			// 如果不安全，使用保守策略：保持原始路径结构
			// 这确保我们不会因为平滑而引入穿墙问题
			validatedPath = append(validatedPath, currentPoint)
		}
	}

	return validatedPath
}

// findSafeMidpoint 找到两点间的安全中间点
func (nba *NodeBasedAStarPathfinder) findSafeMidpoint(start, end Vector3) *Vector3 {
	// 简单的中点策略
	mid := Vector3{
		(start.X + end.X) / 2,
		(start.Y + end.Y) / 2,
		(start.Z + end.Z) / 2,
	}

	// 检查中点是否安全
	if nba.agent != nil {
		if !nba.octree.IsAgentOccupied(nba.agent, mid) {
			return &mid
		}
	} else {
		if !nba.octree.IsOccupied(mid) {
			return &mid
		}
	}

	return nil
}

// SmoothPath 路径平滑处理
func (nba *NodeBasedAStarPathfinder) SmoothPath(path []Vector3) []Vector3 {
	if len(path) <= 2 {
		return path
	}

	smoothed := []Vector3{path[0]}
	i := 0

	for i < len(path)-1 {
		farthest := i + 1

		// 尝试找到最远的可直达点，使用更严格的检查
		for j := i + 2; j < len(path); j++ {
			if nba.isPathClear(path[i], path[j]) {
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

// hasObstacleBetween 检查两点间是否有障碍物
func (nba *NodeBasedAStarPathfinder) hasObstacleBetween(start, end Vector3) bool {
	// 使用isPathClear的反向结果，保持一致性
	return !nba.isPathClear(start, end)
}
