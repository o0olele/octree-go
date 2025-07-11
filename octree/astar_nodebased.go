package octree

import (
	"container/heap"
	"fmt"
	"math"
	"runtime"
	"sync"
	"sync/atomic"
	"time"
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

	// Morton编码优化
	mortonSortedNodes []MortonNodePair
	mortonResolution  uint32

	// 漏斗算法
	funnelAlgorithm *FunnelAlgorithm
}

// MortonNodePair Morton编码和节点的配对
type MortonNodePair struct {
	Morton MortonCode
	Node   *PathNode
}

// NewNodeBasedAStarPathfinder 创建基于节点的A*寻路器
func NewNodeBasedAStarPathfinder(octree *Octree, stepSize float64) *NodeBasedAStarPathfinder {
	return NewNodeBasedAStarPathfinderWithParallel(octree, stepSize, false)
}

// NewNodeBasedAStarPathfinderWithParallel 创建基于节点的A*寻路器，可选择是否使用并行构建
func NewNodeBasedAStarPathfinderWithParallel(octree *Octree, stepSize float64, useParallel bool) *NodeBasedAStarPathfinder {
	pathfinder := &NodeBasedAStarPathfinder{
		octree:           octree,
		stepSize:         stepSize,
		graph:            NewPathGraph(),
		mortonResolution: 1024,                    // 10位精度
		funnelAlgorithm:  NewFunnelAlgorithm(0.5), // 默认Agent半径
	}

	// 构建寻路图
	pathfinder.buildGraphWithMode(useParallel)

	// 构建Morton索引
	pathfinder.buildMortonIndex()

	return pathfinder
}

// buildGraph 构建寻路图
func (nba *NodeBasedAStarPathfinder) buildGraph() {
	nba.buildGraphWithMode(false) // 默认使用串行版本
}

// buildGraphParallel 使用并行版本构建寻路图
func (nba *NodeBasedAStarPathfinder) buildGraphParallel() {
	nba.buildGraphWithMode(true) // 使用并行版本
}

// buildGraphWithMode 构建寻路图，可选择串行或并行模式
func (nba *NodeBasedAStarPathfinder) buildGraphWithMode(useParallel bool) {
	// 收集所有空白叶子节点
	emptyLeaves := nba.collectEmptyLeaves(nba.octree.Root)

	// 为每个空白叶子节点创建寻路节点
	for _, leaf := range emptyLeaves {
		nba.graph.AddNode(leaf)
	}

	// 建立相邻节点之间的连接
	beginTime := time.Now()
	if useParallel {
		nba.buildConnectionsParallel(emptyLeaves)
	} else {
		nba.buildConnections(emptyLeaves)
	}
	fmt.Printf("PathGraph建立连接: 耗时 %s\n", time.Since(beginTime))
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
	totalConnections := 0
	totalChecks := 0

	for i, nodeA := range nodes {
		pathNodeA := nba.graph.Nodes[nodeA]

		for j := i + 1; j < len(nodes); j++ {
			nodeB := nodes[j]
			pathNodeB := nba.graph.Nodes[nodeB]
			totalChecks++

			// 检查两个节点是否相邻（边界相交或接触）
			if nba.areNodesAdjacent(nodeA, nodeB) {
				nba.graph.AddEdge(pathNodeA, pathNodeB)
				totalConnections++
			}
		}
	}

	// 输出调试信息
	fmt.Printf("PathGraph建立连接: 检查了 %d 对节点, 建立了 %d 个连接\n", totalChecks, totalConnections)
	fmt.Printf("节点总数: %d, 边总数: %d\n", len(nba.graph.Nodes), len(nba.graph.Edges))
}

// buildConnectionsParallel 并行版本的建立节点之间的连接（智能版本）
func (nba *NodeBasedAStarPathfinder) buildConnectionsParallel(nodes []*OctreeNode) {
	totalNodes := len(nodes)

	if totalNodes <= 1 {
		fmt.Printf("PathGraph建立连接: 节点数量不足，无需建立连接\n")
		return
	}

	// 计算总的节点对数量
	totalPairs := (totalNodes * (totalNodes - 1)) / 2

	// 智能决策：如果节点对数量太少，直接使用串行版本
	const PARALLEL_THRESHOLD = 1000 // 小于1000个节点对时使用串行
	if totalPairs < PARALLEL_THRESHOLD {
		fmt.Printf("PathGraph建立连接: 节点对数量(%d)较少，使用串行版本\n", totalPairs)
		nba.buildConnections(nodes)
		return
	}

	numWorkers := runtime.NumCPU()

	// 对于大数据集，限制worker数量以减少竞争
	if totalPairs < 10000 {
		numWorkers = min(numWorkers, 4) // 最多4个worker
	}

	fmt.Printf("PathGraph建立连接: 节点对数量(%d)较多，使用并行版本(workers: %d)\n", totalPairs, numWorkers)

	// 使用批量处理减少原子操作
	batchSize := max(totalPairs/(numWorkers*4), 50) // 每个批次至少50个

	totalConnections := int64(0)
	totalChecks := int64(0)

	// 使用切片存储结果，避免动态分配
	type EdgeResult struct {
		pathNodeA *PathNode
		pathNodeB *PathNode
	}

	// 每个worker有自己的结果切片
	workerResults := make([][]EdgeResult, numWorkers)
	for i := range workerResults {
		workerResults[i] = make([]EdgeResult, 0, totalPairs/numWorkers+100)
	}

	var wg sync.WaitGroup
	currentPair := int64(0)

	for workerID := 0; workerID < numWorkers; workerID++ {
		wg.Add(1)
		go func(id int) {
			defer wg.Done()

			localConnections := int64(0)
			localChecks := int64(0)

			for {
				// 批量获取工作，减少原子操作频率
				startIdx := atomic.AddInt64(&currentPair, int64(batchSize)) - int64(batchSize)
				endIdx := startIdx + int64(batchSize)
				if endIdx > int64(totalPairs) {
					endIdx = int64(totalPairs)
				}

				if startIdx >= int64(totalPairs) {
					break
				}

				// 处理这个批次
				for pairIdx := startIdx; pairIdx < endIdx; pairIdx++ {
					// 将线性索引转换为(i,j)
					i, j := nba.pairIndexToIJ(int(pairIdx), totalNodes)
					if i >= j || i >= totalNodes || j >= totalNodes {
						continue
					}

					nodeA := nodes[i]
					nodeB := nodes[j]
					localChecks++

					// 检查两个节点是否相邻
					if nba.areNodesAdjacent(nodeA, nodeB) {
						pathNodeA := nba.graph.Nodes[nodeA]
						pathNodeB := nba.graph.Nodes[nodeB]

						workerResults[id] = append(workerResults[id], EdgeResult{
							pathNodeA: pathNodeA,
							pathNodeB: pathNodeB,
						})
						localConnections++
					}
				}
			}

			// 更新全局计数器
			atomic.AddInt64(&totalConnections, localConnections)
			atomic.AddInt64(&totalChecks, localChecks)
		}(workerID)
	}

	// 等待所有worker完成
	wg.Wait()

	// 在主线程中添加所有边
	finalConnections := 0
	for _, results := range workerResults {
		for _, edge := range results {
			nba.graph.AddEdge(edge.pathNodeA, edge.pathNodeB)
			finalConnections++
		}
	}

	// 输出调试信息
	fmt.Printf("PathGraph并行建立连接: 检查了 %d 对节点, 建立了 %d 个连接 (使用 %d 个工作线程)\n",
		totalChecks, totalConnections, numWorkers)
	fmt.Printf("节点总数: %d, 边总数: %d\n", len(nba.graph.Nodes), len(nba.graph.Edges))
}

// pairIndexToIJ 将线性配对索引转换为(i,j)坐标
func (nba *NodeBasedAStarPathfinder) pairIndexToIJ(pairIndex, n int) (int, int) {
	// 使用数学公式直接计算
	i := int((2*float64(n) - 1 - math.Sqrt(float64((2*n-1)*(2*n-1)-8*pairIndex))) / 2)
	j := pairIndex - i*(2*n-i-1)/2 + i + 1
	return i, j
}

// min 返回两个int中的较小值
func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}

// max 返回两个int中的较大值
func max(a, b int) int {
	if a > b {
		return a
	}
	return b
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

	// 放宽距离阈值检查 - 允许更多的相邻节点连接
	threshold := (maxSizeA + maxSizeB) * 0.8 // 放宽阈值从0.6到0.8
	if distance > threshold {
		return false
	}

	// 更宽松的边界检查
	if !boundsA.Intersects(boundsB) {
		// 计算两个边界框之间的最小距离
		minDistance := nba.calculateAABBDistance(boundsA, boundsB)
		// 允许小的间隙，特别是对于不同大小的节点
		allowedGap := math.Min(maxSizeA, maxSizeB) * 0.2 // 增加允许的间隙
		if minDistance > allowedGap {
			return false
		}
	}

	// 路径清晰度检查 - 使用更宽松的采样
	if !nba.isPathClearRelaxed(centerA, centerB) {
		return false
	}

	return true
}

// calculateAABBDistance 计算两个AABB之间的最小距离
func (nba *NodeBasedAStarPathfinder) calculateAABBDistance(aabb1, aabb2 AABB) float64 {
	// 如果相交，距离为0
	if aabb1.Intersects(aabb2) {
		return 0.0
	}

	// 计算每个轴上的距离
	dx := math.Max(0, math.Max(aabb1.Min.X-aabb2.Max.X, aabb2.Min.X-aabb1.Max.X))
	dy := math.Max(0, math.Max(aabb1.Min.Y-aabb2.Max.Y, aabb2.Min.Y-aabb1.Max.Y))
	dz := math.Max(0, math.Max(aabb1.Min.Z-aabb2.Max.Z, aabb2.Min.Z-aabb1.Max.Z))

	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

// isPathClearRelaxed 使用更宽松的参数检查路径是否畅通
func (nba *NodeBasedAStarPathfinder) isPathClearRelaxed(start, end Vector3) bool {
	// 计算方向向量和距离
	direction := end.Sub(start)
	distance := direction.Length()

	if distance < 0.001 { // 距离太近，认为是同一点
		return true
	}

	// 标准化方向向量
	direction = direction.Scale(1.0 / distance)

	// 使用较少的采样点，提高连通性
	stepSize := math.Min(nba.stepSize*0.5, 0.2) // 增加步长，减少采样密度
	steps := int(math.Ceil(distance / stepSize))

	// 确保有合理的采样点数量
	if steps < 5 {
		steps = 5
	}
	if steps > 20 { // 限制最大采样点数量
		steps = 20
	}

	// 沿着路径进行采样检测
	for i := 0; i <= steps; i++ {
		t := float64(i) / float64(steps)
		samplePoint := start.Add(direction.Scale(distance * t))

		// 检查采样点是否被占用
		if nba.octree.IsOccupied(samplePoint) {
			return false
		}

		// 如果有Agent，还需要检查Agent碰撞，但使用稍微小一点的Agent
		if nba.agent != nil {
			// 创建一个稍微小一点的Agent进行检测，增加连通性
			relaxedAgent := &Agent{
				Radius: nba.agent.Radius * 0.9, // 减小10%
				Height: nba.agent.Height * 0.9, // 减小10%
			}
			if nba.octree.IsAgentOccupied(relaxedAgent, samplePoint) {
				return false
			}
		}
	}

	return true
}

// isPathClear 检查两个点之间的路径是否畅通无阻（向后兼容方法）
func (nba *NodeBasedAStarPathfinder) isPathClear(start, end Vector3) bool {
	return nba.isPathClearRelaxed(start, end)
}

// debugNodeAdjacency 调试两个节点的相邻性检测
func (nba *NodeBasedAStarPathfinder) debugNodeAdjacency(nodeA, nodeB *OctreeNode) {
	boundsA := nodeA.Bounds
	boundsB := nodeB.Bounds
	centerA := boundsA.Center()
	centerB := boundsB.Center()
	distance := centerA.Distance(centerB)

	sizeA := boundsA.Size()
	sizeB := boundsB.Size()
	maxSizeA := math.Max(math.Max(sizeA.X, sizeA.Y), sizeA.Z)
	maxSizeB := math.Max(math.Max(sizeB.X, sizeB.Y), sizeB.Z)

	threshold := (maxSizeA + maxSizeB) * 0.8
	boundsIntersect := boundsA.Intersects(boundsB)
	minDistance := nba.calculateAABBDistance(boundsA, boundsB)
	allowedGap := math.Min(maxSizeA, maxSizeB) * 0.2
	pathClear := nba.isPathClearRelaxed(centerA, centerB)

	fmt.Printf("节点相邻性调试:\n")
	fmt.Printf("  中心距离: %.3f, 阈值: %.3f, 通过: %v\n", distance, threshold, distance <= threshold)
	fmt.Printf("  边界相交: %v\n", boundsIntersect)
	if !boundsIntersect {
		fmt.Printf("  最小距离: %.3f, 允许间隙: %.3f, 通过: %v\n", minDistance, allowedGap, minDistance <= allowedGap)
	}
	fmt.Printf("  路径畅通: %v\n", pathClear)
	fmt.Printf("  最终结果: %v\n", nba.areNodesAdjacent(nodeA, nodeB))
	fmt.Printf("  节点A中心: (%.2f, %.2f, %.2f), 大小: %.2f\n", centerA.X, centerA.Y, centerA.Z, maxSizeA)
	fmt.Printf("  节点B中心: (%.2f, %.2f, %.2f), 大小: %.2f\n", centerB.X, centerB.Y, centerB.Z, maxSizeB)
	fmt.Println()
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
	// 更新漏斗算法的Agent半径
	if agent != nil {
		nba.funnelAlgorithm = NewFunnelAlgorithm(agent.Radius)
	}
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
	startTime := time.Now()
	startNode := nba.findClosestNode(start)
	endNode := nba.findClosestNode(end)
	fmt.Printf("FindPath 时间: %v\n", time.Since(startTime))

	if startNode == nil || endNode == nil {
		return nil
	}

	// 如果起点和终点在同一个节点中
	if startNode == endNode {
		return []Vector3{start, end}
	}

	// 运行A*算法
	startTime = time.Now()
	path := nba.astar(startNode, endNode)
	fmt.Printf("Astar 时间: %v\n", time.Since(startTime))
	if path == nil {
		return nil
	}

	startTime = time.Now()
	// 将节点路径转换为世界坐标路径
	worldPath := nba.convertToWorldPath(path, start, end)
	fmt.Printf("ConvertToWorldPath 时间: %v\n", time.Since(startTime))
	return worldPath
}

// findClosestNode 找到最接近给定位置的空白节点
func (nba *NodeBasedAStarPathfinder) findClosestNode(pos Vector3) *PathNode {
	// 首先尝试直接找到包含该位置的节点
	containingNode := nba.findContainingNode(pos)
	if containingNode != nil {
		return containingNode
	}

	// 使用Morton编码优化的空间查询
	if len(nba.mortonSortedNodes) > 0 {
		return nba.findClosestNodeMorton(pos)
	}

	// 回退到原始方法
	return nba.findClosestNodeSpatial(pos)
}

// findContainingNode 找到包含给定位置的节点
func (nba *NodeBasedAStarPathfinder) findContainingNode(pos Vector3) *PathNode {
	// 从根节点开始搜索
	octreeNode := nba.findContainingOctreeNode(nba.octree.Root, pos)
	if octreeNode != nil {
		if pathNode, exists := nba.graph.Nodes[octreeNode]; exists {
			// 如果有Agent，检查Agent是否能放置在这个位置
			if nba.agent != nil && nba.octree.IsAgentOccupied(nba.agent, pos) {
				return nil
			}
			return pathNode
		}
	}
	return nil
}

// findContainingOctreeNode 在八叉树中找到包含位置的叶子节点
func (nba *NodeBasedAStarPathfinder) findContainingOctreeNode(node *OctreeNode, pos Vector3) *OctreeNode {
	if node == nil || !node.Bounds.Contains(pos) {
		return nil
	}

	if node.IsLeaf {
		if !node.IsOccupied {
			return node
		}
		return nil
	}

	// 递归搜索子节点
	for _, child := range node.Children {
		if child != nil {
			result := nba.findContainingOctreeNode(child, pos)
			if result != nil {
				return result
			}
		}
	}

	return nil
}

// findClosestNodeSpatial 使用空间查询找到最近的节点
func (nba *NodeBasedAStarPathfinder) findClosestNodeSpatial(pos Vector3) *PathNode {
	var closestNode *PathNode
	minDistance := math.Inf(1)

	// 使用广度优先搜索，从最可能的区域开始
	candidates := nba.getSpatialCandidates(pos, 10) // 获取最近的10个候选节点

	for _, node := range candidates {
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

	// 如果候选节点中没有找到合适的，回退到全局搜索
	if closestNode == nil {
		return nba.findClosestNodeBruteForce(pos)
	}

	return closestNode
}

// getSpatialCandidates 获取空间上最近的候选节点
func (nba *NodeBasedAStarPathfinder) getSpatialCandidates(pos Vector3, maxCandidates int) []*PathNode {
	type nodeDistance struct {
		node     *PathNode
		distance float64
	}

	candidates := make([]nodeDistance, 0)

	// 收集所有节点的距离信息
	for _, node := range nba.graph.Nodes {
		distance := pos.Distance(node.Center)
		candidates = append(candidates, nodeDistance{node, distance})
	}

	// 按距离排序
	for i := 0; i < len(candidates)-1; i++ {
		for j := i + 1; j < len(candidates); j++ {
			if candidates[i].distance > candidates[j].distance {
				candidates[i], candidates[j] = candidates[j], candidates[i]
			}
		}
	}

	// 返回最近的候选节点
	result := make([]*PathNode, 0, maxCandidates)
	for i := 0; i < len(candidates) && i < maxCandidates; i++ {
		result = append(result, candidates[i].node)
	}

	return result
}

// findClosestNodeBruteForce 暴力搜索最近节点（备用方法）
func (nba *NodeBasedAStarPathfinder) findClosestNodeBruteForce(pos Vector3) *PathNode {
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
		node.HeapIndex = -1
	}

	startNode.GCost = 0
	startNode.HCost = nba.heuristic(startNode, endNode)
	startNode.CalculateFCost()

	heap.Push(openHeap, startNode)

	maxIterations := 20000
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

	startTime := time.Now()

	// 对于短路径，使用传统方法
	if len(nodePath) <= 3 {
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
		smoothed := nba.SmoothPath(path)
		validated := nba.validatePathSafety(smoothed)
		fmt.Printf("ConvertToWorldPath 时间: %v (传统方法)\n", time.Since(startTime))

		if len(validated) < 2 {
			return nba.validatePathSafety(path)
		}
		return validated
	}

	// 对于长路径，使用漏斗算法
	funnelPath := nba.funnelAlgorithm.SmoothPath(nodePath)

	// 如果漏斗算法失败，回退到传统方法
	if len(funnelPath) == 0 {
		path := []Vector3{start}

		for _, node := range nodePath {
			centerPoint := node.Center
			if nba.agent != nil && nba.octree.IsAgentOccupied(nba.agent, centerPoint) {
				safePoint := nba.findSafePointInNode(node)
				if safePoint != nil {
					centerPoint = *safePoint
				}
			}
			path = append(path, centerPoint)
		}

		path = append(path, end)
		funnelPath = path
	} else {
		// 确保起点和终点正确
		if len(funnelPath) > 0 {
			funnelPath[0] = start
			funnelPath[len(funnelPath)-1] = end
		}
	}

	// 后处理：轻度平滑和验证
	smoothed := nba.SmoothPath(funnelPath)
	validated := nba.validatePathSafety(smoothed)

	fmt.Printf("ConvertToWorldPath 时间: %v (漏斗算法)\n", time.Since(startTime))

	if len(validated) < 2 {
		return nba.validatePathSafety(funnelPath)
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

// GetPathGraph 获取路径图数据，用于可视化
func (nba *NodeBasedAStarPathfinder) GetPathGraph() *PathGraph {
	return nba.graph
}

// PathGraphData 用于JSON序列化的路径图数据结构
type PathGraphData struct {
	Nodes []PathNodeData `json:"nodes"`
	Edges []PathEdgeData `json:"edges"`
}

// PathNodeData 用于JSON序列化的路径节点数据
type PathNodeData struct {
	ID     int     `json:"id"`
	Center Vector3 `json:"center"`
	Bounds AABB    `json:"bounds"`
}

// PathEdgeData 用于JSON序列化的路径边数据
type PathEdgeData struct {
	NodeAID int     `json:"node_a_id"`
	NodeBID int     `json:"node_b_id"`
	Cost    float64 `json:"cost"`
}

// ToPathGraphData 将PathGraph转换为可序列化的数据结构
func (nba *NodeBasedAStarPathfinder) ToPathGraphData() *PathGraphData {
	if nba.graph == nil {
		return &PathGraphData{
			Nodes: []PathNodeData{},
			Edges: []PathEdgeData{},
		}
	}

	// 转换节点数据
	nodes := make([]PathNodeData, 0, len(nba.graph.Nodes))
	for _, node := range nba.graph.Nodes {
		nodes = append(nodes, PathNodeData{
			ID:     node.ID,
			Center: node.Center,
			Bounds: node.Bounds,
		})
	}

	// 转换边数据
	edges := make([]PathEdgeData, 0, len(nba.graph.Edges))
	for _, edge := range nba.graph.Edges {
		edges = append(edges, PathEdgeData{
			NodeAID: edge.NodeA.ID,
			NodeBID: edge.NodeB.ID,
			Cost:    edge.Cost,
		})
	}

	return &PathGraphData{
		Nodes: nodes,
		Edges: edges,
	}
}

// buildMortonIndex 构建基于Morton编码的空间索引
func (nba *NodeBasedAStarPathfinder) buildMortonIndex() {
	// 为所有节点计算Morton编码
	nba.mortonSortedNodes = make([]MortonNodePair, 0, len(nba.graph.Nodes))

	for _, node := range nba.graph.Nodes {
		morton := Vector3ToMorton(node.Center, nba.octree.Root.Bounds, nba.mortonResolution)
		nba.mortonSortedNodes = append(nba.mortonSortedNodes, MortonNodePair{
			Morton: morton,
			Node:   node,
		})
	}

	// 按Morton编码排序
	nba.sortNodesByMorton()
}

// sortNodesByMorton 按Morton编码排序节点
func (nba *NodeBasedAStarPathfinder) sortNodesByMorton() {
	// 使用快速排序按Morton编码排序
	for i := 0; i < len(nba.mortonSortedNodes)-1; i++ {
		for j := i + 1; j < len(nba.mortonSortedNodes); j++ {
			if nba.mortonSortedNodes[i].Morton > nba.mortonSortedNodes[j].Morton {
				nba.mortonSortedNodes[i], nba.mortonSortedNodes[j] = nba.mortonSortedNodes[j], nba.mortonSortedNodes[i]
			}
		}
	}
}

// findClosestNodeMorton 使用Morton编码快速找到最近的节点
func (nba *NodeBasedAStarPathfinder) findClosestNodeMorton(pos Vector3) *PathNode {
	if len(nba.mortonSortedNodes) == 0 {
		return nil
	}

	// 计算目标位置的Morton编码
	targetMorton := Vector3ToMorton(pos, nba.octree.Root.Bounds, nba.mortonResolution)

	// 二分查找最接近的Morton编码
	left, right := 0, len(nba.mortonSortedNodes)-1
	bestIndex := 0

	for left <= right {
		mid := (left + right) / 2
		if nba.mortonSortedNodes[mid].Morton <= targetMorton {
			bestIndex = mid
			left = mid + 1
		} else {
			right = mid - 1
		}
	}

	// 检查前后几个候选节点，找到实际距离最近的
	candidates := nba.getMortonCandidates(bestIndex, 10)

	var closestNode *PathNode
	minDistance := math.Inf(1)

	for _, candidate := range candidates {
		distance := pos.Distance(candidate.Center)
		if distance < minDistance {
			// 如果有Agent，检查Agent是否能放置在节点中心
			if nba.agent != nil && nba.octree.IsAgentOccupied(nba.agent, candidate.Center) {
				continue
			}
			minDistance = distance
			closestNode = candidate
		}
	}

	return closestNode
}

// getMortonCandidates 获取Morton索引附近的候选节点
func (nba *NodeBasedAStarPathfinder) getMortonCandidates(centerIndex, count int) []*PathNode {
	candidates := make([]*PathNode, 0, count)

	// 从中心向两边扩展
	for i := 0; i < count && (centerIndex-i >= 0 || centerIndex+i < len(nba.mortonSortedNodes)); i++ {
		// 向左扩展
		if centerIndex-i >= 0 {
			candidates = append(candidates, nba.mortonSortedNodes[centerIndex-i].Node)
		}
		// 向右扩展（避免重复添加中心节点）
		if i > 0 && centerIndex+i < len(nba.mortonSortedNodes) {
			candidates = append(candidates, nba.mortonSortedNodes[centerIndex+i].Node)
		}
	}

	return candidates
}
