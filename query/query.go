package query

import (
	"container/heap"
	"fmt"
	"time"

	"github.com/o0olele/octree-go/builder"
	"github.com/o0olele/octree-go/math32"
	"github.com/o0olele/octree-go/octree"
)

// NavigationQuery 导航查询器，类似于Detour，只负责运行时查询
type NavigationQuery struct {
	navData         *builder.NavigationData
	agent           *octree.Agent
	stepSize        float32
	pathPreferences *PathPreferences // 路径偏好配置
}

// NewNavigationQuery 创建新的导航查询器
func NewNavigationQuery(navData *builder.NavigationData) (*NavigationQuery, error) {
	return NewNavigationQueryWithCacheSize(navData)
}

// NewNavigationQueryWithCacheSize 创建新的导航查询器，指定缓存大小
func NewNavigationQueryWithCacheSize(navData *builder.NavigationData) (*NavigationQuery, error) {
	if err := navData.Validate(); err != nil {
		return nil, fmt.Errorf("invalid navigation data: %v", err)
	}

	nq := &NavigationQuery{
		navData:         navData,
		stepSize:        navData.StepSize,
		pathPreferences: DefaultPathPreferences(),
	}

	navData.BuildIndexes()
	return nq, nil
}

func (nq *NavigationQuery) GetOctree() *octree.Octree {
	return nq.navData.GetOctree()
}

// SetAgent 设置代理
func (nq *NavigationQuery) SetAgent(agent *octree.Agent) {
	nq.agent = agent
}

// GetAgent 获取代理
func (nq *NavigationQuery) GetAgent() *octree.Agent {

	return nq.agent
}

// SetStepSize 设置步长
func (nq *NavigationQuery) SetStepSize(stepSize float32) {
	nq.stepSize = stepSize
}

// GetStepSize 获取步长
func (nq *NavigationQuery) GetStepSize() float32 {
	return nq.stepSize
}

// FindPath 查找路径
func (nq *NavigationQuery) FindPath(start, end math32.Vector3) []math32.Vector3 {
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
func (nq *NavigationQuery) findClosestNode(pos math32.Vector3) int32 {
	return nq.navData.FindClosestNodeMorton(pos)
}

// astar A*寻路算法
func (nq *NavigationQuery) astar(startNodeID, endNodeID int32) []int32 {
	if startNodeID == endNodeID {
		return []int32{startNodeID}
	}

	// 初始化开放列表和关闭列表
	openSet := &nodeHeap{}
	heap.Init(openSet)

	closedSet := make(map[int32]bool)
	gScore := make(map[int32]float32)
	fScore := make(map[int32]float32)
	cameFrom := make(map[int32]int32)
	inOpenSet := make(map[int32]*heapNode) // 优化：使用map快速查找开放列表中的节点

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
			path := make([]int32, 0)
			nodeID := endNodeID
			for {
				path = append([]int32{nodeID}, path...)
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

			// 计算移动成本
			tentativeG := gScore[currentNodeID] + nq.calculateEnhancedMovementCost(currentNodeID, neighborID)

			if existingG, exists := gScore[neighborID]; !exists || tentativeG < existingG {
				cameFrom[neighborID] = currentNodeID
				gScore[neighborID] = tentativeG
				neighborNode := &nq.navData.Nodes[neighborID]
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
func (nq *NavigationQuery) heuristic(a, b math32.Vector3) float32 {
	return a.Distance(b)
}

// convertToWorldPath 将节点路径转换为世界坐标路径
func (nq *NavigationQuery) convertToWorldPath(nodePath []int32, start, end math32.Vector3) []math32.Vector3 {
	if len(nodePath) == 0 {
		return nil
	}

	// 将节点ID转换为PathNode结构
	pathNodes := make([]*octree.PathNode, len(nodePath)+2)
	pathNodes[0] = &octree.PathNode{
		Center: start,
		ID:     nodePath[0],
	}
	pathNodes[len(pathNodes)-1] = &octree.PathNode{
		Center: end,
		ID:     nodePath[len(nodePath)-1],
	}
	for i, nodeID := range nodePath {
		node := &nq.navData.Nodes[nodeID]
		pathNodes[i+1] = &octree.PathNode{
			ID:     nodeID,
			Center: node.Center,
			Bounds: node.Bounds,
		}
	}

	// 使用视线优化算法
	smoothedPath := nq.SmoothPath(pathNodes)

	// 如果视线优化算法返回空路径，回退到简单方法
	if len(smoothedPath) == 0 {
		path := make([]math32.Vector3, 0, len(nodePath)+2)
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

// GetNavData 获取导航数据
func (nq *NavigationQuery) GetNavData() *builder.NavigationData {
	return nq.navData
}

// GetStats 获取统计信息
func (nq *NavigationQuery) GetStats() NavigationStats {
	return NavigationStats{
		NodeCount:     len(nq.navData.Nodes),
		EdgeCount:     len(nq.navData.Edges),
		GeometryCount: nq.navData.GetGeometryCount(),
		CacheSize:     nq.navData.GetSpatialCacheCount(),
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
