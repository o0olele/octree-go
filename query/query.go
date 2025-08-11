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

func (nq *NavigationQuery) GetNavigationData() *builder.NavigationData {
	return nq.navData
}

// FindPath 查找路径
func (nq *NavigationQuery) FindPath(start, end math32.Vector3) []math32.Vector3 {
	// 1. 找到起点和终点最近的节点
	startNodeID := nq.FindClosestNode(start)
	endNodeID := nq.FindClosestNode(end)

	if startNodeID == -1 || endNodeID == -1 {
		return nil
	}

	// 2. 优先使用双向A*算法查找路径
	startTime := time.Now()
	nodePath := nq.AstarBidirectional(startNodeID, endNodeID)
	if nodePath != nil {
		fmt.Printf("Bi-directional A* algorithm took %v\n", time.Since(startTime))
	} else {
		// 回退到单向A*
		startTime = time.Now()
		nodePath = nq.Astar(startNodeID, endNodeID)
		if nodePath == nil {
			return nil
		}
		fmt.Printf("A* algorithm took %v\n", time.Since(startTime))
	}

	// 3. 转换为世界坐标路径
	path := nq.convertToWorldPath(nodePath, start, end)
	fmt.Printf("Path length: %d, took %v\n", len(path), time.Since(startTime))
	return path
}

// FindClosestNode 查找最近的节点
func (nq *NavigationQuery) FindClosestNode(pos math32.Vector3) int32 {
	return nq.navData.FindClosestNodeMorton(pos)
}

// Astar A*寻路算法
func (nq *NavigationQuery) Astar(startNodeID, endNodeID int32) []int32 {
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

	startHeapNode := newHeapNode(startNodeID, fScore[startNodeID])
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
					newHeapNode := newHeapNode(neighborID, fScore[neighborID])
					heap.Push(openSet, newHeapNode)
					inOpenSet[neighborID] = newHeapNode
				}
			}
		}
	}

	openSet.Clear()
	return nil // 没有找到路径
}

// AstarBidirectional 双向A*寻路算法
func (nq *NavigationQuery) AstarBidirectional(startNodeID, endNodeID int32) []int32 {
	if startNodeID == endNodeID {
		return []int32{startNodeID}
	}

	// 前向与后向开放列表
	forwardOpen := &nodeHeap{}
	backwardOpen := &nodeHeap{}
	heap.Init(forwardOpen)
	heap.Init(backwardOpen)

	// 关闭列表
	forwardClosed := make(map[int32]bool)
	backwardClosed := make(map[int32]bool)

	// gScore 与来源映射
	gForward := make(map[int32]float32)
	gBackward := make(map[int32]float32)
	cameFromForward := make(map[int32]int32)
	cameFromBackward := make(map[int32]int32)

	// 开放列表中的节点引用，便于 O(1) 更新
	inForwardOpen := make(map[int32]*heapNode)
	inBackwardOpen := make(map[int32]*heapNode)

	// 起点与终点
	startNode := &nq.navData.Nodes[startNodeID]
	endNode := &nq.navData.Nodes[endNodeID]

	// 初始化
	gForward[startNodeID] = 0
	gBackward[endNodeID] = 0
	startF := nq.heuristic(startNode.Center, endNode.Center)
	endF := nq.heuristic(endNode.Center, startNode.Center)

	startHeapNode := newHeapNode(startNodeID, startF)
	endHeapNode := newHeapNode(endNodeID, endF)
	heap.Push(forwardOpen, startHeapNode)
	inForwardOpen[startNodeID] = startHeapNode
	heap.Push(backwardOpen, endHeapNode)
	inBackwardOpen[endNodeID] = endHeapNode

	maxIterations := 20000
	iterations := 0

	var meetingNode int32 = -1

	for forwardOpen.Len() > 0 && backwardOpen.Len() > 0 && iterations < maxIterations {
		iterations++

		// 选择扩展方向：取当前两个堆顶 fScore 较小的一侧
		expandForward := false
		if backwardOpen.Len() == 0 {
			expandForward = true
		} else if forwardOpen.Len() == 0 {
			expandForward = false
		} else {
			if (*forwardOpen)[0].fScore <= (*backwardOpen)[0].fScore {
				expandForward = true
			}
		}

		if expandForward {
			current := heap.Pop(forwardOpen).(*heapNode)
			currentID := current.nodeID
			delete(inForwardOpen, currentID)

			if backwardClosed[currentID] {
				meetingNode = currentID
				break
			}

			forwardClosed[currentID] = true

			neighbors := nq.navData.GetNeighbors(currentID)
			for _, neighborID := range neighbors {
				if forwardClosed[neighborID] {
					continue
				}

				tentativeG := gForward[currentID] + nq.calculateEnhancedMovementCost(currentID, neighborID)
				if existingG, exists := gForward[neighborID]; !exists || tentativeG < existingG {
					cameFromForward[neighborID] = currentID
					gForward[neighborID] = tentativeG

					neighborNode := &nq.navData.Nodes[neighborID]
					f := tentativeG + nq.heuristic(neighborNode.Center, endNode.Center)

					if existingHeapNode, exists := inForwardOpen[neighborID]; exists {
						existingHeapNode.fScore = f
						heap.Fix(forwardOpen, existingHeapNode.index)
					} else {
						newHeapNode := newHeapNode(neighborID, f)
						heap.Push(forwardOpen, newHeapNode)
						inForwardOpen[neighborID] = newHeapNode
					}
				}

				if backwardClosed[neighborID] {
					meetingNode = neighborID
					break
				}
			}
			if meetingNode != -1 {
				break
			}
		} else {
			current := heap.Pop(backwardOpen).(*heapNode)
			currentID := current.nodeID
			delete(inBackwardOpen, currentID)

			if forwardClosed[currentID] {
				meetingNode = currentID
				break
			}

			backwardClosed[currentID] = true

			neighbors := nq.navData.GetNeighbors(currentID)
			for _, neighborID := range neighbors {
				if backwardClosed[neighborID] {
					continue
				}

				tentativeG := gBackward[currentID] + nq.calculateEnhancedMovementCost(currentID, neighborID)
				if existingG, exists := gBackward[neighborID]; !exists || tentativeG < existingG {
					cameFromBackward[neighborID] = currentID
					gBackward[neighborID] = tentativeG

					neighborNode := &nq.navData.Nodes[neighborID]
					f := tentativeG + nq.heuristic(neighborNode.Center, startNode.Center)

					if existingHeapNode, exists := inBackwardOpen[neighborID]; exists {
						existingHeapNode.fScore = f
						heap.Fix(backwardOpen, existingHeapNode.index)
					} else {
						newHeapNode := newHeapNode(neighborID, f)
						heap.Push(backwardOpen, newHeapNode)
						inBackwardOpen[neighborID] = newHeapNode
					}
				}

				if forwardClosed[neighborID] {
					meetingNode = neighborID
					break
				}
			}
			if meetingNode != -1 {
				break
			}
		}
	}

	forwardOpen.Clear()
	backwardOpen.Clear()
	if meetingNode == -1 {
		return nil
	}

	// 重构路径：start -> meetingNode
	forwardPath := make([]int32, 0)
	{
		node := meetingNode
		for {
			forwardPath = append([]int32{node}, forwardPath...)
			if node == startNodeID {
				break
			}
			parent, ok := cameFromForward[node]
			if !ok {
				// 如果前向没有记录，可能 meetingNode 就是 startNodeID 或由后向抵达
				break
			}
			node = parent
		}
	}

	// meetingNode -> end（使用后向 cameFrom）
	backwardPath := make([]int32, 0)
	{
		node := meetingNode
		for node != endNodeID {
			next, ok := cameFromBackward[node]
			if !ok {
				break
			}
			// 追加下一个到终点的方向
			backwardPath = append(backwardPath, next)
			node = next
		}
	}

	// 合并，避免重复 meetingNode
	combined := make([]int32, 0, len(forwardPath)+len(backwardPath))
	combined = append(combined, forwardPath...)
	if len(backwardPath) > 0 {
		// backwardPath 已经从 meetingNode 后一个开始
		combined = append(combined, backwardPath...)
	}

	// 最终校验首尾
	if len(combined) == 0 {
		return nil
	}
	if combined[0] != startNodeID {
		// 若前半未包含 start，则补上
		combined = append([]int32{startNodeID}, combined...)
	}
	if combined[len(combined)-1] != endNodeID {
		combined = append(combined, endNodeID)
	}

	return combined
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
