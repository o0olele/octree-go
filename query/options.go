package query

import "github.com/o0olele/octree-go/builder"

// PathPreferences 路径偏好配置
type PathPreferences struct {
	InteriorPathBonus    float32 // 内部路径奖励（降低成本的比例）
	BoundaryPathPenalty  float32 // 边界路径惩罚（增加成本的比例）
	DensityBonus         float32 // 高密度区域奖励
	BoundaryThreshold    float32 // 边界阈值（相对于stepSize的倍数）
	EnablePathPreference bool    // 是否启用路径偏好
	UsingCrossPoint      bool
	UsingDouglasPeucker  bool
}

// DefaultPathPreferences 返回默认的路径偏好配置
func DefaultPathPreferences() *PathPreferences {
	return &PathPreferences{
		InteriorPathBonus:    0.2,  // 内部路径成本降低20%
		BoundaryPathPenalty:  0.5,  // 边界路径成本增加50%
		DensityBonus:         0.2,  // 高密度区域成本降低20%
		BoundaryThreshold:    3.0,  // 边界阈值为stepSize的3倍
		EnablePathPreference: true, // 默认启用路径偏好
		UsingCrossPoint:      false,
		UsingDouglasPeucker:  true,
	}
}

func (nq *NavigationQuery) GetPathPreferences() *PathPreferences {
	return nq.pathPreferences
}

// calculateEnhancedMovementCost 计算增强的移动成本，考虑路径偏好
func (nq *NavigationQuery) calculateEnhancedMovementCost(currentNodeID, neighborNodeID int32) float32 {
	currentNode := &nq.navData.Nodes[currentNodeID]
	neighborNode := &nq.navData.Nodes[neighborNodeID]

	// 基础距离成本
	baseCost := currentNode.Center.Distance(neighborNode.Center)

	// 如果未启用路径偏好，直接返回基础成本
	if !nq.pathPreferences.EnablePathPreference {
		return baseCost
	}

	// 检查是否为边界节点（靠近场景边界的节点）- 使用预计算缓存
	currentIsBoundary := nq.isNodeNearBoundary(currentNode)
	neighborIsBoundary := nq.isNodeNearBoundary(neighborNode)

	// 应用路径偏好权重
	var costMultiplier float32 = 1.0

	if currentIsBoundary && neighborIsBoundary {
		// 两个节点都在边界附近，增加成本以降低优先级
		costMultiplier = 1.0 + nq.pathPreferences.BoundaryPathPenalty
	} else if currentIsBoundary || neighborIsBoundary {
		// 只有一个节点在边界附近，轻微增加成本
		costMultiplier = 1.0 + (nq.pathPreferences.BoundaryPathPenalty * 0.5)
	} else {
		// 两个节点都不在边界附近，给予内部路径奖励
		costMultiplier = 1.0 - nq.pathPreferences.InteriorPathBonus
	}

	// 检查节点密度，密度高的区域（如房间内部）成本较低
	densityFactor := nq.calculateNodeDensityFactor(currentNodeID, neighborNodeID)
	costMultiplier *= densityFactor

	return baseCost * costMultiplier
}

// isNodeNearBoundary 检查节点是否靠近场景边界
func (nq *NavigationQuery) isNodeNearBoundary(node *builder.CompactNode) bool {
	// 使用预计算的节点到边界最小距离判断
	// 阈值仍然来自配置：BoundaryThreshold * stepSize
	threshold := nq.stepSize * nq.pathPreferences.BoundaryThreshold
	dist := nq.navData.GetNodeBoundaryDistance(node.ID)
	return dist < threshold
}

// calculateNodeDensityFactor 计算节点密度因子
func (nq *NavigationQuery) calculateNodeDensityFactor(currentNodeID, neighborNodeID int32) float32 {
	// 使用预计算的邻接度而非实时查询
	currentNeighbors := nq.navData.GetNodeNeighborCount(currentNodeID)
	neighborNeighbors := nq.navData.GetNodeNeighborCount(neighborNodeID)

	avgNeighbors := float32(currentNeighbors+neighborNeighbors) / 2.0

	if avgNeighbors > 6 {
		return 1.0 - nq.pathPreferences.DensityBonus
	} else if avgNeighbors > 4 {
		return 1.0 - (nq.pathPreferences.DensityBonus * 0.5)
	} else if avgNeighbors < 3 {
		return 1.0 + (nq.pathPreferences.DensityBonus * 1.5)
	}
	return 1.0
}
