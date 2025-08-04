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
		UsingCrossPoint:      true,
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

	// 检查是否为边界节点（靠近场景边界的节点）
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
	bounds := nq.navData.Bounds
	boundaryThreshold := nq.stepSize * nq.pathPreferences.BoundaryThreshold // 使用配置的边界阈值

	center := node.Center

	// 检查是否靠近任何边界
	return (center.X-bounds.Min.X) < boundaryThreshold ||
		(bounds.Max.X-center.X) < boundaryThreshold ||
		(center.Y-bounds.Min.Y) < boundaryThreshold ||
		(bounds.Max.Y-center.Y) < boundaryThreshold ||
		(center.Z-bounds.Min.Z) < boundaryThreshold ||
		(bounds.Max.Z-center.Z) < boundaryThreshold
}

// calculateNodeDensityFactor 计算节点密度因子
func (nq *NavigationQuery) calculateNodeDensityFactor(currentNodeID, neighborNodeID int32) float32 {
	// 计算当前节点周围的邻居数量
	currentNeighbors := len(nq.navData.GetNeighbors(currentNodeID))
	neighborNeighbors := len(nq.navData.GetNeighbors(neighborNodeID))

	// 平均邻居数量
	avgNeighbors := float32(currentNeighbors+neighborNeighbors) / 2.0

	// 高密度区域（邻居多）成本较低，低密度区域成本较高
	if avgNeighbors > 6 {
		return 1.0 - nq.pathPreferences.DensityBonus // 使用配置的密度奖励
	} else if avgNeighbors > 4 {
		return 1.0 - (nq.pathPreferences.DensityBonus * 0.5) // 使用一半的密度奖励
	} else if avgNeighbors < 3 {
		return 1.0 + (nq.pathPreferences.DensityBonus * 1.5) // 低密度区域增加成本
	}

	return 1.0 // 默认成本
}
