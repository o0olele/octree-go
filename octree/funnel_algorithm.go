package octree

import (
	"math"
)

// FunnelAlgorithm 漏斗算法实现
type FunnelAlgorithm struct {
	agentRadius float64
	octree      *Octree // 添加对八叉树的引用，用于路径检查
}

// NewFunnelAlgorithm 创建漏斗算法实例
func NewFunnelAlgorithm(agentRadius float64) *FunnelAlgorithm {
	return &FunnelAlgorithm{
		agentRadius: agentRadius,
		octree:      nil, // 默认不设置八叉树，需要外部设置
	}
}

// SetOctree 设置八叉树引用
func (fa *FunnelAlgorithm) SetOctree(octree *Octree) {
	fa.octree = octree
}

// Portal 表示两个节点之间的通道
type Portal struct {
	Left  Vector3
	Right Vector3
}

// SmoothPath 使用简化的漏斗算法平滑路径
func (fa *FunnelAlgorithm) SmoothPath(pathNodes []*PathNode) []Vector3 {
	if len(pathNodes) <= 2 {
		// 对于短路径，直接返回节点中心
		result := make([]Vector3, len(pathNodes))
		for i, node := range pathNodes {
			result[i] = node.Center
		}
		return result
	}

	// return fa.stringPull(fa.buildPortals(pathNodes))
	// 使用改进的直线优化算法
	return fa.optimizePathWithLineOfSight(pathNodes)
}

// optimizePathWithLineOfSight 使用视线优化算法简化路径
func (fa *FunnelAlgorithm) optimizePathWithLineOfSight(pathNodes []*PathNode) []Vector3 {
	if len(pathNodes) <= 2 {
		result := make([]Vector3, len(pathNodes))
		for i, node := range pathNodes {
			result[i] = node.Center
		}
		return result
	}

	smoothed := []Vector3{pathNodes[0].Center}
	current := 0

	for current < len(pathNodes)-1 {
		// 尝试找到最远的可直达节点
		farthest := current
		for next := current + 1; next < len(pathNodes); next++ {
			if fa.canConnectDirectly(pathNodes[current], pathNodes[next]) {
				farthest = next
			}
		}

		// 如果没有找到更远的节点，则只前进到下一个节点
		if farthest == current {
			farthest = current + 1
		}

		// 添加最远可达点
		smoothed = append(smoothed, pathNodes[farthest].Center)
		current = farthest
	}

	return smoothed
}

// canConnectDirectly 检查两个节点是否可以直接连接
func (fa *FunnelAlgorithm) canConnectDirectly(node1, node2 *PathNode) bool {
	// 首先进行快速距离检查
	distance := node1.Center.Distance(node2.Center)
	maxDistance := math.Max(node1.Bounds.Size().Length(), node2.Bounds.Size().Length()) * 10 // 放宽距离限制
	if distance > maxDistance {
		return false
	}

	// 如果有八叉树引用，使用更准确的路径检查
	if fa.octree != nil {
		// 检查两点间的路径是否畅通
		if fa.isPathClear(node1.Center, node2.Center) {
			return true
		}
		return false
	}

	// 如果没有八叉树引用，使用启发式方法
	// 检查两个节点中心点之间是否可以直接连接
	expansion := math.Max(fa.agentRadius, math.Min(node1.Bounds.Size().Length(), node2.Bounds.Size().Length())*0.5)

	expandedBounds1 := AABB{
		Min: Vector3{
			X: node1.Bounds.Min.X - expansion,
			Y: node1.Bounds.Min.Y - expansion,
			Z: node1.Bounds.Min.Z - expansion,
		},
		Max: Vector3{
			X: node1.Bounds.Max.X + expansion,
			Y: node1.Bounds.Max.Y + expansion,
			Z: node1.Bounds.Max.Z + expansion,
		},
	}

	expandedBounds2 := AABB{
		Min: Vector3{
			X: node2.Bounds.Min.X - expansion,
			Y: node2.Bounds.Min.Y - expansion,
			Z: node2.Bounds.Min.Z - expansion,
		},
		Max: Vector3{
			X: node2.Bounds.Max.X + expansion,
			Y: node2.Bounds.Max.Y + expansion,
			Z: node2.Bounds.Max.Z + expansion,
		},
	}

	// 如果扩展后的边界框相交，认为可以直接连接
	if expandedBounds1.Intersects(expandedBounds2) {
		return true
	}

	// 更宽松的间隙检查
	minDistance := fa.calculateAABBDistance(node1.Bounds, node2.Bounds)
	allowedGap := expansion * 2 // 放宽允许的间隙

	return minDistance <= allowedGap
}

// calculateAABBDistance 计算两个AABB之间的最小距离
func (fa *FunnelAlgorithm) calculateAABBDistance(aabb1, aabb2 AABB) float64 {
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

// isPathClear 检查两点之间的路径是否畅通
func (fa *FunnelAlgorithm) isPathClear(start, end Vector3) bool {
	if fa.octree == nil {
		return true // 如果没有八叉树，假设路径畅通
	}

	// 计算方向向量和距离
	direction := end.Sub(start)
	distance := direction.Length()

	if distance < 0.001 { // 距离太近，认为是同一点
		return true
	}

	// 标准化方向向量
	direction = direction.Scale(1.0 / distance)

	// 使用适当的步长进行采样检查
	stepSize := math.Max(0.1, fa.agentRadius*0.5)
	steps := int(math.Ceil(distance / stepSize))

	// 沿着路径进行采样检测
	for i := 0; i <= steps; i++ {
		t := float64(i) / float64(steps)
		samplePoint := start.Add(direction.Scale(distance * t))

		// 检查采样点是否被占用
		if fa.octree.IsOccupied(samplePoint) {
			return false
		}

		// 如果有Agent半径，还需要检查Agent碰撞
		if fa.agentRadius > 0 {
			agent := &Agent{
				Radius: fa.agentRadius,
				Height: fa.agentRadius * 2, // 简化的高度设置
			}
			if fa.octree.IsAgentOccupied(agent, samplePoint) {
				return false
			}
		}
	}

	return true
}

// SmoothPathSimple 最简化的路径平滑（备用方法）
func (fa *FunnelAlgorithm) SmoothPathSimple(pathNodes []*PathNode) []Vector3 {
	if len(pathNodes) <= 2 {
		result := make([]Vector3, len(pathNodes))
		for i, node := range pathNodes {
			result[i] = node.Center
		}
		return result
	}

	// 使用更保守的跳跃策略
	smoothed := []Vector3{pathNodes[0].Center}
	i := 0

	for i < len(pathNodes)-1 {
		// 只尝试跳过一个节点
		if i+2 < len(pathNodes) && fa.canConnectDirectly(pathNodes[i], pathNodes[i+2]) {
			smoothed = append(smoothed, pathNodes[i+2].Center)
			i += 2
		} else {
			smoothed = append(smoothed, pathNodes[i+1].Center)
			i++
		}
	}

	return smoothed
}

// 以下是原有的复杂漏斗算法实现，保留作为备用
// buildPortals 从路径节点构建通道列表
func (fa *FunnelAlgorithm) buildPortals(pathNodes []*PathNode) []Portal {
	if len(pathNodes) < 2 {
		return []Portal{}
	}

	portals := make([]Portal, 0, len(pathNodes)+1)

	// 起始通道（起点）
	startPortal := Portal{
		Left:  pathNodes[0].Center,
		Right: pathNodes[0].Center,
	}
	portals = append(portals, startPortal)

	// 中间通道
	for i := 0; i < len(pathNodes)-1; i++ {
		currentNode := pathNodes[i]
		nextNode := pathNodes[i+1]

		portal := fa.buildPortalBetweenNodes(currentNode, nextNode)
		portals = append(portals, portal)
	}

	// 结束通道（终点）
	endPortal := Portal{
		Left:  pathNodes[len(pathNodes)-1].Center,
		Right: pathNodes[len(pathNodes)-1].Center,
	}
	portals = append(portals, endPortal)

	return portals
}

// buildPortalBetweenNodes 在两个节点之间构建通道
func (fa *FunnelAlgorithm) buildPortalBetweenNodes(node1, node2 *PathNode) Portal {
	// 使用节点边界的交集中心作为通道
	center1 := node1.Center
	center2 := node2.Center

	// 计算中点
	midpoint := center1.Add(center2).Scale(0.5)

	// 计算通道宽度（基于节点大小）
	size1 := node1.Bounds.Size()
	size2 := node2.Bounds.Size()
	avgSize := (size1.Length() + size2.Length()) * 0.25

	// 计算垂直于连接方向的向量
	direction := center2.Sub(center1)
	if direction.Length() > 0 {
		direction = direction.Scale(1.0 / direction.Length())
	}
	perpendicular := Vector3{X: -direction.Z, Y: 0, Z: direction.X}

	left := midpoint.Add(perpendicular.Scale(avgSize))
	right := midpoint.Sub(perpendicular.Scale(avgSize))

	return Portal{Left: left, Right: right}
}

// calculateBoundaryIntersection 计算两个节点边界的交集（保留原实现）
func (fa *FunnelAlgorithm) calculateBoundaryIntersection(node1, node2 *PathNode) Portal {
	// 简化实现：基于节点边界框计算交集
	bounds1 := node1.Bounds
	bounds2 := node2.Bounds

	// 找到两个边界框的重叠区域
	overlapMin := Vector3{
		X: math.Max(bounds1.Min.X, bounds2.Min.X),
		Y: math.Max(bounds1.Min.Y, bounds2.Min.Y),
		Z: math.Max(bounds1.Min.Z, bounds2.Min.Z),
	}

	overlapMax := Vector3{
		X: math.Min(bounds1.Max.X, bounds2.Max.X),
		Y: math.Min(bounds1.Max.Y, bounds2.Max.Y),
		Z: math.Min(bounds1.Max.Z, bounds2.Max.Z),
	}

	// 如果没有重叠，使用节点中心之间的中点
	if overlapMin.X > overlapMax.X || overlapMin.Y > overlapMax.Y || overlapMin.Z > overlapMax.Z {
		center := node1.Center.Add(node2.Center).Scale(0.5)
		return Portal{Left: center, Right: center}
	}

	// 计算重叠区域的边界点
	direction := node2.Center.Sub(node1.Center)
	if direction.Length() > 0 {
		direction = direction.Scale(1.0 / direction.Length())
	}
	perpendicular := Vector3{X: -direction.Z, Y: direction.Y, Z: direction.X}

	center := overlapMin.Add(overlapMax).Scale(0.5)
	halfWidth := math.Min(overlapMax.X-overlapMin.X, overlapMax.Z-overlapMin.Z) * 0.5

	left := center.Add(perpendicular.Scale(halfWidth))
	right := center.Sub(perpendicular.Scale(halfWidth))

	return Portal{Left: left, Right: right}
}

// stringPull 执行漏斗算法的核心逻辑（保留但不使用）
func (fa *FunnelAlgorithm) stringPull(portals []Portal) []Vector3 {
	if len(portals) <= 1 {
		return []Vector3{}
	}

	path := make([]Vector3, 0, len(portals))

	// 初始化漏斗
	apex := portals[0].Left
	portalLeft := portals[1].Left
	portalRight := portals[1].Right
	leftIndex := 1
	rightIndex := 1

	// 添加起点
	path = append(path, apex)

	for i := 2; i < len(portals); i++ {
		left := portals[i].Left
		right := portals[i].Right

		// 更新右侧
		if fa.triArea2(apex, portalRight, right) <= 0.0 {
			if fa.vEqual(apex, portalRight) || fa.triArea2(apex, portalLeft, right) > 0.0 {
				// 收紧漏斗
				portalRight = right
				rightIndex = i
			} else {
				// 右侧越过左侧，添加左侧点到路径并重启
				path = append(path, portalLeft)
				apex = portalLeft

				// 重置漏斗
				portalLeft = apex
				portalRight = apex
				leftIndex = rightIndex
				// rightIndex = rightIndex // 删除自赋值

				// 重启扫描
				i = rightIndex
				continue
			}
		}

		// 更新左侧
		if fa.triArea2(apex, portalLeft, left) >= 0.0 {
			if fa.vEqual(apex, portalLeft) || fa.triArea2(apex, portalRight, left) < 0.0 {
				// 收紧漏斗
				portalLeft = left
				leftIndex = i
			} else {
				// 左侧越过右侧，添加右侧点到路径并重启
				path = append(path, portalRight)
				apex = portalRight

				// 重置漏斗
				portalLeft = apex
				portalRight = apex
				leftIndex = leftIndex // 这里应该是正确的，保持原样
				rightIndex = leftIndex

				// 重启扫描
				i = leftIndex
				continue
			}
		}
	}

	// 添加终点
	if len(portals) > 0 {
		endPoint := portals[len(portals)-1].Left
		if len(path) == 0 || !fa.vEqual(path[len(path)-1], endPoint) {
			path = append(path, endPoint)
		}
	}

	return path
}

// triArea2 计算三角形面积的两倍（用于判断点的相对位置）
func (fa *FunnelAlgorithm) triArea2(a, b, c Vector3) float64 {
	// 使用2D投影到XZ平面
	ax := b.X - a.X
	az := b.Z - a.Z
	bx := c.X - a.X
	bz := c.Z - a.Z
	return bx*az - ax*bz
}

// vEqual 判断两个向量是否相等（带容差）
func (fa *FunnelAlgorithm) vEqual(a, b Vector3) bool {
	const epsilon = 0.001
	return a.Distance(b) < epsilon
}
