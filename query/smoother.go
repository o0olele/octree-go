package query

import (
	"fmt"

	"github.com/o0olele/octree-go/math32"
	"github.com/o0olele/octree-go/octree"
)

// optimizePathWithLineOfSight 使用视线优化算法简化路径
func (nq *NavigationQuery) SmoothPath(pathNodes []*octree.PathNode) []math32.Vector3 {

	if nq.pathPreferences.UsingCrossPoint {
		return nq.optimizePathWithLineOfSight(nq.getCrossingPath(pathNodes))
	}

	if len(pathNodes) <= 2 {
		result := make([]math32.Vector3, len(pathNodes))
		for i, node := range pathNodes {
			result[i] = node.Center
		}
		return result
	}

	smoothed := []math32.Vector3{pathNodes[0].Center}
	current := 0

	for current < len(pathNodes)-1 {
		// 尝试找到最远的可直达节点
		farthest := current
		for next := current + 1; next < len(pathNodes); next++ {
			if nq.isPathClear(pathNodes[current].Center, pathNodes[next].Center) {
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

// optimizePathWithLineOfSight 使用视线优化算法简化路径
func (nq *NavigationQuery) optimizePathWithLineOfSight(pathNodes []math32.Vector3) []math32.Vector3 {
	if len(pathNodes) <= 2 {
		return pathNodes
	}

	smoothed := []math32.Vector3{pathNodes[0]}
	current := 0

	for current < len(pathNodes)-1 {
		// 尝试找到最远的可直达节点
		farthest := current
		for next := current + 1; next < len(pathNodes); next++ {
			if nq.isPathClear(pathNodes[current], pathNodes[next]) {
				farthest = next
			}
		}

		// 如果没有找到更远的节点，则只前进到下一个节点
		if farthest == current {
			farthest = current + 1
		}

		// 添加最远可达点
		smoothed = append(smoothed, pathNodes[farthest])
		current = farthest
	}

	return smoothed
}

// isPathClear 检查两点之间的路径是否畅通
func (nq *NavigationQuery) isPathClear(start, end math32.Vector3) bool {
	// 计算方向向量和距离
	direction := end.Sub(start)
	distance := direction.Length()

	if distance < 0.001 { // 距离太近，认为是同一点
		return true
	}

	// 标准化方向向量
	direction = direction.Scale(1.0 / distance)

	agentRadius := float32(0.4)
	if nq.agent != nil {
		agentRadius = nq.agent.Radius
	}
	// 使用适当的步长进行采样检查
	stepSize := math32.Max(0.1, agentRadius*0.6)
	steps := math32.CeilToInt(distance / stepSize)

	// 沿着路径进行采样检测
	for i := 0; i <= steps; i++ {
		t := float32(i) / float32(steps)
		samplePoint := start.Add(direction.Scale(distance * t))

		// 如果有Agent半径，还需要检查Agent碰撞
		occupied := nq.octree.IsAgentOccupied(nq.agent, samplePoint)
		// if Log {
		// 	fmt.Println(start, end, nq.agent, samplePoint, occupied)
		// }
		if occupied {
			return false
		}

	}

	return true
}

// convertToWorldPath 将节点路径转换为世界坐标路径，使用边界穿越点替代中心点
func (nq *NavigationQuery) getCrossingPath(nodePath []*octree.PathNode) []math32.Vector3 {
	if len(nodePath) == 0 {
		return []math32.Vector3{}
	}

	var path []math32.Vector3

	// 1. 添加起点
	path = append(path, nodePath[0].Center)

	// 2. 如果只有一个节点，直接连接起点到终点
	if len(nodePath) <= 3 {
		path = append(path, nodePath[1].Center)
		path = append(path, nodePath[2].Center)
		return path
	}

	// 3. 处理中间节点间的边界穿越点
	for i := 1; i < len(nodePath)-2; i++ {
		node1 := nodePath[i]
		node2 := nodePath[i+1]

		// 计算两个节点之间的边界穿越点
		crossingPoint := nq.calculateCrossingPoint(node1, node2)
		path = append(path, crossingPoint)
	}

	// 4. 添加终点
	path = append(path, nodePath[len(nodePath)-1].Center)

	return path
}

// calculateCrossingPoint 计算两个相邻节点之间的边界穿越点
func (nq *NavigationQuery) calculateCrossingPoint(node1, node2 *octree.PathNode) math32.Vector3 {
	// 确定共享的边界平面
	planeAxis := -1 // 0=X, 1=Y, 2=Z
	planePos := float32(0.0)

	if floatEqual(node1.Bounds.Max.X, node2.Bounds.Min.X) { // 检查x方向
		planeAxis = 0
		planePos = node1.Bounds.Max.X
	} else if floatEqual(node1.Bounds.Min.X, node2.Bounds.Max.X) {
		planeAxis = 0
		planePos = node1.Bounds.Min.X
	} else if floatEqual(node1.Bounds.Max.Y, node2.Bounds.Min.Y) { // 检查y方向
		planeAxis = 1
		planePos = node1.Bounds.Max.Y
	} else if floatEqual(node1.Bounds.Min.Y, node2.Bounds.Max.Y) {
		planeAxis = 1
		planePos = node1.Bounds.Min.Y
	} else if floatEqual(node1.Bounds.Max.Z, node2.Bounds.Min.Z) { // 检查z方向
		planeAxis = 2
		planePos = node1.Bounds.Max.Z
	} else if floatEqual(node1.Bounds.Min.Z, node2.Bounds.Max.Z) {
		planeAxis = 2
		planePos = node1.Bounds.Min.Z
	} else {
		// 如果没有找到共享平面，可能是错误，返回node1.Center作为默认
		fmt.Println("没有找到共享平面", node1.Center, node2.Center)
		return node1.Center
	}

	// 计算从node1.Center到node2.Center的射线与平面的交点
	dir := node2.Center.Sub(node1.Center)

	var t float32
	switch planeAxis {
	case 0: // X平面
		if math32.Abs(dir.X) < 1e-6 {
			// 避免除以零
			return node1.Center
		}
		t = (planePos - node1.Center.X) / dir.X
	case 1: // Y平面
		if math32.Abs(dir.Y) < 1e-6 {
			return node1.Center
		}
		t = (planePos - node1.Center.Y) / dir.Y
	case 2: // Z平面
		if math32.Abs(dir.Z) < 1e-6 {
			return node1.Center
		}
		t = (planePos - node1.Center.Z) / dir.Z
	}

	// 确保t在[0,1]范围内
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}

	// 计算交点
	point := node1.Center.Add(dir.Mul(t))

	// 检查交点是否在两个节点的实际共享区域内（处理不同大小节点的情况）
	switch planeAxis {
	case 0: // X平面，检查Y和Z范围
		minY := math32.Max(node1.Bounds.Min.Y, node2.Bounds.Min.Y)
		maxY := math32.Min(node1.Bounds.Max.Y, node2.Bounds.Max.Y)
		minZ := math32.Max(node1.Bounds.Min.Z, node2.Bounds.Min.Z)
		maxZ := math32.Min(node1.Bounds.Max.Z, node2.Bounds.Max.Z)

		if point.Y < minY || point.Y > maxY || point.Z < minZ || point.Z > maxZ {
			// 交点不在共享区域内，调整到最近的有效位置
			point.Y = clamp(point.Y, minY, maxY)
			point.Z = clamp(point.Z, minZ, maxZ)
		}
	case 1: // Y平面，检查X和Z范围
		minX := math32.Max(node1.Bounds.Min.X, node2.Bounds.Min.X)
		maxX := math32.Min(node1.Bounds.Max.X, node2.Bounds.Max.X)
		minZ := math32.Max(node1.Bounds.Min.Z, node2.Bounds.Min.Z)
		maxZ := math32.Min(node1.Bounds.Max.Z, node2.Bounds.Max.Z)

		if point.X < minX || point.X > maxX || point.Z < minZ || point.Z > maxZ {
			point.X = clamp(point.X, minX, maxX)
			point.Z = clamp(point.Z, minZ, maxZ)
		}
	case 2: // Z平面，检查X和Y范围
		minX := math32.Max(node1.Bounds.Min.X, node2.Bounds.Min.X)
		maxX := math32.Min(node1.Bounds.Max.X, node2.Bounds.Max.X)
		minY := math32.Max(node1.Bounds.Min.Y, node2.Bounds.Min.Y)
		maxY := math32.Min(node1.Bounds.Max.Y, node2.Bounds.Max.Y)

		if point.X < minX || point.X > maxX || point.Y < minY || point.Y > maxY {
			point.X = clamp(point.X, minX, maxX)
			point.Y = clamp(point.Y, minY, maxY)
		}
	}

	return point
}

// floatEqual 浮点数比较（考虑精度问题）
func floatEqual(a, b float32) bool {
	return math32.Abs(a-b) < 1e-6
}

// clamp 将值限制在[min, max]范围内
func clamp(value, min, max float32) float32 {
	if value < min {
		return min
	}
	if value > max {
		return max
	}
	return value
}
