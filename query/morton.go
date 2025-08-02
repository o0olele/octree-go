package query

import (
	"sort"

	"github.com/o0olele/octree-go/math32"
	"github.com/o0olele/octree-go/octree"
)

// findClosestNodeMorton 使用Morton编码查找最近节点
func (nq *NavigationQuery) findClosestNodeMorton(pos math32.Vector3) int32 {
	if len(nq.navData.MortonIndex) == 0 {
		return nq.findClosestNodeBruteForce(pos)
	}

	// 计算查询位置的Morton编码
	queryMorton := octree.Vector3ToMorton(pos, nq.navData.Bounds, nq.navData.MortonResolution)

	// 在Morton索引中进行二分查找
	targetIndex := sort.Search(len(nq.navData.MortonIndex), func(i int) bool {
		nodeID := nq.navData.MortonIndex[i]
		node := &nq.navData.Nodes[nodeID]
		nodeMorton := octree.Vector3ToMorton(node.Center, nq.navData.Bounds, nq.navData.MortonResolution)
		return nodeMorton >= queryMorton
	})

	// 优化：避免重复计算，使用增量搜索和全局最佳跟踪
	checkedNodes := make(map[int32]bool) // 记录已检查的节点
	globalBestNodeID := int32(-1)
	globalBestDistance := math32.MaxFloat32

	initialRadius := 8
	maxRadius := int(math32.Min(64, float32(len(nq.navData.MortonIndex)/4))) // 最大搜索范围
	prevRadius := 0                                                          // 上一次的搜索半径

	for searchRadius := initialRadius; searchRadius <= maxRadius; searchRadius *= 2 {
		newCandidates := make([]int32, 0, searchRadius)

		// 收集当前搜索半径内的新节点
		startIndex := targetIndex - searchRadius
		endIndex := targetIndex + searchRadius

		// 确保索引范围有效
		if startIndex < 0 {
			startIndex = 0
		}
		if endIndex >= len(nq.navData.MortonIndex) {
			endIndex = len(nq.navData.MortonIndex) - 1
		}

		for i := startIndex; i <= endIndex; i++ {
			// 跳过之前搜索范围内的节点（除了第一次搜索）
			if prevRadius > 0 && i >= targetIndex-prevRadius && i <= targetIndex+prevRadius {
				continue
			}

			nodeID := nq.navData.MortonIndex[i]
			if !checkedNodes[nodeID] {
				newCandidates = append(newCandidates, nodeID)
				checkedNodes[nodeID] = true
			}
		}

		// 如果没有新节点，继续扩大搜索范围
		if len(newCandidates) == 0 {
			prevRadius = searchRadius
			continue
		}

		// 在新候选节点中更新全局最佳节点
		for _, nodeID := range newCandidates {
			node := &nq.navData.Nodes[nodeID]
			distance := pos.Distance(node.Center)

			// 没有Agent时，直接比较距离
			if distance < globalBestDistance {
				globalBestDistance = distance
				globalBestNodeID = nodeID
			}
		}

		// 如果找到了合适的节点，直接返回
		if globalBestNodeID != -1 {
			return globalBestNodeID
		}

		prevRadius = searchRadius
	}

	return -1
}

// findClosestNodeBruteForce 暴力查找最近节点
func (nq *NavigationQuery) findClosestNodeBruteForce(pos math32.Vector3) int32 {
	bestNodeID := int32(-1)
	bestDistance := math32.MaxFloat32

	for i, node := range nq.navData.Nodes {
		distance := pos.Distance(node.Center)
		if distance < bestDistance {
			bestDistance = distance
			bestNodeID = int32(i)
		}
	}

	return bestNodeID
}
