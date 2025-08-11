package builder

import (
	"sort"

	"github.com/o0olele/octree-go/math32"
	"github.com/o0olele/octree-go/octree"
)

// FindClosestNodeBruteForce finds the closest node by brute force.
func (nd *NavigationData) FindClosestNodeBruteForce(pos math32.Vector3) int32 {
	bestNodeID := int32(-1)
	bestDistance := math32.MaxFloat32

	for i, node := range nd.Nodes {
		distance := pos.Distance(node.Center)
		if distance < bestDistance {
			bestDistance = distance
			bestNodeID = int32(i)
		}
	}

	return bestNodeID
}

// FindClosestNodeMorton finds the closest node by Morton code.
func (nd *NavigationData) FindClosestNodeMorton(pos math32.Vector3) int32 {
	// Check cache
	if nodeID, exists := nd.spatialCache.Get(pos.Hash()); exists {
		return nodeID
	}

	if len(nd.MortonIndex) == 0 {
		nodeId := nd.FindClosestNodeBruteForce(pos)
		nd.spatialCache.Put(pos.Hash(), nodeId)
		return nodeId
	}

	// Calculate the Morton code of the query position
	queryMorton := octree.Vector3ToMorton(pos, nd.Bounds, nd.MortonResolution)

	// Binary search in the Morton index
	targetIndex := sort.Search(len(nd.MortonIndex), func(i int) bool {
		nodeID := nd.MortonIndex[i]
		node := &nd.Nodes[nodeID]
		nodeMorton := octree.Vector3ToMorton(node.Center, nd.Bounds, nd.MortonResolution)
		return nodeMorton >= queryMorton
	})

	// Optimization: avoid duplicate calculations, use incremental search and global best tracking
	checkedNodes := make(map[int32]bool) // Record the nodes that have been checked
	globalBestNodeID := int32(-1)
	globalBestDistance := math32.MaxFloat32

	initialRadius := 8
	maxRadius := int(math32.Min(64, float32(len(nd.MortonIndex)/4))) // Maximum search range
	prevRadius := 0                                                  // The previous search radius

	defer func() {
		// Update the cache if a suitable node is found
		if globalBestNodeID != -1 {
			nd.spatialCache.Put(pos.Hash(), globalBestNodeID)
		}
	}()

	for searchRadius := initialRadius; searchRadius <= maxRadius; searchRadius *= 2 {
		newCandidates := make([]int32, 0, searchRadius)

		// Collect new nodes within the current search radius
		startIndex := targetIndex - searchRadius
		endIndex := targetIndex + searchRadius

		// Ensure the index range is valid
		if startIndex < 0 {
			startIndex = 0
		}
		if endIndex >= len(nd.MortonIndex) {
			endIndex = len(nd.MortonIndex) - 1
		}

		for i := startIndex; i <= endIndex; i++ {
			// Skip nodes within the previous search range (except for the first search)
			if prevRadius > 0 && i >= targetIndex-prevRadius && i <= targetIndex+prevRadius {
				continue
			}

			nodeID := nd.MortonIndex[i]
			if !checkedNodes[nodeID] {
				newCandidates = append(newCandidates, nodeID)
				checkedNodes[nodeID] = true
			}
		}

		// If there are no new nodes, continue to expand the search range
		if len(newCandidates) == 0 {
			prevRadius = searchRadius
			continue
		}

		// Update the global best node in the new candidate nodes
		for _, nodeID := range newCandidates {
			node := &nd.Nodes[nodeID]
			distance := pos.Distance(node.Center)

			// If there is no Agent, compare the distance directly
			if distance < globalBestDistance {
				globalBestDistance = distance
				globalBestNodeID = nodeID
			}
		}

		// If a suitable node is found, return directly
		if globalBestNodeID != -1 {
			return globalBestNodeID
		}

		prevRadius = searchRadius
	}

	return -1
}
