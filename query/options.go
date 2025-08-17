package query

import "github.com/o0olele/octree-go/builder"

// PathPreferences is the path preference configuration
type PathPreferences struct {
	InteriorPathBonus    float32 // The interior path bonus (the ratio of the cost reduction)
	BoundaryPathPenalty  float32 // The boundary path penalty (the ratio of the cost increase)
	DensityBonus         float32 // The density bonus
	BoundaryThreshold    float32 // The boundary threshold (the multiple of stepSize)
	EnablePathPreference bool    // Whether to enable path preference
	UsingCrossPoint      bool
	UsingDouglasPeucker  bool
}

// DefaultPathPreferences returns the default path preference configuration
func DefaultPathPreferences() *PathPreferences {
	return &PathPreferences{
		InteriorPathBonus:    0.2,  // The interior path cost is reduced by 20%
		BoundaryPathPenalty:  0.5,  // The boundary path cost is increased by 50%
		DensityBonus:         0.2,  // The density bonus
		BoundaryThreshold:    3.0,  // The boundary threshold is 3 times the stepSize
		EnablePathPreference: true, // The default path preference is enabled
		UsingCrossPoint:      false,
		UsingDouglasPeucker:  true,
	}
}

func (nq *NavigationQuery) GetPathPreferences() *PathPreferences {
	return nq.pathPreferences
}

// calculateEnhancedMovementCost calculates the enhanced movement cost, considering path preference
func (nq *NavigationQuery) calculateEnhancedMovementCost(currentNodeID, neighborNodeID int32) float32 {
	currentNode := &nq.navData.Nodes[currentNodeID]
	neighborNode := &nq.navData.Nodes[neighborNodeID]

	// The base distance cost
	baseCost := currentNode.Center.Distance(neighborNode.Center)

	// If the path preference is not enabled, return the base cost
	if !nq.pathPreferences.EnablePathPreference {
		return baseCost
	}

	// Check if the node is a boundary node (near the scene boundary) - using precomputed cache
	currentIsBoundary := nq.isNodeNearBoundary(currentNode)
	neighborIsBoundary := nq.isNodeNearBoundary(neighborNode)

	// Apply the path preference weight
	var costMultiplier float32 = 1.0

	if currentIsBoundary && neighborIsBoundary {
		// Both nodes are near the boundary, increase the cost to lower the priority
		costMultiplier = 1.0 + nq.pathPreferences.BoundaryPathPenalty
	} else if currentIsBoundary || neighborIsBoundary {
		// Only one node is near the boundary, slightly increase the cost
		costMultiplier = 1.0 + (nq.pathPreferences.BoundaryPathPenalty * 0.5)
	} else {
		// Both nodes are not near the boundary, give the interior path bonus
		costMultiplier = 1.0 - nq.pathPreferences.InteriorPathBonus
	}

	// Check the node density, the cost is lower in high-density areas (such as inside rooms)
	densityFactor := nq.calculateNodeDensityFactor(currentNodeID, neighborNodeID)
	costMultiplier *= densityFactor

	return baseCost * costMultiplier
}

// isNodeNearBoundary checks if the node is near the scene boundary
func (nq *NavigationQuery) isNodeNearBoundary(node *builder.CompactNode) bool {
	// Use the precomputed node-to-boundary minimum distance to determine
	// The threshold still comes from the configuration: BoundaryThreshold * stepSize
	threshold := nq.stepSize * nq.pathPreferences.BoundaryThreshold
	dist := nq.navData.GetNodeBoundaryDistance(node.ID)
	return dist < threshold
}

// calculateNodeDensityFactor calculates the node density factor
func (nq *NavigationQuery) calculateNodeDensityFactor(currentNodeID, neighborNodeID int32) float32 {
	// Use the precomputed adjacency degree instead of real-time query
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
