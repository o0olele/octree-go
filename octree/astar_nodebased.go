package octree

import (
	"container/heap"
	"fmt"
	"math"
	"runtime"
	"sync"
	"sync/atomic"
	"time"

	"github.com/o0olele/octree-go/geometry"
	"github.com/o0olele/octree-go/math32"
)

// NodeBasedAStarPathfinder is a node-based A* pathfinder.
type NodeBasedAStarPathfinder struct {
	octree   *Octree
	agent    *Agent
	stepSize float32
	graph    *PathGraph

	// Morton code optimization
	mortonSortedNodes []MortonNodePair
	mortonResolution  uint32
}

// MortonNodePair is a pair of Morton code and node.
type MortonNodePair struct {
	Morton MortonCode
	Node   *PathNode
}

// NewNodeBasedAStarPathfinder creates a node-based A* pathfinder.
func NewNodeBasedAStarPathfinder(octree *Octree, stepSize float32) *NodeBasedAStarPathfinder {
	return NewNodeBasedAStarPathfinderWithParallel(octree, nil, stepSize, false)
}

// NewNodeBasedAStarPathfinderWithParallel creates a node-based A* pathfinder, optionally using parallel construction.
func NewNodeBasedAStarPathfinderWithParallel(octree *Octree, agent *Agent, stepSize float32, useParallel bool) *NodeBasedAStarPathfinder {

	pathfinder := &NodeBasedAStarPathfinder{
		octree:           octree,
		agent:            agent,
		stepSize:         stepSize,
		graph:            NewPathGraph(),
		mortonResolution: 1024, // 10-bit precision
	}

	// Build the path graph
	pathfinder.buildGraphWithMode(useParallel)

	// Build the Morton index
	pathfinder.buildMortonIndex()

	return pathfinder
}

// GetGraph returns the path graph.
func (nba *NodeBasedAStarPathfinder) GetGraph() *PathGraph {
	return nba.graph
}

// BuildGraph builds the path graph.
func (nba *NodeBasedAStarPathfinder) BuildGraph() {
	nba.buildGraphWithMode(false) // Default to using the serial version
}

// BuildGraphParallel builds the path graph using the parallel version.
func (nba *NodeBasedAStarPathfinder) BuildGraphParallel() {
	nba.buildGraphWithMode(true) // Use the parallel version
}

// buildGraphWithMode builds the path graph, optionally using the serial or parallel version.
func (nba *NodeBasedAStarPathfinder) buildGraphWithMode(useParallel bool) {
	// Collect all empty leaves
	emptyLeaves := nba.collectEmptyLeaves(nba.octree.Root)

	// Create a path node for each empty leaf
	for _, leaf := range emptyLeaves {
		nba.graph.AddNode(leaf)
	}

	// Build connections between adjacent nodes
	beginTime := time.Now()
	if useParallel {
		nba.buildConnectionsParallel(emptyLeaves)
	} else {
		nba.buildConnections(emptyLeaves)
	}
	runtime.GC()
	runtime.Gosched()
	fmt.Printf("PathGraph build connections: %s\n", time.Since(beginTime))
}

// collectEmptyLeaves collects all empty leaves.
func (nba *NodeBasedAStarPathfinder) collectEmptyLeaves(node *OctreeNode) []*OctreeNode {
	var emptyLeaves []*OctreeNode

	if node.IsLeaf() {
		// Check if the leaf node is empty (not occupied)
		if !node.IsOccupied() {
			hit, _, _, _ := nba.octree.Raycast(node.Bounds.Center(), math32.Vector3{X: 0, Y: 1, Z: 0}, nba.agent.Height)
			if !hit {
				emptyLeaves = append(emptyLeaves, node)
			}
		}
		return emptyLeaves
	}

	// Recursively check child nodes
	for _, child := range node.Children {
		if child != nil {
			childLeaves := nba.collectEmptyLeaves(child)
			emptyLeaves = append(emptyLeaves, childLeaves...)
		}
	}

	return emptyLeaves
}

// buildConnections builds connections between nodes.
func (nba *NodeBasedAStarPathfinder) buildConnections(nodes []*OctreeNode) {
	totalConnections := 0
	totalChecks := 0

	for i, nodeA := range nodes {
		for j := i + 1; j < len(nodes); j++ {
			nodeB := nodes[j]
			totalChecks++

			// Check if the two nodes are adjacent (boundary intersects or touches)
			if nba.areNodesAdjacent(nodeA, nodeB) {
				pathNodeA := nba.graph.Nodes[nodeA]
				pathNodeB := nba.graph.Nodes[nodeB]
				nba.graph.AddEdge(pathNodeA, pathNodeB)
				totalConnections++
			}
		}
	}

	// Output debug information
	fmt.Printf("PathGraph build connections: checked %d pairs, built %d connections\n", totalChecks, totalConnections)
	fmt.Printf("Node count: %d, Edge count: %d\n", len(nba.graph.Nodes), len(nba.graph.Edges))
}

// buildConnectionsParallel builds connections between nodes using the parallel version.
func (nba *NodeBasedAStarPathfinder) buildConnectionsParallel(nodes []*OctreeNode) {
	totalNodes := len(nodes)

	if totalNodes <= 1 {
		fmt.Printf("PathGraph build connections: node count is less than 1, no connections need to be built\n")
		return
	}

	// Calculate the total number of node pairs
	totalPairs := (totalNodes * (totalNodes - 1)) / 2

	// Smart decision: if the number of node pairs is too few, use the serial version
	const PARALLEL_THRESHOLD = 1000 // Use serial version if the number of node pairs is less than 1000
	if totalPairs < PARALLEL_THRESHOLD {
		fmt.Printf("PathGraph build connections: node pair count (%d) is less than 1000, use serial version\n", totalPairs)
		nba.buildConnections(nodes)
		return
	}

	numWorkers := runtime.NumCPU()

	// For large datasets, limit the number of workers to reduce competition
	if totalPairs < 10000 {
		numWorkers = min(numWorkers, 4) // Maximum 4 workers
	}

	fmt.Printf("PathGraph build connections: node pair count (%d) is more than 10000, use parallel version (workers: %d)\n", totalPairs, numWorkers)

	// Use batch processing to reduce atomic operations
	batchSize := max(totalPairs/(numWorkers*4), 50) // At least 50 per batch

	totalConnections := int64(0)
	totalChecks := int64(0)

	// Use a slice to store results, avoiding dynamic allocation
	type EdgeResult struct {
		pathNodeA *PathNode
		pathNodeB *PathNode
	}

	// Each worker has its own result slice
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
				// Batch get work, reduce atomic operation frequency
				startIdx := atomic.AddInt64(&currentPair, int64(batchSize)) - int64(batchSize)
				endIdx := startIdx + int64(batchSize)
				if endIdx > int64(totalPairs) {
					endIdx = int64(totalPairs)
				}

				if startIdx >= int64(totalPairs) {
					break
				}

				// Process this batch
				for pairIdx := startIdx; pairIdx < endIdx; pairIdx++ {
					// Convert linear index to (i,j)
					i, j := nba.pairIndexToIJ(int(pairIdx), totalNodes)
					if i >= j || i >= totalNodes || j >= totalNodes {
						continue
					}

					nodeA := nodes[i]
					nodeB := nodes[j]
					localChecks++

					// Check if the two nodes are adjacent
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

			// Update the global counter
			atomic.AddInt64(&totalConnections, localConnections)
			atomic.AddInt64(&totalChecks, localChecks)
		}(workerID)
	}

	// Wait for all workers to complete
	wg.Wait()

	// Add all edges in the main thread
	finalConnections := 0
	for _, results := range workerResults {
		for _, edge := range results {
			nba.graph.AddEdge(edge.pathNodeA, edge.pathNodeB)
			finalConnections++
		}
	}

	// Output debug information
	fmt.Printf("PathGraph build connections: checked %d pairs, built %d connections (using %d workers)\n",
		totalChecks, totalConnections, numWorkers)
	fmt.Printf("Node count: %d, Edge count: %d\n", len(nba.graph.Nodes), len(nba.graph.Edges))
}

// pairIndexToIJ converts a linear pairing index to (i,j) coordinates
func (nba *NodeBasedAStarPathfinder) pairIndexToIJ(pairIndex, n int) (int, int) {
	// Use the mathematical formula directly
	i := int((2*float64(n) - 1 - math.Sqrt(float64((2*n-1)*(2*n-1)-8*pairIndex))) / 2)
	j := pairIndex - i*(2*n-i-1)/2 + i + 1
	return i, j
}

// min returns the smaller of two ints
func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}

// max returns the larger of two ints
func max(a, b int) int {
	if a > b {
		return a
	}
	return b
}

// areNodesAdjacent checks if two nodes are adjacent and connected
func (nba *NodeBasedAStarPathfinder) areNodesAdjacent(nodeA, nodeB *OctreeNode) bool {
	// Check if the boundaries intersect or touch
	boundsA := nodeA.Bounds
	boundsB := nodeB.Bounds

	// Calculate the distance between the centers of the two nodes
	centerA := boundsA.Center()
	centerB := boundsB.Center()
	distance := centerA.Distance(centerB)

	// Calculate the maximum size of the two nodes
	sizeA := boundsA.Size()
	sizeB := boundsB.Size()
	maxSizeA := math32.Max(math32.Max(sizeA.X, sizeA.Y), sizeA.Z)
	maxSizeB := math32.Max(math32.Max(sizeB.X, sizeB.Y), sizeB.Z)

	// Relax the distance threshold check - allow more adjacent node connections
	threshold := (maxSizeA + maxSizeB) * 0.6 // Relax the threshold from 0.6 to 0.8
	if distance > threshold {
		return false
	}

	// More relaxed boundary check
	if !boundsA.Intersects(boundsB) {
		// Calculate the minimum distance between the two bounding boxes
		minDistance := nba.calculateAABBDistance(boundsA, boundsB)
		// Allow small gaps, especially for nodes of different sizes
		allowedGap := math32.Min(maxSizeA, maxSizeB) * 0.2 // Increase the allowed gap
		if minDistance > allowedGap {
			return false
		}
	}

	// Path clarity check - use more relaxed sampling
	if !nba.isPathClearRelaxed(centerA, centerB) {
		return false
	}

	return true
}

// calculateAABBDistance calculates the minimum distance between two AABBs
func (nba *NodeBasedAStarPathfinder) calculateAABBDistance(aabb1, aabb2 geometry.AABB) float32 {
	// If they intersect, the distance is 0
	if aabb1.Intersects(aabb2) {
		return 0.0
	}

	// Calculate the distance on each axis
	dx := math32.Max(0, math32.Max(aabb1.Min.X-aabb2.Max.X, aabb2.Min.X-aabb1.Max.X))
	dy := math32.Max(0, math32.Max(aabb1.Min.Y-aabb2.Max.Y, aabb2.Min.Y-aabb1.Max.Y))
	dz := math32.Max(0, math32.Max(aabb1.Min.Z-aabb2.Max.Z, aabb2.Min.Z-aabb1.Max.Z))

	return math32.Sqrt(dx*dx + dy*dy + dz*dz)
}

// isPathClearRelaxed uses more relaxed parameters to check if the path is clear
func (nba *NodeBasedAStarPathfinder) isPathClearRelaxed(start, end math32.Vector3) bool {
	// Calculate the direction vector and distance
	direction := end.Sub(start)
	distance := direction.Length()

	if distance < 0.001 { // The distance is too close, considered the same point
		return true
	}

	// Normalize the direction vector
	direction = direction.Scale(1.0 / distance)

	if hit, _, _, _ := nba.octree.Raycast(start, direction, distance); hit {
		return false
	}

	// Use fewer sampling points, improve connectivity
	stepSize := math32.Min(nba.stepSize*0.5, 0.2) // Increase the step size, reduce sampling density
	steps := math32.CeilToInt(distance / stepSize)

	// Ensure there are a reasonable number of sampling points
	if steps < 5 {
		steps = 5
	}
	if steps > 20 { // Limit the maximum number of sampling points
		steps = 20
	}

	// Sample along the path for detection
	for i := 0; i <= steps; i++ {
		t := float32(i) / float32(steps)
		samplePoint := start.Add(direction.Scale(distance * t))

		// Check if the sampling point is occupied
		if nba.octree.IsOccupied(samplePoint) {
			return false
		}

		// If there is an Agent, check for Agent collisions, but use a slightly smaller Agent
		if nba.agent != nil {
			// // Create a slightly smaller Agent for detection, improve connectivity
			// relaxedAgent := &Agent{
			// 	Radius: nba.agent.Radius, // 减小10%
			// 	Height: nba.agent.Height, // 减小10%
			// }
			if nba.octree.IsAgentOccupied(nba.agent, samplePoint) {
				return false
			}
			if hit, _, _, _ := nba.octree.Raycast(samplePoint, math32.Vector3{X: 0, Y: 1, Z: 0}, nba.agent.Height); hit {
				return false
			}
		}
	}

	return true
}

// GetNodeMarginPoints gets the sampling points of the node boundary
func (nba *NodeBasedAStarPathfinder) GetNodeMarginPoints(bounds geometry.AABB) []math32.Vector3 {
	center := bounds.Center()

	// Generate sampling points within the node boundary (avoid points on the boundary)
	margin := float32(0.1) // Shrink the boundary by a small amount
	points := []math32.Vector3{
		// Center point
		center,
		// 8 corner points
		{X: bounds.Min.X + margin, Y: bounds.Min.Y + margin, Z: bounds.Min.Z + margin},
		{X: bounds.Max.X - margin, Y: bounds.Min.Y + margin, Z: bounds.Min.Z + margin},
		{X: bounds.Min.X + margin, Y: bounds.Max.Y - margin, Z: bounds.Min.Z + margin},
		{X: bounds.Max.X - margin, Y: bounds.Max.Y - margin, Z: bounds.Min.Z + margin},
		{X: bounds.Min.X + margin, Y: bounds.Min.Y + margin, Z: bounds.Max.Z - margin},
		{X: bounds.Max.X - margin, Y: bounds.Min.Y + margin, Z: bounds.Max.Z - margin},
		{X: bounds.Min.X + margin, Y: bounds.Max.Y - margin, Z: bounds.Max.Z - margin},
		{X: bounds.Max.X - margin, Y: bounds.Max.Y - margin, Z: bounds.Max.Z - margin},
		// 6 face centers
		{X: center.X, Y: center.Y, Z: bounds.Min.Z + margin},
		{X: center.X, Y: center.Y, Z: bounds.Max.Z - margin},
		{X: center.X, Y: bounds.Min.Y + margin, Z: center.Z},
		{X: center.X, Y: bounds.Max.Y - margin, Z: center.Z},
		{X: bounds.Min.X + margin, Y: center.Y, Z: center.Z},
		{X: bounds.Max.X - margin, Y: center.Y, Z: center.Z},
	}

	return points
}

// SetAgent sets the pathfinding Agent
func (nba *NodeBasedAStarPathfinder) SetAgent(agent *Agent) {
	nba.agent = agent
}

// GetAgent gets the current Agent
func (nba *NodeBasedAStarPathfinder) GetAgent() *Agent {
	return nba.agent
}

// GetStepSize gets the step size
func (nba *NodeBasedAStarPathfinder) GetStepSize() float32 {
	return nba.stepSize
}

// SetStepSize sets the step size
func (nba *NodeBasedAStarPathfinder) SetStepSize(stepSize float32) {
	nba.stepSize = stepSize
}

// ToGridCoord converts a Vector3 to grid coordinates (compatible interface)
func (nba *NodeBasedAStarPathfinder) ToGridCoord(pos math32.Vector3) (int, int, int) {
	origin := nba.octree.Root.Bounds.Min
	return math32.RoundToInt((pos.X - origin.X) / nba.stepSize),
		math32.RoundToInt((pos.Y - origin.Y) / nba.stepSize),
		math32.RoundToInt((pos.Z - origin.Z) / nba.stepSize)
}

// FindPath uses the node-based A* algorithm to find a path
func (nba *NodeBasedAStarPathfinder) FindPath(start, end math32.Vector3) []math32.Vector3 {
	// Find the closest empty node to the start and end points
	startTime := time.Now()
	startNode := nba.findClosestNode(start)
	endNode := nba.findClosestNode(end)
	fmt.Printf("FindPath time: %v\n", time.Since(startTime))

	if startNode == nil || endNode == nil {
		return nil
	}

	// If the start and end points are in the same node
	if startNode == endNode {
		return []math32.Vector3{start, end}
	}

	// Run the A* algorithm
	startTime = time.Now()
	path := nba.astar(startNode, endNode)
	fmt.Printf("Astar time: %v\n", time.Since(startTime))
	if path == nil {
		return nil
	}

	startTime = time.Now()
	// Convert the node path to a world coordinate path
	worldPath := nba.convertToWorldPath(path, start, end)
	fmt.Printf("ConvertToWorldPath time: %v\n", time.Since(startTime))
	return worldPath
}

// findClosestNode finds the closest empty node to the given position
func (nba *NodeBasedAStarPathfinder) findClosestNode(pos math32.Vector3) *PathNode {
	// First try to find the node containing the position directly
	containingNode := nba.findContainingNode(pos)
	if containingNode != nil {
		return containingNode
	}

	// Use Morton code optimized spatial query
	if len(nba.mortonSortedNodes) > 0 {
		return nba.findClosestNodeMorton(pos)
	}

	// Fall back to the original method
	return nba.findClosestNodeSpatial(pos)
}

// findContainingNode finds the node containing the given position
func (nba *NodeBasedAStarPathfinder) findContainingNode(pos math32.Vector3) *PathNode {
	// Start searching from the root node
	octreeNode := nba.findContainingOctreeNode(nba.octree.Root, pos)
	if octreeNode != nil {
		if pathNode, exists := nba.graph.Nodes[octreeNode]; exists {
			// If there is an Agent, check if the Agent can be placed at this position
			if nba.agent != nil && nba.octree.IsAgentOccupied(nba.agent, pos) {
				return nil
			}
			return pathNode
		}
	}
	return nil
}

// findContainingOctreeNode finds the leaf node containing the position in the octree
func (nba *NodeBasedAStarPathfinder) findContainingOctreeNode(node *OctreeNode, pos math32.Vector3) *OctreeNode {
	if node == nil || !node.Bounds.Contains(pos) {
		return nil
	}

	if node.IsLeaf() {
		if !node.IsOccupied() {
			return node
		}
		return nil
	}

	// Recursively search child nodes
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

// findClosestNodeSpatial uses spatial query to find the closest node
func (nba *NodeBasedAStarPathfinder) findClosestNodeSpatial(pos math32.Vector3) *PathNode {
	var closestNode *PathNode
	minDistance := math32.MaxFloat32

	// Use breadth-first search, starting from the most likely area
	candidates := nba.getSpatialCandidates(pos, 10) // Get the 10 closest candidate nodes

	for _, node := range candidates {
		distance := pos.Distance(node.Center)
		if distance < minDistance {
			// If there is an Agent, check if the Agent can be placed at the node center
			if nba.agent != nil && nba.octree.IsAgentOccupied(nba.agent, node.Center) {
				continue
			}
			minDistance = distance
			closestNode = node
		}
	}

	// If no suitable candidate is found, fall back to global search
	if closestNode == nil {
		return nba.findClosestNodeBruteForce(pos)
	}

	return closestNode
}

// getSpatialCandidates gets the closest candidate nodes in space
func (nba *NodeBasedAStarPathfinder) getSpatialCandidates(pos math32.Vector3, maxCandidates int) []*PathNode {
	type nodeDistance struct {
		node     *PathNode
		distance float32
	}

	candidates := make([]nodeDistance, 0)

	// Collect distance information for all nodes
	for _, node := range nba.graph.Nodes {
		distance := pos.Distance(node.Center)
		candidates = append(candidates, nodeDistance{node, distance})
	}

	// Sort by distance
	for i := 0; i < len(candidates)-1; i++ {
		for j := i + 1; j < len(candidates); j++ {
			if candidates[i].distance > candidates[j].distance {
				candidates[i], candidates[j] = candidates[j], candidates[i]
			}
		}
	}

	// Return the closest candidate nodes
	result := make([]*PathNode, 0, maxCandidates)
	for i := 0; i < len(candidates) && i < maxCandidates; i++ {
		result = append(result, candidates[i].node)
	}

	return result
}

// findClosestNodeBruteForce brute force search for the closest node (backup method)
func (nba *NodeBasedAStarPathfinder) findClosestNodeBruteForce(pos math32.Vector3) *PathNode {
	var closestNode *PathNode
	minDistance := math32.MaxFloat32

	for _, node := range nba.graph.Nodes {
		// Check if the position is within the node boundary
		if node.Bounds.Contains(pos) {
			// If there is an Agent, check if the Agent can be placed at this position
			if nba.agent != nil && nba.octree.IsAgentOccupied(nba.agent, pos) {
				continue
			}
			return node
		}

		// Calculate the distance to the node center
		distance := pos.Distance(node.Center)
		if distance < minDistance {
			// If there is an Agent, check if the Agent can be placed at the node center
			if nba.agent != nil && nba.octree.IsAgentOccupied(nba.agent, node.Center) {
				continue
			}
			minDistance = distance
			closestNode = node
		}
	}

	return closestNode
}

// astar runs the A* algorithm
func (nba *NodeBasedAStarPathfinder) astar(startNode, endNode *PathNode) []*PathNode {
	openHeap := &PathNodeHeap{}
	heap.Init(openHeap)
	closedSet := make(map[int32]bool)

	// Reset all A* data for all nodes
	for _, node := range nba.graph.Nodes {
		node.GCost = math32.MaxFloat32
		node.HCost = 0
		node.FCost = math32.MaxFloat32
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

		// Check all adjacent nodes
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

	return nil // No path found
}

// heuristic heuristic function
func (nba *NodeBasedAStarPathfinder) heuristic(a, b *PathNode) float32 {
	return a.Center.Distance(b.Center)
}

// reconstructNodePath reconstructs the node path
func (nba *NodeBasedAStarPathfinder) reconstructNodePath(node *PathNode) []*PathNode {
	var path []*PathNode
	current := node

	for current != nil {
		path = append([]*PathNode{current}, path...)
		current = current.Parent
	}

	return path
}

// convertToWorldPath converts the node path to a world coordinate path
func (nba *NodeBasedAStarPathfinder) convertToWorldPath(nodePath []*PathNode, start, end math32.Vector3) []math32.Vector3 {
	if len(nodePath) == 0 {
		return nil
	}

	startTime := time.Now()

	smoothedPath := nba.SmoothPath(nodePath)

	// If the funnel algorithm fails, fall back to the traditional method
	if len(smoothedPath) == 0 {
		path := []math32.Vector3{start}

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
		smoothedPath = path
	} else {
		// Ensure the start and end points are correct
		if len(smoothedPath) > 0 {
			smoothedPath[0] = start
			smoothedPath[len(smoothedPath)-1] = end
		}
	}

	// Post-processing: mild smoothing and validation
	validated := nba.validatePathSafety(smoothedPath)

	fmt.Printf("ConvertToWorldPath time: %v (funnel algorithm)\n", time.Since(startTime))

	return validated
}

// SmoothPath smooths the path
func (nba *NodeBasedAStarPathfinder) SmoothPath(pathNodes []*PathNode) []math32.Vector3 {

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
		// Try to find the farthest reachable node
		farthest := current
		for next := current + 1; next < len(pathNodes); next++ {
			if nba.isPathClear(pathNodes[current].Center, pathNodes[next].Center) {
				farthest = next
			}
		}

		// If no farther node is found, only move to the next node
		if farthest == current {
			farthest = current + 1
		}

		// Add the farthest reachable point
		smoothed = append(smoothed, pathNodes[farthest].Center)
		current = farthest
	}

	return smoothed
}

// isPathClear checks if the path between two points is clear
func (nba *NodeBasedAStarPathfinder) isPathClear(start, end math32.Vector3) bool {
	// Calculate the direction vector and distance
	direction := end.Sub(start)
	distance := direction.Length()

	if distance < 0.001 { // The distance is too close, considered the same point
		return true
	}

	// Normalize the direction vector
	direction = direction.Scale(1.0 / distance)

	agentRadius := float32(0.4)
	if nba.agent != nil {
		agentRadius = nba.agent.Radius
	}
	// Use an appropriate step size for sampling checks
	stepSize := math32.Max(0.1, agentRadius*0.6)
	steps := math32.CeilToInt(distance / stepSize)

	// 沿着路径进行采样检测
	for i := 0; i <= steps; i++ {
		t := float32(i) / float32(steps)
		samplePoint := start.Add(direction.Scale(distance * t))

		// If there is an Agent radius, check for Agent collisions
		occupied := nba.octree.IsAgentOccupied(nba.agent, samplePoint)
		if occupied {
			return false
		}

	}

	return true
}

// findSafePointInNode finds the safe position for the Agent in the node
func (nba *NodeBasedAStarPathfinder) findSafePointInNode(node *PathNode) *math32.Vector3 {
	if nba.agent == nil {
		return &node.Center
	}

	// Sample multiple points within the node boundary
	bounds := node.Bounds
	size := bounds.Size()

	// Shrink the search range to avoid boundary collisions
	margin := math32.Min(math32.Min(size.X, size.Y), size.Z) * 0.1
	searchBounds := geometry.AABB{
		Min: bounds.Min.Add(math32.Vector3{X: margin, Y: margin, Z: margin}),
		Max: bounds.Max.Sub(math32.Vector3{X: margin, Y: margin, Z: margin}),
	}

	// Sample in the search area
	samples := 8
	for i := 0; i < samples; i++ {
		for j := 0; j < samples; j++ {
			for k := 0; k < samples; k++ {
				t := math32.Vector3{
					X: float32(i) / float32(samples-1),
					Y: float32(j) / float32(samples-1),
					Z: float32(k) / float32(samples-1),
				}

				testPoint := math32.Vector3{
					X: searchBounds.Min.X + t.X*(searchBounds.Max.X-searchBounds.Min.X),
					Y: searchBounds.Min.Y + t.Y*(searchBounds.Max.Y-searchBounds.Min.Y),
					Z: searchBounds.Min.Z + t.Z*(searchBounds.Max.Z-searchBounds.Min.Z),
				}

				if !nba.octree.IsAgentOccupied(nba.agent, testPoint) {
					return &testPoint
				}
			}
		}
	}

	// If no safe point is found, return the center point
	return &node.Center
}

// validatePathSafety validates the path safety
func (nba *NodeBasedAStarPathfinder) validatePathSafety(path []math32.Vector3) []math32.Vector3 {
	if len(path) <= 1 {
		return path
	}

	validatedPath := []math32.Vector3{path[0]}

	for i := 1; i < len(path); i++ {
		lastPoint := validatedPath[len(validatedPath)-1]
		currentPoint := path[i]

		// Check if the path from the last valid point to the current point is safe
		if nba.isPathClear(lastPoint, currentPoint) {
			validatedPath = append(validatedPath, currentPoint)
		} else {
			// If not safe, use a conservative strategy: keep the original path structure
			// This ensures we don't introduce wall-through problems due to smoothing
			validatedPath = append(validatedPath, currentPoint)
		}
	}

	return validatedPath
}

// findSafeMidpoint finds the safe midpoint between two points
func (nba *NodeBasedAStarPathfinder) FindSafeMidpoint(start, end math32.Vector3) *math32.Vector3 {
	// Simple midpoint strategy
	mid := math32.Vector3{
		X: (start.X + end.X) / 2,
		Y: (start.Y + end.Y) / 2,
		Z: (start.Z + end.Z) / 2,
	}

	// Check if the midpoint is safe
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

// hasObstacleBetween checks if there is an obstacle between two points
func (nba *NodeBasedAStarPathfinder) HasObstacleBetween(start, end math32.Vector3) bool {
	// Use the reverse result of isPathClear, maintain consistency
	return !nba.isPathClear(start, end)
}

// GetPathGraph gets the path graph data, for visualization
func (nba *NodeBasedAStarPathfinder) GetPathGraph() *PathGraph {
	return nba.graph
}

// PathGraphData is the path graph data structure for JSON serialization
type PathGraphData struct {
	Nodes []PathNodeData `json:"nodes"`
	Edges []PathEdgeData `json:"edges"`
}

// PathNodeData is the path node data structure for JSON serialization
type PathNodeData struct {
	ID     int32          `json:"id"`
	Center math32.Vector3 `json:"center"`
	Bounds geometry.AABB  `json:"bounds"`
}

// PathEdgeData is the path edge data structure for JSON serialization
type PathEdgeData struct {
	NodeAID int32   `json:"node_a_id"`
	NodeBID int32   `json:"node_b_id"`
	Cost    float32 `json:"cost"`
}

// ToPathGraphData converts the PathGraph to a serializable data structure
func (nba *NodeBasedAStarPathfinder) ToPathGraphData() *PathGraphData {
	if nba.graph == nil {
		return &PathGraphData{
			Nodes: []PathNodeData{},
			Edges: []PathEdgeData{},
		}
	}

	// Convert node data
	nodes := make([]PathNodeData, 0, len(nba.graph.Nodes))
	for _, node := range nba.graph.Nodes {
		nodes = append(nodes, PathNodeData{
			ID:     node.ID,
			Center: node.Center,
			Bounds: node.Bounds,
		})
	}

	// Convert edge data
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

// buildMortonIndex builds the spatial index based on Morton coding
func (nba *NodeBasedAStarPathfinder) buildMortonIndex() {
	// Calculate Morton codes for all nodes
	nba.mortonSortedNodes = make([]MortonNodePair, 0, len(nba.graph.Nodes))

	for _, node := range nba.graph.Nodes {
		morton := Vector3ToMorton(node.Center, nba.octree.Root.Bounds, nba.mortonResolution)
		nba.mortonSortedNodes = append(nba.mortonSortedNodes, MortonNodePair{
			Morton: morton,
			Node:   node,
		})
	}

	// Sort nodes by Morton code
	nba.sortNodesByMorton()
}

// sortNodesByMorton sorts nodes by Morton code
func (nba *NodeBasedAStarPathfinder) sortNodesByMorton() {
	// Use quick sort to sort nodes by Morton code
	for i := 0; i < len(nba.mortonSortedNodes)-1; i++ {
		for j := i + 1; j < len(nba.mortonSortedNodes); j++ {
			if nba.mortonSortedNodes[i].Morton > nba.mortonSortedNodes[j].Morton {
				nba.mortonSortedNodes[i], nba.mortonSortedNodes[j] = nba.mortonSortedNodes[j], nba.mortonSortedNodes[i]
			}
		}
	}
}

// findClosestNodeMorton uses Morton code to quickly find the closest node
func (nba *NodeBasedAStarPathfinder) findClosestNodeMorton(pos math32.Vector3) *PathNode {
	if len(nba.mortonSortedNodes) == 0 {
		return nil
	}

	// Calculate the Morton code for the target position
	targetMorton := Vector3ToMorton(pos, nba.octree.Root.Bounds, nba.mortonResolution)

	// Binary search for the closest Morton code
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

	// Check the previous and next few candidate nodes, find the actual distance closest
	candidates := nba.getMortonCandidates(bestIndex, 10)

	var closestNode *PathNode
	minDistance := math32.MaxFloat32

	for _, candidate := range candidates {
		distance := pos.Distance(candidate.Center)
		if distance < minDistance {
			// If there is an Agent, check if the Agent can be placed at the node center
			if nba.agent != nil && nba.octree.IsAgentOccupied(nba.agent, candidate.Center) {
				continue
			}
			minDistance = distance
			closestNode = candidate
		}
	}

	return closestNode
}

// getMortonCandidates gets the candidate nodes near the Morton index
func (nba *NodeBasedAStarPathfinder) getMortonCandidates(centerIndex, count int) []*PathNode {
	candidates := make([]*PathNode, 0, count)

	// Expand from the center to both sides
	for i := 0; i < count && (centerIndex-i >= 0 || centerIndex+i < len(nba.mortonSortedNodes)); i++ {
		// Expand to the left
		if centerIndex-i >= 0 {
			candidates = append(candidates, nba.mortonSortedNodes[centerIndex-i].Node)
		}
		// Expand to the right (avoid adding the center node twice)
		if i > 0 && centerIndex+i < len(nba.mortonSortedNodes) {
			candidates = append(candidates, nba.mortonSortedNodes[centerIndex+i].Node)
		}
	}

	return candidates
}
