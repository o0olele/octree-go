package query

import (
	"container/heap"
	"fmt"
	"time"

	"github.com/o0olele/octree-go/builder"
	"github.com/o0olele/octree-go/math32"
	"github.com/o0olele/octree-go/octree"
)

// NavigationQuery is the navigation query, similar to Detour, only responsible for runtime queries
type NavigationQuery struct {
	navData         *builder.NavigationData // the navigation data
	agent           *octree.Agent           // the agent
	stepSize        float32                 // the step size
	pathPreferences *PathPreferences        // path preferences configuration
}

// NewNavigationQuery creates a new navigation query
func NewNavigationQuery(navData *builder.NavigationData) (*NavigationQuery, error) {
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

// GetOctree gets the octree
func (nq *NavigationQuery) GetOctree() *octree.Octree {
	return nq.navData.GetOctree()
}

// SetAgent sets the agent
func (nq *NavigationQuery) SetAgent(agent *octree.Agent) {
	nq.agent = agent
}

// GetAgent gets the agent
func (nq *NavigationQuery) GetAgent() *octree.Agent {

	return nq.agent
}

// SetStepSize sets the step size
func (nq *NavigationQuery) SetStepSize(stepSize float32) {
	nq.stepSize = stepSize
}

// GetStepSize gets the step size
func (nq *NavigationQuery) GetStepSize() float32 {
	return nq.stepSize
}

// GetNavigationData gets the navigation data
func (nq *NavigationQuery) GetNavigationData() *builder.NavigationData {
	return nq.navData
}

// FindPath finds the path
func (nq *NavigationQuery) FindPath(start, end math32.Vector3) []math32.Vector3 {
	// 1. find the closest nodes to the start and end points
	startNodeID := nq.FindClosestNode(start)
	endNodeID := nq.FindClosestNode(end)

	if startNodeID == -1 || endNodeID == -1 {
		return nil
	}

	// 2. use the bidirectional A* algorithm to find the path
	startTime := time.Now()
	nodePath := nq.AstarBidirectional(startNodeID, endNodeID)
	if nodePath != nil {
		fmt.Printf("Bi-directional A* algorithm took %v\n", time.Since(startTime))
	} else {
		// fall back to the single-direction A* algorithm
		startTime = time.Now()
		nodePath = nq.Astar(startNodeID, endNodeID)
		if nodePath == nil {
			return nil
		}
		fmt.Printf("A* algorithm took %v\n", time.Since(startTime))
	}

	// 3. convert to world coordinate path
	path := nq.ConvertToWorldPath(nodePath, start, end)
	fmt.Printf("Path length: %d, took %v\n", len(path), time.Since(startTime))
	return path
}

// FindClosestNode finds the closest node
func (nq *NavigationQuery) FindClosestNode(pos math32.Vector3) int32 {
	return nq.navData.FindClosestNodeMorton(pos)
}

// Astar is the A* algorithm
func (nq *NavigationQuery) Astar(startNodeID, endNodeID int32) []int32 {
	if startNodeID == endNodeID {
		return []int32{startNodeID}
	}

	// initialize the open and closed lists
	openSet := &nodeHeap{}
	heap.Init(openSet)

	closedSet := make(map[int32]bool)
	gScore := make(map[int32]float32)
	fScore := make(map[int32]float32)
	cameFrom := make(map[int32]int32)
	inOpenSet := make(map[int32]*heapNode) // optimization: use map to quickly find nodes in the open list

	// start point
	startNode := &nq.navData.Nodes[startNodeID]
	endNode := &nq.navData.Nodes[endNodeID]

	gScore[startNodeID] = 0
	fScore[startNodeID] = nq.heuristic(startNode.Center, endNode.Center)

	startHeapNode := newHeapNode(startNodeID, fScore[startNodeID])
	heap.Push(openSet, startHeapNode)
	inOpenSet[startNodeID] = startHeapNode

	// add the maximum iteration limit to prevent infinite loop
	maxIterations := 20000
	iterations := 0

	for openSet.Len() > 0 && iterations < maxIterations {
		iterations++

		current := heap.Pop(openSet).(*heapNode)
		currentNodeID := current.nodeID
		delete(inOpenSet, currentNodeID) // delete from the open list mapping

		if currentNodeID == endNodeID {
			// reconstruct the path
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

		// check the neighbors
		neighbors := nq.navData.GetNeighbors(currentNodeID)
		for _, neighborID := range neighbors {
			if closedSet[neighborID] {
				continue
			}

			// calculate the movement cost
			tentativeG := gScore[currentNodeID] + nq.calculateEnhancedMovementCost(currentNodeID, neighborID)

			if existingG, exists := gScore[neighborID]; !exists || tentativeG < existingG {
				cameFrom[neighborID] = currentNodeID
				gScore[neighborID] = tentativeG
				neighborNode := &nq.navData.Nodes[neighborID]
				fScore[neighborID] = tentativeG + nq.heuristic(neighborNode.Center, endNode.Center)

				// optimization: use map to quickly check if the node is in the open list
				if existingHeapNode, exists := inOpenSet[neighborID]; exists {
					// update the fScore of the existing node and fix the heap
					existingHeapNode.fScore = fScore[neighborID]
					heap.Fix(openSet, existingHeapNode.index)
				} else {
					// add the new node to the open list
					newHeapNode := newHeapNode(neighborID, fScore[neighborID])
					heap.Push(openSet, newHeapNode)
					inOpenSet[neighborID] = newHeapNode
				}
			}
		}
	}

	openSet.Clear()
	return nil // no path found
}

// AstarBidirectional is the bidirectional A* algorithm
func (nq *NavigationQuery) AstarBidirectional(startNodeID, endNodeID int32) []int32 {
	if startNodeID == endNodeID {
		return []int32{startNodeID}
	}

	// forward and backward open lists
	forwardOpen := &nodeHeap{}
	backwardOpen := &nodeHeap{}
	heap.Init(forwardOpen)
	heap.Init(backwardOpen)

	// closed list
	forwardClosed := make(map[int32]bool)
	backwardClosed := make(map[int32]bool)

	// gScore and cameFrom mapping
	gForward := make(map[int32]float32)
	gBackward := make(map[int32]float32)
	cameFromForward := make(map[int32]int32)
	cameFromBackward := make(map[int32]int32)

	// node references in the open list, for O(1) update
	inForwardOpen := make(map[int32]*heapNode)
	inBackwardOpen := make(map[int32]*heapNode)

	// start and end points
	startNode := &nq.navData.Nodes[startNodeID]
	endNode := &nq.navData.Nodes[endNodeID]

	// initialize
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

		// select the expansion direction: take the side with the smaller fScore of the current two heaps
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

	// reconstruct the path: start -> meetingNode
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
				// if the forward has no record, the meetingNode may be startNodeID or reached by the backward
				break
			}
			node = parent
		}
	}

	// meetingNode -> end (using the backward cameFrom)
	backwardPath := make([]int32, 0)
	{
		node := meetingNode
		for node != endNodeID {
			next, ok := cameFromBackward[node]
			if !ok {
				break
			}
			// append the next to the end
			backwardPath = append(backwardPath, next)
			node = next
		}
	}

	// merge, avoid duplicate meetingNode
	combined := make([]int32, 0, len(forwardPath)+len(backwardPath))
	combined = append(combined, forwardPath...)
	if len(backwardPath) > 0 {
		// backwardPath has already started from the next of the meetingNode
		combined = append(combined, backwardPath...)
	}

	// finally check the start and end
	if len(combined) == 0 {
		return nil
	}
	if combined[0] != startNodeID {
		// if the start is not included in the first half, add it
		combined = append([]int32{startNodeID}, combined...)
	}
	if combined[len(combined)-1] != endNodeID {
		combined = append(combined, endNodeID)
	}

	return combined
}

// heuristic is the heuristic function
func (nq *NavigationQuery) heuristic(a, b math32.Vector3) float32 {
	return a.Distance(b)
}

// ConvertToWorldPath converts the node path to world coordinate path
func (nq *NavigationQuery) ConvertToWorldPath(nodePath []int32, start, end math32.Vector3) []math32.Vector3 {
	if len(nodePath) == 0 {
		return nil
	}

	// convert the node ID to PathNode structure
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

	// use the visibility optimization algorithm
	smoothedPath := nq.SmoothPath(pathNodes)

	// if the visibility optimization algorithm returns an empty path, fall back to the simple method
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

	// ensure the start and end points are correct
	if len(smoothedPath) > 0 {
		smoothedPath[0] = start
		smoothedPath[len(smoothedPath)-1] = end
	}

	return smoothedPath
}

// GetNavData gets the navigation data
func (nq *NavigationQuery) GetNavData() *builder.NavigationData {
	return nq.navData
}

// GetStats gets the statistics
func (nq *NavigationQuery) GetStats() NavigationStats {
	return NavigationStats{
		NodeCount:     len(nq.navData.Nodes),
		EdgeCount:     len(nq.navData.Edges),
		GeometryCount: nq.navData.GetGeometryCount(),
		CacheSize:     nq.navData.GetSpatialCacheCount(),
		DataSize:      nq.navData.GetDataSize(),
	}
}

// NavigationStats is the navigation statistics
type NavigationStats struct {
	NodeCount     int // node count
	EdgeCount     int // edge count
	GeometryCount int // geometry count
	CacheSize     int // cache size
	DataSize      int // data size (bytes)
}
