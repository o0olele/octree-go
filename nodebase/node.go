package nodebase

import (
	"container/heap"
	"math"

	"github.com/o0olele/octree-go/octree"
)

// Node represents a node in the game map
type Node struct {
	Id     uint32
	AABB   octree.AABB
	Edges  []*Connector
	Octree *octree.Octree // Each node has its own octree for internal pathfinding

	// A* pathfinding fields
	GCost     float64
	HCost     float64
	FCost     float64
	Parent    *Node
	HeapIndex int
}

// Connector represents a connection between two nodes
type Connector struct {
	NodeA        *Node
	NodeB        *Node
	ConnectPoint octree.Vector3
	Width        float64 // Width of the connection for agent passage
}

// CalculateFCost updates the F cost for A* pathfinding
func (r *Node) CalculateFCost() {
	r.FCost = r.GCost + r.HCost
}

// GetCenter returns the center point of the node
func (r *Node) GetCenter() octree.Vector3 {
	return r.AABB.Center()
}

// ContainsPoint checks if a point is within this node
func (r *Node) ContainsPoint(point octree.Vector3) bool {
	return r.AABB.Contains(point)
}

// NodeGraph represents the graph of nodes and their connections
type NodeGraph struct {
	Nodes map[uint32]*Node
	Edges []*Connector
}

// NewNodeGraph creates a new node graph
func NewNodeGraph() *NodeGraph {
	return &NodeGraph{
		Nodes: make(map[uint32]*Node),
		Edges: make([]*Connector, 0),
	}
}

// AddNode adds a node to the graph
func (ng *NodeGraph) AddNode(node *Node) {
	ng.Nodes[node.Id] = node
}

// AddConnector adds a connection between two nodes
func (ng *NodeGraph) AddConnector(connector *Connector) {
	ng.Edges = append(ng.Edges, connector)

	// Add to both rooms' edge lists
	connector.NodeA.Edges = append(connector.NodeA.Edges, connector)
	connector.NodeB.Edges = append(connector.NodeB.Edges, connector)
}

// FindNodeContaining finds the node that contains the given point
func (ng *NodeGraph) FindNodeContaining(point octree.Vector3) *Node {
	for _, node := range ng.Nodes {
		if node.ContainsPoint(point) {
			return node
		}
	}
	return nil
}

// NodeNodeHeap implements heap.Interface for A* pathfinding
type NodeNodeHeap []*Node

func (h NodeNodeHeap) Len() int           { return len(h) }
func (h NodeNodeHeap) Less(i, j int) bool { return h[i].FCost < h[j].FCost }
func (h NodeNodeHeap) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].HeapIndex = i
	h[j].HeapIndex = j
}

func (h *NodeNodeHeap) Push(x interface{}) {
	node := x.(*Node)
	node.HeapIndex = len(*h)
	*h = append(*h, node)
}

func (h *NodeNodeHeap) Pop() interface{} {
	old := *h
	n := len(old)
	node := old[n-1]
	node.HeapIndex = -1
	*h = old[0 : n-1]
	return node
}

// RoomBasedPathfinder implements two-layer pathfinding
type NodeBasedPathfinder struct {
	nodeGraph *NodeGraph
	agent     *octree.Agent
}

// NewNodeBasedPathfinder creates a new node-based pathfinder
func NewNodeBasedPathfinder(nodeGraph *NodeGraph) *NodeBasedPathfinder {
	return &NodeBasedPathfinder{
		nodeGraph: nodeGraph,
	}
}

// SetAgent sets the agent for pathfinding
func (nbp *NodeBasedPathfinder) SetAgent(agent *octree.Agent) {
	nbp.agent = agent
}

// FindPath performs two-layer pathfinding
func (nbp *NodeBasedPathfinder) FindPath(start, end octree.Vector3) []octree.Vector3 {
	// Step 1: Find nodes containing start and end points
	startNode := nbp.nodeGraph.FindNodeContaining(start)
	endNode := nbp.nodeGraph.FindNodeContaining(end)

	if startNode == nil || endNode == nil {
		return nil // Start or end point not in any node
	}

	// Step 2: If start and end are in the same room, use direct octree pathfinding
	if startNode.Id == endNode.Id {
		return nbp.findPathInNode(startNode, start, end)
	}

	// Step 3: Find room-level path using A*
	nodePath := nbp.findNodePath(startNode, endNode)
	if nodePath == nil {
		return nil // No node-level path found
	}

	// Step 4: Convert node path to detailed world path
	return nbp.convertNodePathToWorldPath(nodePath, start, end)
}

// findRoomPath finds a path between rooms using A*
func (nbp *NodeBasedPathfinder) findNodePath(startNode, endNode *Node) []*Node {
	openHeap := &NodeNodeHeap{}
	heap.Init(openHeap)
	closedSet := make(map[uint32]bool)

	// Reset all room A* data
	for _, node := range nbp.nodeGraph.Nodes {
		node.GCost = math.Inf(1)
		node.HCost = 0
		node.FCost = math.Inf(1)
		node.Parent = nil
		node.HeapIndex = -1
	}

	// Initialize start room
	startNode.GCost = 0
	startNode.HCost = nbp.heuristic(startNode, endNode)
	startNode.CalculateFCost()

	heap.Push(openHeap, startNode)

	maxIterations := 1000
	iterations := 0

	for openHeap.Len() > 0 && iterations < maxIterations {
		iterations++

		current := heap.Pop(openHeap).(*Node)

		if current.Id == endNode.Id {
			return nbp.reconstructNodePath(current)
		}

		closedSet[current.Id] = true

		// Check all connected rooms
		for _, connector := range current.Edges {
			neighbor := connector.NodeA
			if neighbor.Id == current.Id {
				neighbor = connector.NodeB
			}

			if closedSet[neighbor.Id] {
				continue
			}

			// Calculate movement cost (distance between room centers)
			tentativeGCost := current.GCost + current.GetCenter().Distance(neighbor.GetCenter())

			if tentativeGCost < neighbor.GCost {
				neighbor.Parent = current
				neighbor.GCost = tentativeGCost
				neighbor.HCost = nbp.heuristic(neighbor, endNode)
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

// heuristic calculates the heuristic distance between two rooms
func (nbp *NodeBasedPathfinder) heuristic(nodeA, nodeB *Node) float64 {
	return nodeA.GetCenter().Distance(nodeB.GetCenter())
}

// reconstructRoomPath reconstructs the room path from A* result
func (nbp *NodeBasedPathfinder) reconstructNodePath(node *Node) []*Node {
	var path []*Node
	current := node

	for current != nil {
		path = append([]*Node{current}, path...)
		current = current.Parent
	}

	return path
}

// findPathInNode finds a path within a single node using its octree
func (nbp *NodeBasedPathfinder) findPathInNode(node *Node, start, end octree.Vector3) []octree.Vector3 {
	if node.Octree == nil {
		// If no octree, return direct path
		return []octree.Vector3{start, end}
	}

	// Create a pathfinder for this node's octree
	pathfinder := octree.NewNodeBasedAStarPathfinder(node.Octree, 0.5)
	if nbp.agent != nil {
		pathfinder.SetAgent(nbp.agent)
	}

	return pathfinder.FindPath(start, end)
}

// convertNodePathToWorldPath converts node-level path to detailed world coordinates
func (nbp *NodeBasedPathfinder) convertNodePathToWorldPath(nodePath []*Node, start, end octree.Vector3) []octree.Vector3 {
	if len(nodePath) == 0 {
		return nil
	}

	var worldPath []octree.Vector3
	worldPath = append(worldPath, start)

	// For each consecutive pair of nodes, find the connection point and path through it
	for i := 0; i < len(nodePath)-1; i++ {
		currentNode := nodePath[i]
		nextNode := nodePath[i+1]

		// Find the connector between these nodes
		connector := nbp.findConnector(currentNode, nextNode)
		if connector == nil {
			continue // Skip if no connector found
		}

		// Determine exit point
		var exitPoint octree.Vector3

		if i == 0 {
			// First node: path from start to connector
			exitPoint = connector.ConnectPoint
			nodePath := nbp.findPathInNode(currentNode, start, exitPoint)
			if len(nodePath) > 1 {
				worldPath = append(worldPath, nodePath[1:len(nodePath)-1]...) // Exclude start and end
			}
		} else {
			// Middle nodes: path from previous connector to next connector
			prevConnector := nbp.findConnector(nodePath[i-1], currentNode)
			if prevConnector != nil {
				exitPoint = connector.ConnectPoint
				nodePath := nbp.findPathInNode(currentNode, prevConnector.ConnectPoint, exitPoint)
				if len(nodePath) > 1 {
					worldPath = append(worldPath, nodePath[1:len(nodePath)-1]...) // Exclude start and end
				}
			}
		}

		// Add the connection point
		worldPath = append(worldPath, connector.ConnectPoint)
	}

	// Handle the last node: path from connector to end
	if len(nodePath) > 1 {
		lastNode := nodePath[len(nodePath)-1]
		prevNode := nodePath[len(nodePath)-2]
		connector := nbp.findConnector(prevNode, lastNode)

		if connector != nil {
			nodePath := nbp.findPathInNode(lastNode, connector.ConnectPoint, end)
			if len(nodePath) > 1 {
				worldPath = append(worldPath, nodePath[1:]...) // Exclude start, include end
			}
		}
	} else {
		// Single node case
		worldPath = append(worldPath, end)
	}

	return worldPath
}

// findConnector finds the connector between two nodes
func (nbp *NodeBasedPathfinder) findConnector(nodeA, nodeB *Node) *Connector {
	for _, connector := range nodeA.Edges {
		if (connector.NodeA.Id == nodeA.Id && connector.NodeB.Id == nodeB.Id) ||
			(connector.NodeA.Id == nodeB.Id && connector.NodeB.Id == nodeA.Id) {
			return connector
		}
	}
	return nil
}

// GetNodeGraph returns the node graph for external access
func (nbp *NodeBasedPathfinder) GetNodeGraph() *NodeGraph {
	return nbp.nodeGraph
}
