package builder

import (
	"fmt"

	"github.com/o0olele/octree-go/geometry"
	"github.com/o0olele/octree-go/math32"
	"github.com/o0olele/octree-go/octree"
)

// File format constants
const (
	NAVIGATION_FILE_MAGIC   = 0x4E415649 // "NAVI"
	NAVIGATION_FILE_VERSION = 1
)

// FileHeader is the header of the navigation data file.
type FileHeader struct {
	Magic   uint32 // Magic number
	Version uint32 // Version number
}

// CompactNode is the compact node data structure.
type CompactNode struct {
	ID     int32          `json:"id"`
	Bounds geometry.AABB  `json:"bounds"`
	Center math32.Vector3 `json:"-"`
}

// CompactEdge is the compact edge data structure.
type CompactEdge struct {
	NodeAID int32   `json:"node_a_id"`
	NodeBID int32   `json:"node_b_id"`
	Cost    float32 `json:"cost"`
}

// NavigationData is the compact navigation data structure, similar to the format of Detour.
type NavigationData struct {
	// Basic information
	Bounds    geometry.AABB   `json:"bounds"`     // The bounds of the navigation data.
	MaxDepth  uint8           `json:"max_depth"`  // The max depth of the octree.
	MinSize   float32         `json:"min_size"`   // The min size of the octree.
	StepSize  float32         `json:"step_size"`  // The step size of the octree.
	GridSize  math32.Vector3i `json:"grid_size"`  // The size of the voxel grid.
	VoxelSize float32         `json:"voxel_size"` // The size of the voxel.

	// Node and edge data
	Nodes []CompactNode `json:"nodes"` // The nodes of the navigation data.
	Edges []CompactEdge `json:"edges"` // The edges of the navigation data.

	// Spatial index
	MortonIndex      []int32 `json:"morton_index"`      // The morton index of the navigation data.
	MortonResolution uint32  `json:"morton_resolution"` // The morton resolution of the navigation data.

	GeometryData []geometry.Triangle `json:"geometry_data"` // Geometry data, using triangles (for collision detection)
	VoxelData    []uint64            `json:"voxel_data"`    // Voxel data, using bitmap (for collision detection)

	// Adjacency table index (built at runtime, not serialized)
	// Compressed adjacency (CSR): continuous adjacency array + offset + degree
	csrNeighbors []int32                 `json:"-"` // neighbors array
	csrOffsets   []int32                 `json:"-"` // len = number of nodes
	csrDegrees   []int32                 `json:"-"` // len = number of nodes
	edgeIndex    map[int32][]CompactEdge `json:"-"`

	// Pre-computed cache (built at runtime, not serialized)
	nodeNeighborCounts    []int32        `json:"-"`
	nodeBoundaryDistances []float32      `json:"-"`
	edgePairToIndex       map[uint64]int `json:"-"`

	initialized  bool                         `json:"-"` // Whether the navigation data is initialized
	octree       *octree.Octree               `json:"-"` // Octree, build from GeometryData when not using voxel data
	spatialCache *math32.Cache[uint64, int32] `json:"-"` // Spatial query optimization - using LRU cache
}

// init initializes the navigation data.
func (nd *NavigationData) init() error {
	// Pre-build indexes to improve query performance
	nd.BuildIndexes()

	// Build octree if using geometry data
	if len(nd.GeometryData) > 0 {
		nd.octree = octree.NewOctree(nd.Bounds, nd.MaxDepth, nd.StepSize)
		nd.octree.SetTriangles(nd.GeometryData)
		nd.octree.Build()
	}

	// Initialize spatial cache
	nd.spatialCache = math32.NewCache[uint64, int32](10000)
	return nil
}

// GetOctree returns the octree.
func (nd *NavigationData) GetOctree() *octree.Octree {
	return nd.octree
}

// GetDataSize gets the size of the navigation data (in bytes).
func (nd *NavigationData) GetDataSize() int {
	size := 0

	// Basic information
	size += 4 * 6 // bounds (6 float32)
	size += 4     // max_depth
	size += 4     // min_size
	size += 4     // step_size
	size += 4 * 3 // grid_size (3 int32)
	size += 4     // voxel_size

	// Node data
	size += len(nd.Nodes) * (4 + 4*6 + 4*3) // id + center + bounds

	// Edge data
	size += len(nd.Edges) * (4 + 4 + 4) // nodeA + nodeB + cost

	// Morton index
	size += len(nd.MortonIndex) * 4
	size += 4 // morton_resolution

	// Geometry data
	size += len(nd.GeometryData) * 3 * 3 * 4 // 3 vertices (3 float32)

	// Voxel data
	size += len(nd.VoxelData) * 8 // 8 bytes per voxel

	return size
}

// GetNodeCount gets the number of nodes.
func (nd *NavigationData) GetNodeCount() int {
	return len(nd.Nodes)
}

// GetEdgeCount gets the number of edges.
func (nd *NavigationData) GetEdgeCount() int {
	return len(nd.Edges)
}

// GetGeometryCount gets the number of geometries.
func (nd *NavigationData) GetGeometryCount() int {
	return len(nd.GeometryData)
}

// GetGeometryData gets the geometry data.
func (nd *NavigationData) GetGeometryData() []geometry.Triangle {
	return nd.GeometryData
}

// GetSpatialCacheCount gets the number of spatial cache.
func (nd *NavigationData) GetSpatialCacheCount() int {
	return nd.spatialCache.Len()
}

// FindNodeByID finds the node by ID.
func (nd *NavigationData) FindNodeByID(id int) *CompactNode {
	if id < 0 || id >= len(nd.Nodes) {
		return nil
	}
	return &nd.Nodes[id]
}

// GetNodeEdges gets the edges of a node.
func (nd *NavigationData) GetNodeEdges(nodeID int32) []CompactEdge {
	// Ensure the index is initialized
	if nd.edgeIndex == nil || !nd.initialized {
		nd.BuildIndexes()
	}
	return nd.edgeIndex[nodeID]
}

// GetNeighbors gets the neighbors of a node.
func (nd *NavigationData) GetNeighbors(nodeID int32) []int32 {
	if nd.csrNeighbors == nil || nd.csrOffsets == nil || nd.csrDegrees == nil || !nd.initialized {
		// Lazy load the index
		nd.BuildIndexes()
	}
	if nodeID < 0 || int(nodeID) >= len(nd.csrOffsets) {
		return nil
	}
	off := nd.csrOffsets[nodeID]
	deg := nd.csrDegrees[nodeID]
	if deg == 0 {
		return nil
	}
	return nd.csrNeighbors[off : off+deg]
}

// BuildIndexes builds the adjacency table and edge index.
func (nd *NavigationData) BuildIndexes() {
	if nd.initialized {
		return
	}

	n := len(nd.Nodes)
	deg := make([]int32, n)
	for _, e := range nd.Edges {
		if e.NodeAID >= 0 && int(e.NodeAID) < n {
			deg[e.NodeAID]++
		}
		if e.NodeBID >= 0 && int(e.NodeBID) < n {
			deg[e.NodeBID]++
		}
	}

	offsets := make([]int32, n)
	var total int32 = 0
	for i := 0; i < n; i++ {
		offsets[i] = total
		total += deg[i]
	}
	neighbors := make([]int32, total)
	// Temporary write pointer
	cursor := make([]int32, n)
	copy(cursor, offsets)

	for _, e := range nd.Edges {
		a := e.NodeAID
		b := e.NodeBID
		if a >= 0 && int(a) < n {
			pos := cursor[a]
			neighbors[pos] = b
			cursor[a] = pos + 1
		}
		if b >= 0 && int(b) < n {
			pos := cursor[b]
			neighbors[pos] = a
			cursor[b] = pos + 1
		}
	}

	// Build the edge index (keep the map for simplicity)
	nd.edgeIndex = make(map[int32][]CompactEdge)
	for _, e := range nd.Edges {
		nd.edgeIndex[e.NodeAID] = append(nd.edgeIndex[e.NodeAID], e)
		nd.edgeIndex[e.NodeBID] = append(nd.edgeIndex[e.NodeBID], e)
	}

	nd.csrNeighbors = neighbors
	nd.csrOffsets = offsets
	nd.csrDegrees = deg

	// Pre-compute: node neighbor count cache
	nd.nodeNeighborCounts = make([]int32, n)
	copy(nd.nodeNeighborCounts, deg)

	// Pre-compute: node to scene boundary minimum distance
	nd.nodeBoundaryDistances = make([]float32, n)
	for i := 0; i < n; i++ {
		center := nd.Nodes[i].Bounds.Center()
		dx := float32(math32.Min(center.X-nd.Bounds.Min.X, nd.Bounds.Max.X-center.X))
		dy := float32(math32.Min(center.Y-nd.Bounds.Min.Y, nd.Bounds.Max.Y-center.Y))
		dz := float32(math32.Min(center.Z-nd.Bounds.Min.Z, nd.Bounds.Max.Z-center.Z))
		nd.nodeBoundaryDistances[i] = math32.Min(dx, math32.Min(dy, dz))
	}

	// Pre-compute: edge pair to index mapping
	nd.edgePairToIndex = make(map[uint64]int, len(nd.Edges))
	for idx, e := range nd.Edges {
		a := e.NodeAID
		b := e.NodeBID
		if a > b {
			a, b = b, a
		}
		key := (uint64(uint32(a)) << 32) | uint64(uint32(b))
		nd.edgePairToIndex[key] = idx
	}

	nd.initialized = true
}

// Validate validates the navigation data.
func (nd *NavigationData) Validate() error {
	// Check node ID continuity
	for i, node := range nd.Nodes {
		nd.Nodes[i].Center = node.Bounds.Center()
		if node.ID != int32(i) {
			return fmt.Errorf("node ID mismatch at index %d: expected %d, got %d", i, i, node.ID)
		}
	}

	// Check the validity of the edges
	for i, edge := range nd.Edges {
		if edge.NodeAID < 0 || edge.NodeAID >= int32(len(nd.Nodes)) {
			return fmt.Errorf("edge %d has invalid NodeAID: %d", i, edge.NodeAID)
		}
		if edge.NodeBID < 0 || edge.NodeBID >= int32(len(nd.Nodes)) {
			return fmt.Errorf("edge %d has invalid NodeBID: %d", i, edge.NodeBID)
		}
		if edge.Cost < 0 {
			return fmt.Errorf("edge %d has negative cost: %f", i, edge.Cost)
		}
	}

	// Check the validity of the morton index
	if len(nd.MortonIndex) != len(nd.Nodes) {
		return fmt.Errorf("morton index length mismatch: expected %d, got %d", len(nd.Nodes), len(nd.MortonIndex))
	}

	for i, nodeID := range nd.MortonIndex {
		if nodeID < 0 || nodeID >= int32(len(nd.Nodes)) {
			return fmt.Errorf("morton index %d has invalid node ID: %d", i, nodeID)
		}
	}

	return nil
}

// IsPathClear checks if the path is clear between two points.
func (nd *NavigationData) IsPathClear(agent *octree.Agent, start, end math32.Vector3) bool {
	if len(nd.VoxelData) > 0 {
		return nd.IsPathClearByVoxel(agent, start, end)
	}
	return nd.octree.IsPathClear(agent, start, end)
}

// GetNodeNeighborCount gets the number of neighbors of a node.
func (nd *NavigationData) GetNodeNeighborCount(nodeID int32) int32 {
	if nd.nodeNeighborCounts == nil || !nd.initialized {
		nd.BuildIndexes()
	}
	if nodeID < 0 || int(nodeID) >= len(nd.nodeNeighborCounts) {
		return 0
	}
	return nd.nodeNeighborCounts[nodeID]
}

// GetNodeBoundaryDistance gets the minimum distance from a node to the boundary.
func (nd *NavigationData) GetNodeBoundaryDistance(nodeID int32) float32 {
	if nd.nodeBoundaryDistances == nil || !nd.initialized {
		nd.BuildIndexes()
	}
	if nodeID < 0 || int(nodeID) >= len(nd.nodeBoundaryDistances) {
		return 0
	}
	return nd.nodeBoundaryDistances[nodeID]
}

// FindEdgeIndex finds the index of an edge.
func (nd *NavigationData) FindEdgeIndex(nodeAID, nodeBID int32) int {
	if nd.edgePairToIndex == nil || !nd.initialized {
		nd.BuildIndexes()
	}
	a, b := nodeAID, nodeBID
	if a > b {
		a, b = b, a
	}
	key := (uint64(uint32(a)) << 32) | uint64(uint32(b))
	if idx, ok := nd.edgePairToIndex[key]; ok {
		return idx
	}
	return -1
}

// GetEdgeCostByNodes gets the cost of an edge by node IDs.
func (nd *NavigationData) GetEdgeCostByNodes(nodeAID, nodeBID int32) (float32, bool) {
	idx := nd.FindEdgeIndex(nodeAID, nodeBID)
	if idx < 0 {
		return 0, false
	}
	return nd.Edges[idx].Cost, true
}

// IsWorldOccupiedByVoxel checks if a world position is occupied by a voxel.
func (nd *NavigationData) IsWorldOccupiedByVoxel(worldPos math32.Vector3) bool {
	if len(nd.VoxelData) == 0 {
		return false
	}
	localPos := worldPos.Sub(nd.Bounds.Min)
	coord := math32.Vector3i{
		X: int32(localPos.X / nd.VoxelSize),
		Y: int32(localPos.Y / nd.VoxelSize),
		Z: int32(localPos.Z / nd.VoxelSize),
	}
	bitmap := math32.Bitmap(nd.VoxelData)
	return bitmap.Contains(uint32(coord.Z*nd.GridSize.X*nd.GridSize.Y + coord.Y*nd.GridSize.X + coord.X))
}

// IsPathClearByVoxel checks if the path is clear between two points by voxel.
func (nd *NavigationData) IsPathClearByVoxel(agent *octree.Agent, start, end math32.Vector3) bool {
	// Calculate the direction vector and distance
	direction := end.Sub(start)
	distance := direction.Length()

	if distance < 0.001 { // The distance is too close, considered as the same point
		return true
	}

	// Normalize the direction vector
	direction = direction.Scale(1.0 / distance)

	agentRadius := float32(0.4)
	if agent != nil {
		agentRadius = agent.Radius
	}
	// Use appropriate step size for sampling check
	stepSize := math32.Max(0.1, agentRadius*0.6)
	steps := math32.CeilToInt(distance / stepSize)

	// Sample along the path for detection
	for i := 0; i <= steps; i++ {
		t := float32(i) / float32(steps)
		samplePoint := start.Add(direction.Scale(distance * t))

		// If there is an agent radius, also check the agent collision
		occupied := nd.IsWorldOccupiedByVoxel(samplePoint)
		if occupied {
			return false
		}

	}

	return true
}
