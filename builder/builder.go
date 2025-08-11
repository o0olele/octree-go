package builder

import (
	"fmt"
	"math"
	"runtime"
	"sort"
	"time"

	"github.com/o0olele/octree-go/geometry"
	"github.com/o0olele/octree-go/math32"
	"github.com/o0olele/octree-go/octree"
	"github.com/o0olele/octree-go/voxel"
)

// Builder is a tool for building navigation data.
type Builder struct {
	octree      *octree.Octree                   // The octree used to build the navigation data.
	pathfinder  *octree.NodeBasedAStarPathfinder // The pathfinder used to build the navigation data.
	bounds      geometry.AABB                    // The bounds of the navigation data.
	voxelGrid   *voxel.VoxelGrid                 // The voxel grid used to build the navigation data.
	maxDepth    uint8                            // The max depth of the octree.
	minSize     float32                          // The min size of the octree.
	stepSize    float32                          // The step size of the octree.
	useParallel bool                             // Whether to use parallel building.
	usePrune    bool                             // Whether to use prune to remove redundant edges.
	useVoxel    bool                             // Whether to use voxel for collision detection.
}

// NewBuilder creates a new builder.
func NewBuilder(bounds geometry.AABB, maxDepth uint8, minSize float32, stepSize float32) *Builder {
	return &Builder{
		bounds:      bounds,
		maxDepth:    maxDepth,
		minSize:     minSize,
		stepSize:    stepSize,
		useParallel: true,
		usePrune:    true,
		octree:      octree.NewOctree(bounds, maxDepth, minSize),
	}
}

// SetUsePrune sets whether to use prune to remove redundant edges.
func (nb *Builder) SetUsePrune(usePrune bool) {
	nb.usePrune = usePrune
}

// SetUseVoxel sets whether to use voxel for collision detection.
func (nb *Builder) SetUseVoxel(useVoxel bool) {
	nb.useVoxel = useVoxel
}

// GetOctree returns the octree.
func (nb *Builder) GetOctree() *octree.Octree {
	return nb.octree
}

// GetVoxelGrid returns the voxel grid.
func (nb *Builder) GetVoxelGrid() *voxel.VoxelGrid {
	return nb.voxelGrid
}

// GetPathFinder returns the pathfinder.
func (nb *Builder) GetPathFinder() *octree.NodeBasedAStarPathfinder {
	return nb.pathfinder
}

// AddTriangle adds a triangle to the builder.
func (nb *Builder) AddTriangle(triangle geometry.Triangle) {
	nb.octree.AddTriangle(triangle)
}

// AddTriangles adds a batch of triangles to the builder.
func (nb *Builder) AddTriangles(triangles []geometry.Triangle) {
	nb.octree.AddTriangles(triangles)
}

// SetParallel sets whether to use parallel building.
func (nb *Builder) SetParallel(parallel bool) {
	nb.useParallel = parallel
}

// calculateVoxelGridSize calculates the voxel grid size.
func (nb *Builder) calculateVoxelGridSize(stepSize float32) math32.Vector3i {
	return math32.Vector3i{
		X: int32(math.Ceil(float64(nb.bounds.Size().X / stepSize))),
		Y: int32(math.Ceil(float64(nb.bounds.Size().Y / stepSize))),
		Z: int32(math.Ceil(float64(nb.bounds.Size().Z / stepSize))),
	}
}

// Build builds the navigation data.
func (nb *Builder) Build(agent *octree.Agent) (*NavigationData, error) {
	startTime := time.Now()

	// 1. 构建Octree
	fmt.Println("Building octree...")
	octreeStart := time.Now()
	fmt.Printf("Octree triangles added: %d\n", nb.octree.GetTrianglesNum())
	nb.octree.Build()
	octreeTime := time.Since(octreeStart)
	fmt.Printf("Octree built in %v\n", octreeTime)

	// 2. 构建PathGraph
	fmt.Println("Building pathfinder...")
	pathfinderStart := time.Now()
	nb.pathfinder = octree.NewNodeBasedAStarPathfinderWithParallel(nb.octree, agent, nb.stepSize, nb.useParallel)
	pathfinderTime := time.Since(pathfinderStart)
	fmt.Printf("Pathfinder built in %v\n", pathfinderTime)

	if nb.useVoxel && agent != nil {
		var voxelSize float32 = agent.Radius * 0.5
		nb.voxelGrid = voxel.NewVoxelGrid(nb.calculateVoxelGridSize(voxelSize), voxelSize, nb.bounds.Min)
		nb.voxelGrid.VoxelizeWithPadding(nb.octree.GetTriangles(), agent.Radius, agent.Height)
	}

	// 3. 创建导航数据
	navData := nb.createNavigationData()

	totalTime := time.Since(startTime)
	fmt.Printf("Total build time: %v\n", totalTime)

	return navData, nil
}

// collectEmptyLeaves collects all empty leaves from the octree.
func (nb *Builder) collectEmptyLeaves(node *octree.OctreeNode) []*octree.OctreeNode {
	var emptyNodes []*octree.OctreeNode

	if node.IsLeaf() {
		if !node.IsOccupied() {
			emptyNodes = append(emptyNodes, node)
		}
	} else {
		for _, child := range node.Children {
			if child != nil {
				emptyNodes = append(emptyNodes, nb.collectEmptyLeaves(child)...)
			}
		}
	}

	return emptyNodes
}

// createNavigationData creates an optimized navigation data structure.
func (nb *Builder) createNavigationData() *NavigationData {

	// 1. Collect all empty leaves from the octree.
	emptyNodes := nb.collectEmptyLeaves(nb.octree.Root)

	// 2. Create compact node data.
	nodes := make([]CompactNode, 0, len(emptyNodes))
	nodeMap := make(map[*octree.OctreeNode]int32) // 原始节点到新索引的映射

	for i, octreeNode := range emptyNodes {
		node := CompactNode{
			ID:     int32(i),
			Bounds: octreeNode.Bounds,
		}
		nodes = append(nodes, node)
		nodeMap[octreeNode] = int32(i)
	}

	// 3. Create compact edge data.
	edges := make([]CompactEdge, 0)
	if nb.pathfinder != nil && nb.pathfinder.GetPathGraph() != nil {
		for _, edge := range nb.pathfinder.GetPathGraph().Edges {
			if nodeAIdx, okA := nodeMap[edge.NodeA.OctreeNode]; okA {
				if nodeBIdx, okB := nodeMap[edge.NodeB.OctreeNode]; okB {
					compactEdge := CompactEdge{
						NodeAID: nodeAIdx,
						NodeBID: nodeBIdx,
						Cost:    edge.Cost,
					}
					edges = append(edges, compactEdge)
				}
			}
		}
	}

	// 4. Prune redundant edges.
	if nb.usePrune {
		edges = pruneRedundantEdges(edges, nodes, 12, 2.5)
		fmt.Printf("Edges after pruning: %d\n", len(edges))
	}

	// 5. Build Morton index.
	mortonNodes := make([]octree.MortonNodePair, 0, len(nodes))
	mortonResolution := uint32(1024) // Fixed resolution.

	for i, node := range nodes {
		morton := octree.Vector3ToMorton(node.Bounds.Center(), nb.bounds, mortonResolution)
		mortonNodes = append(mortonNodes, octree.MortonNodePair{
			Morton: morton,
			Node:   &octree.PathNode{ID: int32(i), Center: node.Bounds.Center(), Bounds: node.Bounds},
		})
	}

	// 6. Sort Morton index.
	sort.Slice(mortonNodes, func(i, j int) bool {
		return mortonNodes[i].Morton < mortonNodes[j].Morton
	})

	// 7. Create Morton index.
	mortonIndex := make([]int32, len(mortonNodes))
	for i, pair := range mortonNodes {
		mortonIndex[i] = pair.Node.ID
	}

	data := &NavigationData{
		Bounds:           nb.bounds,
		MaxDepth:         nb.maxDepth,
		MinSize:          nb.minSize,
		StepSize:         nb.stepSize,
		Nodes:            nodes,
		Edges:            edges,
		MortonIndex:      mortonIndex,
		MortonResolution: mortonResolution,
		GeometryData:     nb.octree.GetTriangles(),
		spatialCache:     math32.NewCache[uint64, int32](10000),
		octree:           nb.octree,
	}

	if nb.useVoxel {
		// Use voxel data as collision detection.
		data.VoxelData = nb.voxelGrid.ToBitmap()
		data.GridSize = nb.voxelGrid.Size
		data.VoxelSize = nb.voxelGrid.VoxelSize

		// Release geometry data.
		data.GeometryData = nil
	}

	return data
}

// pruneRedundantEdges removes redundant edges from the edges.
// - Remove duplicate/approximate duplicate edges (keep the minimum cost for each undirected pair).
// - Filter relative long edges: If an edge's length is more than factor times the minimum adjacent edge length of its two ends, discard it.
// - Limit the maximum degree of each node, prioritize edges with lower cost.
func pruneRedundantEdges(edges []CompactEdge, _ []CompactNode, maxDegree int, longEdgeFactor float32) []CompactEdge {
	if len(edges) == 0 {
		return edges
	}

	// 1. Remove duplicate edges.
	type pair struct{ a, b int32 }
	key := func(a, b int32) pair {
		if a > b {
			a, b = b, a
		}
		return pair{a, b}
	}
	uniq := make(map[pair]CompactEdge, len(edges))
	for _, e := range edges {
		k := key(e.NodeAID, e.NodeBID)
		if old, ok := uniq[k]; !ok || e.Cost < old.Cost {
			uniq[k] = e
		}
	}

	dedup := make([]CompactEdge, 0, len(uniq))
	for _, e := range uniq {
		dedup = append(dedup, e)
	}

	// 2. Count the minimum adjacent edge length of each node.
	minLen := make(map[int32]float32)
	for _, e := range dedup {
		if v, ok := minLen[e.NodeAID]; !ok || e.Cost < v {
			minLen[e.NodeAID] = e.Cost
		}
		if v, ok := minLen[e.NodeBID]; !ok || e.Cost < v {
			minLen[e.NodeBID] = e.Cost
		}
	}

	// 3. Filter relative long edges.
	filtered := dedup[:0]
	for _, e := range dedup {
		la := minLen[e.NodeAID]
		lb := minLen[e.NodeBID]
		if la == 0 || lb == 0 {
			filtered = append(filtered, e)
			continue
		}
		if e.Cost <= la*longEdgeFactor || e.Cost <= lb*longEdgeFactor {
			filtered = append(filtered, e)
		}
	}

	// 4. Limit the maximum degree of each node.
	if maxDegree > 0 {
		// Build adjacency list.
		adj := make(map[int32][]CompactEdge)
		for _, e := range filtered {
			adj[e.NodeAID] = append(adj[e.NodeAID], e)
			// Add symmetric edges for convenience of degree counting.
			adj[e.NodeBID] = append(adj[e.NodeBID], CompactEdge{NodeAID: e.NodeBID, NodeBID: e.NodeAID, Cost: e.Cost})
		}

		// Mark the edges to be kept.
		keep := make(map[pair]bool)

		for nodeID, list := range adj {
			sort.Slice(list, func(i, j int) bool { return list[i].Cost < list[j].Cost })
			if len(list) > maxDegree {
				list = list[:maxDegree]
			}
			for _, e := range list {
				keep[key(nodeID, e.NodeBID)] = true
			}
		}

		// Collect the edges to be kept.
		final := make([]CompactEdge, 0, len(filtered))
		for _, e := range filtered {
			if keep[key(e.NodeAID, e.NodeBID)] {
				final = append(final, e)
			}
		}

		// Strictly prune the edges to ensure the maximum degree is not exceeded.
		degree := make(map[int32]int)
		strict := final[:0]
		for _, e := range final {
			if degree[e.NodeAID] < maxDegree && degree[e.NodeBID] < maxDegree {
				strict = append(strict, e)
				degree[e.NodeAID]++
				degree[e.NodeBID]++
			}
		}
		return strict
	}

	return filtered
}

// GetMemoryUsage gets the memory usage of the builder.
func (nb *Builder) GetMemoryUsage() BuildMemoryStats {
	var m runtime.MemStats
	runtime.ReadMemStats(&m)

	stats := BuildMemoryStats{
		TotalAlloc: m.TotalAlloc,
		Sys:        m.Sys,
		HeapAlloc:  m.HeapAlloc,
		HeapSys:    m.HeapSys,
		NumGC:      m.NumGC,
	}

	if nb.octree != nil {
		stats.OctreeNodes = nb.countOctreeNodes(nb.octree.Root)
	}

	if nb.pathfinder != nil && nb.pathfinder.GetGraph() != nil {
		graph := nb.pathfinder.GetGraph()
		stats.PathNodes = len(graph.Nodes)
		stats.PathEdges = len(graph.Edges)
	}

	return stats
}

// countOctreeNodes counts the number of octree nodes.
func (nb *Builder) countOctreeNodes(node *octree.OctreeNode) int {

	if node == nil {
		return 0
	}

	count := 1
	if !node.IsLeaf() {
		for _, child := range node.Children {
			count += nb.countOctreeNodes(child)
		}
	}

	return count
}

// BuildMemoryStats is the memory usage of the builder.
type BuildMemoryStats struct {
	TotalAlloc  uint64 // Total allocated memory
	Sys         uint64 // System memory
	HeapAlloc   uint64 // Heap allocated memory
	HeapSys     uint64 // Heap system memory
	NumGC       uint32 // GC times
	OctreeNodes int    // Octree nodes
	PathNodes   int    // Path nodes
	PathEdges   int    // Path edges
}
