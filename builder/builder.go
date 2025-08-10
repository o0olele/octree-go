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

// Builder 导航网格构建器，类似于Recast
type Builder struct {
	octree      *octree.Octree
	pathfinder  *octree.NodeBasedAStarPathfinder
	bounds      geometry.AABB
	voxelGrid   *voxel.VoxelGrid
	maxDepth    uint8
	minSize     float32
	stepSize    float32
	useParallel bool
	usePrune    bool
	useVoxel    bool
}

// NewBuilder 创建新的导航网格构建器
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

func (nb *Builder) SetUsePrune(usePrune bool) {
	nb.usePrune = usePrune
}

func (nb *Builder) SetUseVoxel(useVoxel bool) {
	nb.useVoxel = useVoxel
}

func (nb *Builder) GetOctree() *octree.Octree {
	return nb.octree
}

func (nb *Builder) GetVoxelGrid() *voxel.VoxelGrid {
	return nb.voxelGrid
}

func (nb *Builder) GetPathFinder() *octree.NodeBasedAStarPathfinder {
	return nb.pathfinder
}

// AddTriangle 添加几何体到构建器
func (nb *Builder) AddTriangle(triangle geometry.Triangle) {
	nb.octree.AddTriangle(triangle)
}

// AddTriangles 批量添加几何体
func (nb *Builder) AddTriangles(triangles []geometry.Triangle) {
	nb.octree.AddTriangles(triangles)
}

// SetParallel 设置是否使用并行构建
func (nb *Builder) SetParallel(parallel bool) {
	nb.useParallel = parallel
}

func (nb *Builder) calculateVoxelSize(stepSize float32) math32.Vector3i {
	return math32.Vector3i{
		X: int32(math.Ceil(float64(nb.bounds.Size().X / stepSize))),
		Y: int32(math.Ceil(float64(nb.bounds.Size().Y / stepSize))),
		Z: int32(math.Ceil(float64(nb.bounds.Size().Z / stepSize))),
	}
}

// Build 构建导航网格，返回可序列化的导航数据
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
		nb.voxelGrid = voxel.NewVoxelGrid(nb.calculateVoxelSize(agent.Radius), agent.Radius, nb.bounds.Min)
		nb.voxelGrid.VoxelizeWithPadding(nb.octree.GetTriangles(), agent.Radius, agent.Height)
	}

	// 3. 创建导航数据
	navData := nb.createNavigationData()

	totalTime := time.Since(startTime)
	fmt.Printf("Total build time: %v\n", totalTime)

	return navData, nil
}

// collectEmptyLeaves 收集所有空白叶子节点
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

// createNavigationData 创建优化的导航数据结构
func (nb *Builder) createNavigationData() *NavigationData {

	// 收集所有空白叶子节点
	emptyNodes := nb.collectEmptyLeaves(nb.octree.Root)

	// 创建紧凑的节点数据
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

	// 创建紧凑的边数据
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

	// 邻接去冗余与裁剪：去重复、过滤长边、限制每个节点的最大度
	if nb.usePrune {
		edges = pruneRedundantEdges(edges, nodes, 12, 2.5)
		fmt.Printf("Edges after pruning: %d\n", len(edges))
	}

	// 构建Morton索引
	mortonNodes := make([]octree.MortonNodePair, 0, len(nodes))
	mortonResolution := uint32(1024) // 固定分辨率

	for i, node := range nodes {
		morton := octree.Vector3ToMorton(node.Bounds.Center(), nb.bounds, mortonResolution)
		mortonNodes = append(mortonNodes, octree.MortonNodePair{
			Morton: morton,
			Node:   &octree.PathNode{ID: int32(i), Center: node.Bounds.Center(), Bounds: node.Bounds},
		})
	}

	// 对Morton编码排序
	sort.Slice(mortonNodes, func(i, j int) bool {
		return mortonNodes[i].Morton < mortonNodes[j].Morton
	})

	// 创建Morton索引
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
		// 使用体素数据作为碰撞检测
		data.VoxelData = nb.voxelGrid.ToBitmap()
		data.GridSize = nb.voxelGrid.Size
		data.VoxelSize = nb.voxelGrid.VoxelSize

		// 释放几何体数据
		data.GeometryData = nil
	}

	return data
}

// pruneRedundantEdges 对边进行去重与裁剪
// - 移除重复/近似重复边（按无向对保留最小代价）
// - 过滤相对长边：若一条边长度大于其两端最小邻接边长度的 factor 倍则丢弃
// - 限制每个节点的最大度，优先保留代价更低的边
func pruneRedundantEdges(edges []CompactEdge, _ []CompactNode, maxDegree int, longEdgeFactor float32) []CompactEdge {
	if len(edges) == 0 {
		return edges
	}

	// 1) 去重：无向对 -> 最小 cost
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

	// 2) 统计每节点的最小相邻边长
	minLen := make(map[int32]float32)
	for _, e := range dedup {
		if v, ok := minLen[e.NodeAID]; !ok || e.Cost < v {
			minLen[e.NodeAID] = e.Cost
		}
		if v, ok := minLen[e.NodeBID]; !ok || e.Cost < v {
			minLen[e.NodeBID] = e.Cost
		}
	}

	// 3) 过滤相对过长的边
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

	// 4) 限制每个节点的最大度：按 cost 升序保留
	if maxDegree > 0 {
		// 建邻接
		adj := make(map[int32][]CompactEdge)
		for _, e := range filtered {
			adj[e.NodeAID] = append(adj[e.NodeAID], e)
			// 对称也加入，方便统计度
			adj[e.NodeBID] = append(adj[e.NodeBID], CompactEdge{NodeAID: e.NodeBID, NodeBID: e.NodeAID, Cost: e.Cost})
		}

		// 标记保留的无向边
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

		// 收集保留边：需两端至少一端标记保留
		final := make([]CompactEdge, 0, len(filtered))
		for _, e := range filtered {
			if keep[key(e.NodeAID, e.NodeBID)] {
				final = append(final, e)
			}
		}

		// 最后再做一次严格度裁剪，确保不超过 maxDegree
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

// GetMemoryUsage 获取构建过程的内存使用情况
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
		stats.PathNodes = len(nb.pathfinder.GetGraph().Nodes)
		stats.PathEdges = len(nb.pathfinder.GetGraph().Edges)
	}

	return stats
}

// countOctreeNodes 统计octree节点数量
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

// BuildMemoryStats 构建过程的内存统计
type BuildMemoryStats struct {
	TotalAlloc  uint64 // 总分配内存
	Sys         uint64 // 系统内存
	HeapAlloc   uint64 // 堆内存
	HeapSys     uint64 // 堆系统内存
	NumGC       uint32 // GC次数
	OctreeNodes int    // Octree节点数量
	PathNodes   int    // 路径节点数量
	PathEdges   int    // 路径边数量
}

// SerializableGeometry 可序列化的几何体
type SerializableGeometry struct {
	Type string      `json:"type"`
	Data interface{} `json:"data"`
}
