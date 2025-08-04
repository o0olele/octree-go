package builder

import (
	"fmt"
	"runtime"
	"sort"
	"time"

	"github.com/o0olele/octree-go/geometry"
	"github.com/o0olele/octree-go/math32"
	"github.com/o0olele/octree-go/octree"
)

// Builder 导航网格构建器，类似于Recast
type Builder struct {
	octree      *octree.Octree
	pathfinder  *octree.NodeBasedAStarPathfinder
	bounds      geometry.AABB
	maxDepth    uint8
	minSize     float32
	stepSize    float32
	useParallel bool
}

// NewBuilder 创建新的导航网格构建器
func NewBuilder(bounds geometry.AABB, maxDepth uint8, minSize float32, stepSize float32) *Builder {
	return &Builder{
		bounds:      bounds,
		maxDepth:    maxDepth,
		minSize:     minSize,
		stepSize:    stepSize,
		useParallel: true,
		octree:      octree.NewOctree(bounds, maxDepth, minSize),
	}
}

func (nb *Builder) GetOctree() *octree.Octree {
	return nb.octree
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

	return &NavigationData{
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
