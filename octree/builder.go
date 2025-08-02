package octree

import (
	"bytes"
	"encoding/binary"
	"encoding/gob"
	"fmt"
	"os"
	"runtime"
	"time"

	"github.com/o0olele/octree-go/geometry"
)

// NavigationBuilder 导航网格构建器，类似于Recast
type NavigationBuilder struct {
	octree      *Octree
	pathfinder  *NodeBasedAStarPathfinder
	triangles   []geometry.Triangle
	bounds      geometry.AABB
	maxDepth    int
	minSize     float32
	stepSize    float32
	useParallel bool
}

// NewNavigationBuilder 创建新的导航网格构建器
func NewNavigationBuilder(bounds geometry.AABB, maxDepth int, minSize float32, stepSize float32) *NavigationBuilder {
	return &NavigationBuilder{
		bounds:      bounds,
		maxDepth:    maxDepth,
		minSize:     minSize,
		stepSize:    stepSize,
		useParallel: true,
		triangles:   make([]geometry.Triangle, 0),
	}
}

// AddTriangle 添加几何体到构建器
func (nb *NavigationBuilder) AddTriangle(triangle geometry.Triangle) {
	nb.triangles = append(nb.triangles, triangle)
}

// AddTriangles 批量添加几何体
func (nb *NavigationBuilder) AddTriangles(triangles []geometry.Triangle) {
	nb.triangles = append(nb.triangles, triangles...)
}

// SetParallel 设置是否使用并行构建
func (nb *NavigationBuilder) SetParallel(parallel bool) {
	nb.useParallel = parallel
}

// Build 构建导航网格，返回可序列化的导航数据
func (nb *NavigationBuilder) Build(agent *Agent) (*NavigationData, error) {
	startTime := time.Now()

	// 1. 构建Octree
	fmt.Println("Building octree...")
	octreeStart := time.Now()
	nb.octree = NewOctree(nb.bounds, nb.maxDepth, nb.minSize)

	for _, triangle := range nb.triangles {
		nb.octree.AddTriangle(triangle)
	}
	fmt.Printf("Octree triangles added: %d\n", len(nb.triangles))

	nb.octree.Build()
	octreeTime := time.Since(octreeStart)
	fmt.Printf("Octree built in %v\n", octreeTime)

	// 2. 构建PathGraph
	fmt.Println("Building pathfinder...")
	pathfinderStart := time.Now()
	nb.pathfinder = NewNodeBasedAStarPathfinderWithParallel(nb.octree, agent, nb.stepSize, nb.useParallel)
	pathfinderTime := time.Since(pathfinderStart)
	fmt.Printf("Pathfinder built in %v\n", pathfinderTime)

	// 3. 创建导航数据
	navData := nb.createNavigationData()

	totalTime := time.Since(startTime)
	fmt.Printf("Total build time: %v\n", totalTime)

	return navData, nil
}

// createNavigationData 创建优化的导航数据结构
func (nb *NavigationBuilder) createNavigationData() *NavigationData {
	// 收集所有空白叶子节点
	emptyNodes := nb.collectEmptyLeaves(nb.octree.Root)

	// 创建紧凑的节点数据
	nodes := make([]CompactNode, 0, len(emptyNodes))
	nodeMap := make(map[*OctreeNode]int) // 原始节点到新索引的映射

	for i, octreeNode := range emptyNodes {
		node := CompactNode{
			ID:     i,
			Center: octreeNode.Bounds.Center(),
			Bounds: octreeNode.Bounds,
		}
		nodes = append(nodes, node)
		nodeMap[octreeNode] = i
	}

	// 创建紧凑的边数据
	edges := make([]CompactEdge, 0)
	if nb.pathfinder != nil && nb.pathfinder.graph != nil {
		for _, edge := range nb.pathfinder.graph.Edges {
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
	mortonNodes := make([]MortonNodePair, 0, len(nodes))
	mortonResolution := uint32(1024) // 固定分辨率

	for i, node := range nodes {
		morton := Vector3ToMorton(node.Center, nb.bounds, mortonResolution)
		mortonNodes = append(mortonNodes, MortonNodePair{
			Morton: morton,
			Node:   &PathNode{ID: i, Center: node.Center, Bounds: node.Bounds},
		})
	}

	// 对Morton编码排序
	sortMortonNodes(mortonNodes)

	// 创建Morton索引
	mortonIndex := make([]int, len(mortonNodes))
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
		GeometryData:     nb.serializeGeometries(),
	}
}

// collectEmptyLeaves 收集所有空白叶子节点
func (nb *NavigationBuilder) collectEmptyLeaves(node *OctreeNode) []*OctreeNode {
	var emptyNodes []*OctreeNode

	if node.IsLeaf {
		if !node.IsOccupied {
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

// serializeGeometries 序列化几何体数据
func (nb *NavigationBuilder) serializeGeometries() []byte {
	if len(nb.triangles) == 0 {
		return []byte{}
	}

	var buf bytes.Buffer
	encoder := gob.NewEncoder(&buf)

	// 创建可序列化的几何体数据
	geomData := make([]geometry.Triangle, 0, len(nb.triangles))
	for _, triangle := range nb.triangles {
		geomData = append(geomData, triangle)
	}

	if err := encoder.Encode(geomData); err != nil {
		fmt.Printf("Error encoding geometries: %v\n", err)
		return []byte{}
	}

	return buf.Bytes()
}

// SaveToFile 保存导航数据到文件
func (nb *NavigationBuilder) SaveToFile(navData *NavigationData, filename string) error {
	file, err := os.Create(filename)
	if err != nil {
		return fmt.Errorf("failed to create file: %v", err)
	}
	defer file.Close()

	// 写入文件头
	header := NavigationFileHeader{
		Magic:   NAVIGATION_FILE_MAGIC,
		Version: NAVIGATION_FILE_VERSION,
	}

	if err := binary.Write(file, binary.LittleEndian, header); err != nil {
		return fmt.Errorf("failed to write header: %v", err)
	}

	// 使用gob编码写入数据
	encoder := gob.NewEncoder(file)
	if err := encoder.Encode(navData); err != nil {
		return fmt.Errorf("failed to encode navigation data: %v", err)
	}

	return nil
}

// GetMemoryUsage 获取构建过程的内存使用情况
func (nb *NavigationBuilder) GetMemoryUsage() BuildMemoryStats {
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

	if nb.pathfinder != nil && nb.pathfinder.graph != nil {
		stats.PathNodes = len(nb.pathfinder.graph.Nodes)
		stats.PathEdges = len(nb.pathfinder.graph.Edges)
	}

	return stats
}

// countOctreeNodes 统计octree节点数量
func (nb *NavigationBuilder) countOctreeNodes(node *OctreeNode) int {
	if node == nil {
		return 0
	}

	count := 1
	if !node.IsLeaf {
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
