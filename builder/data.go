package builder

import (
	"fmt"

	"github.com/o0olele/octree-go/geometry"
	"github.com/o0olele/octree-go/math32"
	"github.com/o0olele/octree-go/octree"
)

// 文件格式常量
const (
	NAVIGATION_FILE_MAGIC   = 0x4E415649 // "NAVI"
	NAVIGATION_FILE_VERSION = 1
)

// FileHeader 导航文件头
type FileHeader struct {
	Magic   uint32 // 文件魔数
	Version uint32 // 版本号
}

// CompactNode 紧凑的节点数据结构
type CompactNode struct {
	ID     int32          `json:"id"`
	Bounds geometry.AABB  `json:"bounds"`
	Center math32.Vector3 `json:"-"`
}

// CompactEdge 紧凑的边数据结构
type CompactEdge struct {
	NodeAID int32   `json:"node_a_id"`
	NodeBID int32   `json:"node_b_id"`
	Cost    float32 `json:"cost"`
}

// NavigationData 紧凑的导航数据结构，类似于Detour的数据格式
type NavigationData struct {
	// 基本信息
	Bounds   geometry.AABB `json:"bounds"`
	MaxDepth uint8         `json:"max_depth"`
	MinSize  float32       `json:"min_size"`
	StepSize float32       `json:"step_size"`

	// 节点和边数据
	Nodes []CompactNode `json:"nodes"`
	Edges []CompactEdge `json:"edges"`

	// 空间索引
	MortonIndex      []int32 `json:"morton_index"`
	MortonResolution uint32  `json:"morton_resolution"`

	// 几何体数据（用于碰撞检测）
	GeometryData []geometry.Triangle `json:"geometry_data"`

	// 邻接表索引（运行时构建，不序列化）
	adjacencyList map[int32][]int32            `json:"-"`
	edgeIndex     map[int32][]CompactEdge      `json:"-"`
	initialized   bool                         `json:"-"`
	octree        *octree.Octree               `json:"-"`
	spatialCache  *math32.Cache[uint64, int32] `json:"-"` // 空间查询优化 - 使用LRU缓存
}

func (nd *NavigationData) init() error {
	// 预先构建索引以提高查询性能
	nd.BuildIndexes()

	// 构建八叉树
	nd.octree = octree.NewOctree(nd.Bounds, nd.MaxDepth, nd.StepSize)
	nd.octree.SetTriangles(nd.GeometryData)
	nd.octree.Build()

	// 初始化空间缓存
	nd.spatialCache = math32.NewCache[uint64, int32](10000)
	return nil
}

func (nd *NavigationData) GetOctree() *octree.Octree {
	return nd.octree
}

// GetDataSize 计算导航数据的大小（字节）
func (nd *NavigationData) GetDataSize() int {
	size := 0

	// 基本信息
	size += 8 * 6 // bounds (6 float64)
	size += 4     // max_depth
	size += 8     // min_size
	size += 8     // step_size

	// 节点数据
	size += len(nd.Nodes) * (4 + 8*6) // id + center + bounds

	// 边数据
	size += len(nd.Edges) * (4 + 4 + 8) // nodeA + nodeB + cost

	// Morton索引
	size += len(nd.MortonIndex) * 4
	size += 4 // morton_resolution

	// 几何体数据
	size += len(nd.GeometryData)

	return size
}

// GetNodeCount 获取节点数量
func (nd *NavigationData) GetNodeCount() int {
	return len(nd.Nodes)
}

// GetEdgeCount 获取边数量
func (nd *NavigationData) GetEdgeCount() int {
	return len(nd.Edges)
}

func (nd *NavigationData) GetGeometryCount() int {
	return len(nd.GeometryData)
}

func (nd *NavigationData) GetGeometryData() []geometry.Triangle {
	return nd.GeometryData
}

func (nd *NavigationData) GetSpatialCacheCount() int {
	return nd.spatialCache.Len()
}

// FindNodeByID 根据ID查找节点
func (nd *NavigationData) FindNodeByID(id int) *CompactNode {
	if id < 0 || id >= len(nd.Nodes) {
		return nil
	}
	return &nd.Nodes[id]
}

// GetNodeEdges 获取节点的所有边
func (nd *NavigationData) GetNodeEdges(nodeID int32) []CompactEdge {
	// 确保邻接表和边索引已初始化
	if nd.edgeIndex == nil || !nd.initialized {
		nd.GetNeighbors(nodeID) // 这会初始化邻接表和边索引
	}

	return nd.edgeIndex[nodeID]
}

// GetNeighbors 获取节点的邻居节点ID
func (nd *NavigationData) GetNeighbors(nodeID int32) []int32 {
	if nd.adjacencyList == nil || !nd.initialized {
		// 懒加载邻接表和边索引
		nd.BuildIndexes()
	}

	return nd.adjacencyList[nodeID]
}

// BuildIndexes 预先构建邻接表和边索引
func (nd *NavigationData) BuildIndexes() {
	if nd.initialized {
		return
	}

	nd.adjacencyList = make(map[int32][]int32)
	nd.edgeIndex = make(map[int32][]CompactEdge)

	for _, edge := range nd.Edges {
		// 构建邻接表
		if _, exists := nd.adjacencyList[edge.NodeAID]; !exists {
			nd.adjacencyList[edge.NodeAID] = make([]int32, 0)
		}
		nd.adjacencyList[edge.NodeAID] = append(nd.adjacencyList[edge.NodeAID], edge.NodeBID)

		if _, exists := nd.adjacencyList[edge.NodeBID]; !exists {
			nd.adjacencyList[edge.NodeBID] = make([]int32, 0)
		}
		nd.adjacencyList[edge.NodeBID] = append(nd.adjacencyList[edge.NodeBID], edge.NodeAID)

		// 构建边索引
		if _, exists := nd.edgeIndex[edge.NodeAID]; !exists {
			nd.edgeIndex[edge.NodeAID] = make([]CompactEdge, 0)
		}
		nd.edgeIndex[edge.NodeAID] = append(nd.edgeIndex[edge.NodeAID], edge)

		if _, exists := nd.edgeIndex[edge.NodeBID]; !exists {
			nd.edgeIndex[edge.NodeBID] = make([]CompactEdge, 0)
		}
		nd.edgeIndex[edge.NodeBID] = append(nd.edgeIndex[edge.NodeBID], edge)
	}

	nd.initialized = true
}

// Validate 验证导航数据的完整性
func (nd *NavigationData) Validate() error {
	// 检查节点ID连续性
	for i, node := range nd.Nodes {
		nd.Nodes[i].Center = node.Bounds.Center()
		if node.ID != int32(i) {
			return fmt.Errorf("node ID mismatch at index %d: expected %d, got %d", i, i, node.ID)
		}
	}

	// 检查边的有效性
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

	// 检查Morton索引
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

// isPathClear 检查两点之间的路径是否畅通
func (nd *NavigationData) IsPathClear(agent *octree.Agent, start, end math32.Vector3) bool {
	return nd.octree.IsPathClear(agent, start, end)
}
