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
	Bounds    geometry.AABB   `json:"bounds"`
	MaxDepth  uint8           `json:"max_depth"`
	MinSize   float32         `json:"min_size"`
	StepSize  float32         `json:"step_size"`
	GridSize  math32.Vector3i `json:"grid_size"`
	VoxelSize float32         `json:"voxel_size"`

	// 节点和边数据
	Nodes []CompactNode `json:"nodes"`
	Edges []CompactEdge `json:"edges"`

	// 空间索引
	MortonIndex      []int32 `json:"morton_index"`
	MortonResolution uint32  `json:"morton_resolution"`

	// 几何体数据（用于碰撞检测）
	GeometryData []geometry.Triangle `json:"geometry_data"`
	// 体素数据 (用于碰撞检测)
	VoxelData []uint64 `json:"voxel_data"`

	// 邻接表索引（运行时构建，不序列化）
	// 压缩邻接（CSR）：连续邻接数组 + 偏移 + 度
	csrNeighbors []int32                 `json:"-"`
	csrOffsets   []int32                 `json:"-"` // len = 节点数
	csrDegrees   []int32                 `json:"-"` // len = 节点数
	edgeIndex    map[int32][]CompactEdge `json:"-"`

	// 预计算缓存（运行时，不序列化）
	nodeNeighborCounts    []int32        `json:"-"`
	nodeBoundaryDistances []float32      `json:"-"`
	edgePairToIndex       map[uint64]int `json:"-"`

	initialized  bool                         `json:"-"`
	octree       *octree.Octree               `json:"-"`
	spatialCache *math32.Cache[uint64, int32] `json:"-"` // 空间查询优化 - 使用LRU缓存
}

func (nd *NavigationData) init() error {
	// 预先构建索引以提高查询性能
	nd.BuildIndexes()

	// 构建八叉树
	if len(nd.GeometryData) > 0 {
		nd.octree = octree.NewOctree(nd.Bounds, nd.MaxDepth, nd.StepSize)
		nd.octree.SetTriangles(nd.GeometryData)
		nd.octree.Build()
	}

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
	// 确保索引已初始化
	if nd.edgeIndex == nil || !nd.initialized {
		nd.BuildIndexes()
	}
	return nd.edgeIndex[nodeID]
}

// GetNeighbors 获取节点的邻居节点ID
func (nd *NavigationData) GetNeighbors(nodeID int32) []int32 {
	if nd.csrNeighbors == nil || nd.csrOffsets == nil || nd.csrDegrees == nil || !nd.initialized {
		// 懒加载索引
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

// BuildIndexes 预先构建邻接表和边索引
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
	// 临时写指针
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

	// 构建边索引（保持 map 以简化调用方）
	nd.edgeIndex = make(map[int32][]CompactEdge)
	for _, e := range nd.Edges {
		nd.edgeIndex[e.NodeAID] = append(nd.edgeIndex[e.NodeAID], e)
		nd.edgeIndex[e.NodeBID] = append(nd.edgeIndex[e.NodeBID], e)
	}

	nd.csrNeighbors = neighbors
	nd.csrOffsets = offsets
	nd.csrDegrees = deg

	// 预计算：节点邻接度缓存
	nd.nodeNeighborCounts = make([]int32, n)
	copy(nd.nodeNeighborCounts, deg)

	// 预计算：节点到场景边界的最小距离
	nd.nodeBoundaryDistances = make([]float32, n)
	for i := 0; i < n; i++ {
		center := nd.Nodes[i].Bounds.Center()
		dx := float32(math32.Min(center.X-nd.Bounds.Min.X, nd.Bounds.Max.X-center.X))
		dy := float32(math32.Min(center.Y-nd.Bounds.Min.Y, nd.Bounds.Max.Y-center.Y))
		dz := float32(math32.Min(center.Z-nd.Bounds.Min.Z, nd.Bounds.Max.Z-center.Z))
		nd.nodeBoundaryDistances[i] = math32.Min(dx, math32.Min(dy, dz))
	}

	// 预计算：边对到索引的映射
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
	if len(nd.VoxelData) > 0 {
		return nd.IsPathClearByVoxel(agent, start, end)
	}
	return nd.octree.IsPathClear(agent, start, end)
}

// GetNodeNeighborCount 返回节点的邻居数量（缓存）
func (nd *NavigationData) GetNodeNeighborCount(nodeID int32) int32 {
	if nd.nodeNeighborCounts == nil || !nd.initialized {
		nd.BuildIndexes()
	}
	if nodeID < 0 || int(nodeID) >= len(nd.nodeNeighborCounts) {
		return 0
	}
	return nd.nodeNeighborCounts[nodeID]
}

// GetNodeBoundaryDistance 返回节点到边界的最小距离（缓存）
func (nd *NavigationData) GetNodeBoundaryDistance(nodeID int32) float32 {
	if nd.nodeBoundaryDistances == nil || !nd.initialized {
		nd.BuildIndexes()
	}
	if nodeID < 0 || int(nodeID) >= len(nd.nodeBoundaryDistances) {
		return 0
	}
	return nd.nodeBoundaryDistances[nodeID]
}

// FindEdgeIndex 返回边索引
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

// GetEdgeCostByNodes 通过节点ID获取边成本
func (nd *NavigationData) GetEdgeCostByNodes(nodeAID, nodeBID int32) (float32, bool) {
	idx := nd.FindEdgeIndex(nodeAID, nodeBID)
	if idx < 0 {
		return 0, false
	}
	return nd.Edges[idx].Cost, true
}

func (nd *NavigationData) IsWorldOccupiedByVoxel(agent *octree.Agent, worldPos math32.Vector3) bool {
	if len(nd.VoxelData) == 0 {
		return false
	}
	localPos := worldPos.Sub(nd.Bounds.Min)
	coord := math32.Vector3i{
		X: int32(localPos.X / agent.Radius),
		Y: int32(localPos.Y / agent.Radius),
		Z: int32(localPos.Z / agent.Radius),
	}
	bitmap := math32.Bitmap(nd.VoxelData)
	return bitmap.Contains(uint32(coord.Z*nd.GridSize.X*nd.GridSize.Y + coord.Y*nd.GridSize.X + coord.X))
}

func (nd *NavigationData) IsPathClearByVoxel(agent *octree.Agent, start, end math32.Vector3) bool {
	// 计算方向向量和距离
	direction := end.Sub(start)
	distance := direction.Length()

	if distance < 0.001 { // 距离太近，认为是同一点
		return true
	}

	// 标准化方向向量
	direction = direction.Scale(1.0 / distance)

	agentRadius := float32(0.4)
	if agent != nil {
		agentRadius = agent.Radius
	}
	// 使用适当的步长进行采样检查
	stepSize := math32.Max(0.1, agentRadius*0.6)
	steps := math32.CeilToInt(distance / stepSize)

	// 沿着路径进行采样检测
	for i := 0; i <= steps; i++ {
		t := float32(i) / float32(steps)
		samplePoint := start.Add(direction.Scale(distance * t))

		// 如果有Agent半径，还需要检查Agent碰撞
		occupied := nd.IsWorldOccupiedByVoxel(agent, samplePoint)
		if occupied {
			return false
		}

	}

	return true
}
