package octree

import (
	"github.com/o0olele/octree-go/geometry"
	"github.com/o0olele/octree-go/math32"
)

// OptimizedNavigationData 优化的导航数据结构，进一步减小文件大小
type OptimizedNavigationData struct {
	// 基本信息
	Bounds   geometry.AABB `json:"bounds"`
	MaxDepth int           `json:"max_depth"`
	MinSize  float32       `json:"min_size"`
	StepSize float32       `json:"step_size"`

	// 量化的节点数据
	QuantizedNodes []QuantizedNode `json:"quantized_nodes"`

	// 压缩的边数据
	CompressedEdges []CompressedEdge `json:"compressed_edges"`

	// 压缩的Morton索引
	MortonIndex      []uint16 `json:"morton_index"` // 使用16位索引
	MortonResolution uint32   `json:"morton_resolution"`

	// 压缩的几何体数据
	CompressedGeometryData []byte `json:"compressed_geometry_data"`

	// 量化参数
	QuantizationScale  math32.Vector3 `json:"quantization_scale"`
	QuantizationOffset math32.Vector3 `json:"quantization_offset"`
}

// QuantizedNode 量化的节点数据（使用16位整数存储坐标）
type QuantizedNode struct {
	CenterX uint16 `json:"center_x"`
	CenterY uint16 `json:"center_y"`
	CenterZ uint16 `json:"center_z"`
	SizeX   uint16 `json:"size_x"`
	SizeY   uint16 `json:"size_y"`
	SizeZ   uint16 `json:"size_z"`
}

// CompressedEdge 压缩的边数据（使用16位节点索引）
type CompressedEdge struct {
	NodeAID uint16 `json:"node_a_id"`
	NodeBID uint16 `json:"node_b_id"`
	Cost    uint16 `json:"cost"` // 量化的成本值
}

// OptimizeNavigationData 优化导航数据，减小文件大小
func OptimizeNavigationData(navData *NavigationData) *OptimizedNavigationData {
	// 计算量化参数
	scale, offset := calculateQuantizationParams(navData.Bounds)

	// 量化节点数据
	quantizedNodes := make([]QuantizedNode, len(navData.Nodes))
	for i, node := range navData.Nodes {
		quantizedNodes[i] = quantizeNode(node, scale, offset)
	}

	// 压缩边数据
	compressedEdges := make([]CompressedEdge, len(navData.Edges))
	maxCost := findMaxCost(navData.Edges)

	for i, edge := range navData.Edges {
		compressedEdges[i] = CompressedEdge{
			NodeAID: uint16(edge.NodeAID),
			NodeBID: uint16(edge.NodeBID),
			Cost:    uint16((edge.Cost / maxCost) * 65535), // 量化到16位
		}
	}

	// 压缩Morton索引
	mortonIndex := make([]uint16, len(navData.MortonIndex))
	for i, idx := range navData.MortonIndex {
		mortonIndex[i] = uint16(idx)
	}

	return &OptimizedNavigationData{
		Bounds:                 navData.Bounds,
		MaxDepth:               navData.MaxDepth,
		MinSize:                navData.MinSize,
		StepSize:               navData.StepSize,
		QuantizedNodes:         quantizedNodes,
		CompressedEdges:        compressedEdges,
		MortonIndex:            mortonIndex,
		MortonResolution:       navData.MortonResolution,
		CompressedGeometryData: compressGeometryData(navData.GeometryData),
		QuantizationScale:      scale,
		QuantizationOffset:     offset,
	}
}

// DeoptimizeNavigationData 将优化的数据转换回标准格式
func DeoptimizeNavigationData(optimized *OptimizedNavigationData) *NavigationData {
	// 反量化节点数据
	nodes := make([]CompactNode, len(optimized.QuantizedNodes))
	for i, qNode := range optimized.QuantizedNodes {
		nodes[i] = dequantizeNode(qNode, i, optimized.QuantizationScale, optimized.QuantizationOffset)
	}

	// 解压缩边数据
	edges := make([]CompactEdge, len(optimized.CompressedEdges))
	maxCost := findMaxCostFromBounds(optimized.Bounds) // 估算最大成本

	for i, cEdge := range optimized.CompressedEdges {
		edges[i] = CompactEdge{
			NodeAID: int(cEdge.NodeAID),
			NodeBID: int(cEdge.NodeBID),
			Cost:    (float32(cEdge.Cost) / 65535) * maxCost,
		}
	}

	// 解压缩Morton索引
	mortonIndex := make([]int, len(optimized.MortonIndex))
	for i, idx := range optimized.MortonIndex {
		mortonIndex[i] = int(idx)
	}

	return &NavigationData{
		Bounds:           optimized.Bounds,
		MaxDepth:         optimized.MaxDepth,
		MinSize:          optimized.MinSize,
		StepSize:         optimized.StepSize,
		Nodes:            nodes,
		Edges:            edges,
		MortonIndex:      mortonIndex,
		MortonResolution: optimized.MortonResolution,
		GeometryData:     decompressGeometryData(optimized.CompressedGeometryData),
	}
}

// calculateQuantizationParams 计算量化参数
func calculateQuantizationParams(bounds geometry.AABB) (math32.Vector3, math32.Vector3) {
	size := bounds.Size()
	scale := math32.Vector3{
		X: 65535.0 / size.X,
		Y: 65535.0 / size.Y,
		Z: 65535.0 / size.Z,
	}
	offset := bounds.Min
	return scale, offset
}

// quantizeNode 量化节点数据
func quantizeNode(node CompactNode, scale, offset math32.Vector3) QuantizedNode {
	center := node.Center.Sub(offset)
	size := node.Bounds.Size()

	return QuantizedNode{
		CenterX: uint16(center.X * scale.X),
		CenterY: uint16(center.Y * scale.Y),
		CenterZ: uint16(center.Z * scale.Z),
		SizeX:   uint16(size.X * scale.X),
		SizeY:   uint16(size.Y * scale.Y),
		SizeZ:   uint16(size.Z * scale.Z),
	}
}

// dequantizeNode 反量化节点数据
func dequantizeNode(qNode QuantizedNode, id int, scale, offset math32.Vector3) CompactNode {
	center := math32.Vector3{
		X: float32(qNode.CenterX)/scale.X + offset.X,
		Y: float32(qNode.CenterY)/scale.Y + offset.Y,
		Z: float32(qNode.CenterZ)/scale.Z + offset.Z,
	}

	size := math32.Vector3{
		X: float32(qNode.SizeX) / scale.X,
		Y: float32(qNode.SizeY) / scale.Y,
		Z: float32(qNode.SizeZ) / scale.Z,
	}

	bounds := geometry.AABB{
		Min: center.Sub(size.Scale(0.5)),
		Max: center.Add(size.Scale(0.5)),
	}

	return CompactNode{
		ID:     id,
		Center: center,
		Bounds: bounds,
	}
}

// findMaxCost 找到最大成本值
func findMaxCost(edges []CompactEdge) float32 {
	maxCost := float32(0.0)
	for _, edge := range edges {
		if edge.Cost > maxCost {
			maxCost = edge.Cost
		}
	}
	return maxCost
}

// findMaxCostFromBounds 从边界估算最大成本
func findMaxCostFromBounds(bounds geometry.AABB) float32 {
	// 估算最大可能的距离（对角线长度）
	diagonal := bounds.Size()
	return math32.Sqrt(diagonal.X*diagonal.X + diagonal.Y*diagonal.Y + diagonal.Z*diagonal.Z)
}

// compressGeometryData 压缩几何体数据
func compressGeometryData(data []byte) []byte {
	// 这里可以实现更高级的压缩算法
	// 目前返回原始数据
	return data
}

// decompressGeometryData 解压缩几何体数据
func decompressGeometryData(data []byte) []byte {
	// 这里可以实现解压缩算法
	// 目前返回原始数据
	return data
}

// GetOptimizedDataSize 计算优化后的数据大小
func (opt *OptimizedNavigationData) GetOptimizedDataSize() int {
	size := 0

	// 基本信息
	size += 8 * 6 // bounds
	size += 4     // max_depth
	size += 8     // min_size
	size += 8     // step_size

	// 量化节点数据（每个节点12字节）
	size += len(opt.QuantizedNodes) * 12

	// 压缩边数据（每条边6字节）
	size += len(opt.CompressedEdges) * 6

	// Morton索引（每个索引2字节）
	size += len(opt.MortonIndex) * 2
	size += 4 // morton_resolution

	// 几何体数据
	size += len(opt.CompressedGeometryData)

	// 量化参数
	size += 8 * 6 // scale + offset

	return size
}

// GetCompressionRatio 计算压缩比
func GetCompressionRatio(original *NavigationData, optimized *OptimizedNavigationData) float32 {
	originalSize := original.GetDataSize()
	optimizedSize := optimized.GetOptimizedDataSize()

	if originalSize == 0 {
		return 0
	}

	return float32(optimizedSize) / float32(originalSize)
}

// AnalyzeOptimization 分析优化效果
func AnalyzeOptimization(original *NavigationData, optimized *OptimizedNavigationData) OptimizationAnalysis {
	originalSize := original.GetDataSize()
	optimizedSize := optimized.GetOptimizedDataSize()

	return OptimizationAnalysis{
		OriginalSize:     originalSize,
		OptimizedSize:    optimizedSize,
		CompressionRatio: GetCompressionRatio(original, optimized),
		SpaceSaved:       originalSize - optimizedSize,
		NodeCount:        len(original.Nodes),
		EdgeCount:        len(original.Edges),
		QuantizationLoss: calculateQuantizationLoss(original, optimized),
	}
}

// OptimizationAnalysis 优化分析结果
type OptimizationAnalysis struct {
	OriginalSize     int     `json:"original_size"`
	OptimizedSize    int     `json:"optimized_size"`
	CompressionRatio float32 `json:"compression_ratio"`
	SpaceSaved       int     `json:"space_saved"`
	NodeCount        int     `json:"node_count"`
	EdgeCount        int     `json:"edge_count"`
	QuantizationLoss float32 `json:"quantization_loss"`
}

// calculateQuantizationLoss 计算量化损失
func calculateQuantizationLoss(original *NavigationData, optimized *OptimizedNavigationData) float32 {
	// 计算量化前后节点位置的平均误差
	if len(original.Nodes) == 0 {
		return 0
	}

	deoptimized := DeoptimizeNavigationData(optimized)
	totalError := float32(0.0)

	for i, originalNode := range original.Nodes {
		if i < len(deoptimized.Nodes) {
			deoptimizedNode := deoptimized.Nodes[i]
			error := originalNode.Center.Distance(deoptimizedNode.Center)
			totalError += error
		}
	}

	return totalError / float32(len(original.Nodes))
}
