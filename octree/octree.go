package octree

import (
	"encoding/json"
	"math"
)

// Vector3 表示3D向量
type Vector3 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

func (v Vector3) Add(other Vector3) Vector3 {
	return Vector3{v.X + other.X, v.Y + other.Y, v.Z + other.Z}
}

func (v Vector3) Sub(other Vector3) Vector3 {
	return Vector3{v.X - other.X, v.Y - other.Y, v.Z - other.Z}
}

func (v Vector3) Scale(s float64) Vector3 {
	return Vector3{v.X * s, v.Y * s, v.Z * s}
}

func (v Vector3) Distance(other Vector3) float64 {
	diff := v.Sub(other)
	return math.Sqrt(diff.X*diff.X + diff.Y*diff.Y + diff.Z*diff.Z)
}

func (v Vector3) Length() float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
}

// AABB 轴对齐包围盒
type AABB struct {
	Min Vector3 `json:"min"`
	Max Vector3 `json:"max"`
}

func (aabb AABB) Contains(point Vector3) bool {
	return point.X >= aabb.Min.X && point.X <= aabb.Max.X &&
		point.Y >= aabb.Min.Y && point.Y <= aabb.Max.Y &&
		point.Z >= aabb.Min.Z && point.Z <= aabb.Max.Z
}

func (aabb AABB) Center() Vector3 {
	return Vector3{
		(aabb.Min.X + aabb.Max.X) / 2,
		(aabb.Min.Y + aabb.Max.Y) / 2,
		(aabb.Min.Z + aabb.Max.Z) / 2,
	}
}

func (aabb AABB) Size() Vector3 {
	return aabb.Max.Sub(aabb.Min)
}

// Geometry 接口定义
type Geometry interface {
	GetBounds() AABB
	IntersectsAABB(aabb AABB) bool
	ContainsPoint(point Vector3) bool
	GetType() string
}

// Triangle 三角形几何体
type Triangle struct {
	A Vector3 `json:"a"`
	B Vector3 `json:"b"`
	C Vector3 `json:"c"`
}

func (t Triangle) GetBounds() AABB {
	minX := math.Min(math.Min(t.A.X, t.B.X), t.C.X)
	maxX := math.Max(math.Max(t.A.X, t.B.X), t.C.X)
	minY := math.Min(math.Min(t.A.Y, t.B.Y), t.C.Y)
	maxY := math.Max(math.Max(t.A.Y, t.B.Y), t.C.Y)
	minZ := math.Min(math.Min(t.A.Z, t.B.Z), t.C.Z)
	maxZ := math.Max(math.Max(t.A.Z, t.B.Z), t.C.Z)
	return AABB{Vector3{minX, minY, minZ}, Vector3{maxX, maxY, maxZ}}
}

func (t Triangle) IntersectsAABB(aabb AABB) bool {
	// 简化的三角形-AABB相交测试
	bounds := t.GetBounds()
	return !(bounds.Max.X < aabb.Min.X || bounds.Min.X > aabb.Max.X ||
		bounds.Max.Y < aabb.Min.Y || bounds.Min.Y > aabb.Max.Y ||
		bounds.Max.Z < aabb.Min.Z || bounds.Min.Z > aabb.Max.Z)
}

func (t Triangle) ContainsPoint(point Vector3) bool {
	// 简化的点在三角形内测试
	bounds := t.GetBounds()
	return bounds.Contains(point)
}

func (t Triangle) GetType() string {
	return "triangle"
}

// Box 立方体几何体
type Box struct {
	Center Vector3 `json:"center"`
	Size   Vector3 `json:"size"`
}

func (b Box) GetBounds() AABB {
	halfSize := b.Size.Scale(0.5)
	return AABB{
		b.Center.Sub(halfSize),
		b.Center.Add(halfSize),
	}
}

func (b Box) IntersectsAABB(aabb AABB) bool {
	bounds := b.GetBounds()
	return !(bounds.Max.X < aabb.Min.X || bounds.Min.X > aabb.Max.X ||
		bounds.Max.Y < aabb.Min.Y || bounds.Min.Y > aabb.Max.Y ||
		bounds.Max.Z < aabb.Min.Z || bounds.Min.Z > aabb.Max.Z)
}

func (b Box) ContainsPoint(point Vector3) bool {
	bounds := b.GetBounds()
	return bounds.Contains(point)
}

func (b Box) GetType() string {
	return "box"
}

// Capsule 胶囊体几何体
type Capsule struct {
	Start  Vector3 `json:"start"`
	End    Vector3 `json:"end"`
	Radius float64 `json:"radius"`
}

func (c Capsule) GetBounds() AABB {
	minX := math.Min(c.Start.X, c.End.X) - c.Radius
	maxX := math.Max(c.Start.X, c.End.X) + c.Radius
	minY := math.Min(c.Start.Y, c.End.Y) - c.Radius
	maxY := math.Max(c.Start.Y, c.End.Y) + c.Radius
	minZ := math.Min(c.Start.Z, c.End.Z) - c.Radius
	maxZ := math.Max(c.Start.Z, c.End.Z) + c.Radius
	return AABB{Vector3{minX, minY, minZ}, Vector3{maxX, maxY, maxZ}}
}

func (c Capsule) IntersectsAABB(aabb AABB) bool {
	bounds := c.GetBounds()
	return !(bounds.Max.X < aabb.Min.X || bounds.Min.X > aabb.Max.X ||
		bounds.Max.Y < aabb.Min.Y || bounds.Min.Y > aabb.Max.Y ||
		bounds.Max.Z < aabb.Min.Z || bounds.Min.Z > aabb.Max.Z)
}

func (c Capsule) ContainsPoint(point Vector3) bool {
	// 计算点到胶囊轴线的距离
	axis := c.End.Sub(c.Start)
	toPoint := point.Sub(c.Start)

	if axis.Length() == 0 {
		return toPoint.Length() <= c.Radius
	}

	t := math.Max(0, math.Min(1, dot(toPoint, axis)/dot(axis, axis)))
	closest := c.Start.Add(axis.Scale(t))
	return point.Distance(closest) <= c.Radius
}

func (c Capsule) GetType() string {
	return "capsule"
}

func dot(a, b Vector3) float64 {
	return a.X*b.X + a.Y*b.Y + a.Z*b.Z
}

// OctreeNode 八叉树节点
type OctreeNode struct {
	Bounds     AABB           `json:"bounds"`
	Children   [8]*OctreeNode `json:"children,omitempty"`
	Geometries []Geometry     `json:"-"`
	IsLeaf     bool           `json:"is_leaf"`
	IsOccupied bool           `json:"is_occupied"`
	Depth      int            `json:"depth"`
}

// Octree 八叉树主结构
type Octree struct {
	Root       *OctreeNode `json:"root"`
	MaxDepth   int         `json:"max_depth"`
	MinSize    float64     `json:"min_size"`
	geometries []Geometry
}

// NewOctree 创建新的八叉树
func NewOctree(bounds AABB, maxDepth int, minSize float64) *Octree {
	return &Octree{
		Root: &OctreeNode{
			Bounds:     bounds,
			IsLeaf:     true,
			IsOccupied: false,
			Depth:      0,
		},
		MaxDepth:   maxDepth,
		MinSize:    minSize,
		geometries: make([]Geometry, 0),
	}
}

// AddGeometry 添加几何体到八叉树
func (o *Octree) AddGeometry(geom Geometry) {
	o.geometries = append(o.geometries, geom)
	o.insertGeometry(o.Root, geom)
}

func (o *Octree) insertGeometry(node *OctreeNode, geom Geometry) {
	if !geom.IntersectsAABB(node.Bounds) {
		return
	}

	node.Geometries = append(node.Geometries, geom)
	node.IsOccupied = true

	// 如果节点是叶子且达到分割条件，则分割
	if node.IsLeaf && node.Depth < o.MaxDepth && node.Bounds.Size().X > o.MinSize {
		o.subdivide(node)
	}

	// 如果不是叶子，递归插入到子节点
	if !node.IsLeaf {
		for _, child := range node.Children {
			if child != nil {
				o.insertGeometry(child, geom)
			}
		}
	}
}

func (o *Octree) subdivide(node *OctreeNode) {
	center := node.Bounds.Center()
	size := node.Bounds.Size().Scale(0.5)

	positions := [8]Vector3{
		{node.Bounds.Min.X, node.Bounds.Min.Y, node.Bounds.Min.Z}, // 000
		{center.X, node.Bounds.Min.Y, node.Bounds.Min.Z},          // 100
		{node.Bounds.Min.X, center.Y, node.Bounds.Min.Z},          // 010
		{center.X, center.Y, node.Bounds.Min.Z},                   // 110
		{node.Bounds.Min.X, node.Bounds.Min.Y, center.Z},          // 001
		{center.X, node.Bounds.Min.Y, center.Z},                   // 101
		{node.Bounds.Min.X, center.Y, center.Z},                   // 011
		{center.X, center.Y, center.Z},                            // 111
	}

	for i := 0; i < 8; i++ {
		childBounds := AABB{
			positions[i],
			positions[i].Add(size),
		}

		node.Children[i] = &OctreeNode{
			Bounds:     childBounds,
			IsLeaf:     true,
			IsOccupied: false,
			Depth:      node.Depth + 1,
		}
	}

	node.IsLeaf = false
}

// Build 构建八叉树
func (o *Octree) Build() {
	for _, geom := range o.geometries {
		o.insertGeometry(o.Root, geom)
	}
}

// IsOccupied 检查指定位置是否被占用
func (o *Octree) IsOccupied(point Vector3) bool {
	return o.isOccupiedRecursive(o.Root, point)
}

func (o *Octree) isOccupiedRecursive(node *OctreeNode, point Vector3) bool {
	if !node.Bounds.Contains(point) {
		return false
	}

	if node.IsLeaf {
		if !node.IsOccupied {
			return false
		}
		// 检查所有几何体
		for _, geom := range node.Geometries {
			if geom.ContainsPoint(point) {
				return true
			}
		}
		return false
	}

	// 递归检查子节点
	for _, child := range node.Children {
		if child != nil && o.isOccupiedRecursive(child, point) {
			return true
		}
	}

	return false
}

// 用于JSON序列化的简化结构
type OctreeExport struct {
	Root     *OctreeNodeExport `json:"root"`
	MaxDepth int               `json:"max_depth"`
	MinSize  float64           `json:"min_size"`
}

type OctreeNodeExport struct {
	Bounds     AABB                `json:"bounds"`
	Children   []*OctreeNodeExport `json:"children,omitempty"`
	IsLeaf     bool                `json:"is_leaf"`
	IsOccupied bool                `json:"is_occupied"`
	Depth      int                 `json:"depth"`
}

// ToJSON 导出八叉树为JSON
func (o *Octree) ToJSON() ([]byte, error) {
	export := &OctreeExport{
		Root:     o.nodeToExport(o.Root),
		MaxDepth: o.MaxDepth,
		MinSize:  o.MinSize,
	}
	return json.Marshal(export)
}

func (o *Octree) nodeToExport(node *OctreeNode) *OctreeNodeExport {
	if node == nil {
		return nil
	}

	export := &OctreeNodeExport{
		Bounds:     node.Bounds,
		IsLeaf:     node.IsLeaf,
		IsOccupied: node.IsOccupied,
		Depth:      node.Depth,
	}

	if !node.IsLeaf {
		for _, child := range node.Children {
			export.Children = append(export.Children, o.nodeToExport(child))
		}
	}

	return export
}
