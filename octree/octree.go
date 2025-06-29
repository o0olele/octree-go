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

func (v Vector3) Mul(s float64) Vector3 {
	return Vector3{v.X * s, v.Y * s, v.Z * s}
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

func (aabb AABB) Intersects(other AABB) bool {
	return aabb.Min.X <= other.Max.X && aabb.Max.X >= other.Min.X &&
		aabb.Min.Y <= other.Max.Y && aabb.Max.Y >= other.Min.Y &&
		aabb.Min.Z <= other.Max.Z && aabb.Max.Z >= other.Min.Z
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
	// 首先进行快速包围盒检测
	bounds := t.GetBounds()
	if !bounds.Intersects(aabb) {
		return false
	}

	// 对于简单情况，如果三角形完全在AABB内，直接返回true
	if aabb.Contains(t.A) && aabb.Contains(t.B) && aabb.Contains(t.C) {
		return true
	}

	// 使用Separating Axis Theorem (SAT) 进行精确检测
	// 将AABB转换为中心点和半尺寸
	center := aabb.Center()
	halfSize := aabb.Size().Scale(0.5)

	// 将三角形顶点转换为相对于AABB中心的坐标
	v0 := t.A.Sub(center)
	v1 := t.B.Sub(center)
	v2 := t.C.Sub(center)

	// 计算三角形的边向量
	f0 := v1.Sub(v0) // edge 0
	f1 := v2.Sub(v1) // edge 1
	f2 := v0.Sub(v2) // edge 2

	// 测试三角形的法向量
	normal := cross(f0, f1)
	if normal.Length() > 1e-10 {
		if !t.testSeparatingAxis(normal, v0, v1, v2, halfSize) {
			return false
		}
	}

	// 测试AABB的3个面法向量
	aabbAxes := []Vector3{
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
	}

	for _, axis := range aabbAxes {
		if !t.testSeparatingAxis(axis, v0, v1, v2, halfSize) {
			return false
		}
	}

	// 测试9个轴（3个AABB面法向量 × 3个三角形边向量的叉积）
	crossAxes := []Vector3{
		// 轴 u0 x f0, u0 x f1, u0 x f2
		{0, -f0.Z, f0.Y},
		{0, -f1.Z, f1.Y},
		{0, -f2.Z, f2.Y},
		// 轴 u1 x f0, u1 x f1, u1 x f2
		{f0.Z, 0, -f0.X},
		{f1.Z, 0, -f1.X},
		{f2.Z, 0, -f2.X},
		// 轴 u2 x f0, u2 x f1, u2 x f2
		{-f0.Y, f0.X, 0},
		{-f1.Y, f1.X, 0},
		{-f2.Y, f2.X, 0},
	}

	for _, axis := range crossAxes {
		// 跳过零向量
		if axis.Length() < 1e-10 {
			continue
		}
		if !t.testSeparatingAxis(axis, v0, v1, v2, halfSize) {
			return false
		}
	}

	return true
}

// testSeparatingAxis 测试给定轴上的投影是否分离
func (t Triangle) testSeparatingAxis(axis Vector3, v0, v1, v2, halfSize Vector3) bool {
	// 计算三角形顶点在轴上的投影
	p0 := dot(v0, axis)
	p1 := dot(v1, axis)
	p2 := dot(v2, axis)

	// 找到三角形投影的最小值和最大值
	triMin := math.Min(math.Min(p0, p1), p2)
	triMax := math.Max(math.Max(p0, p1), p2)

	// 计算AABB在轴上的投影半径
	r := math.Abs(halfSize.X*axis.X) + math.Abs(halfSize.Y*axis.Y) + math.Abs(halfSize.Z*axis.Z)

	// 检查是否分离
	return !(triMax < -r || triMin > r)
}

func (t Triangle) ContainsPoint(point Vector3) bool {
	bounds := t.GetBounds()
	return bounds.Contains(point)

	// 首先检查点是否在三角形所在的平面上
	if !t.isPointOnTrianglePlane(point) {
		return false
	}

	// 使用重心坐标检测点是否在三角形内
	return t.isPointInTriangle(point)
}

// isPointOnTrianglePlane 检查点是否在三角形所在的平面上（允许一定的误差）
func (t Triangle) isPointOnTrianglePlane(point Vector3) bool {
	// 计算三角形的法向量
	edge1 := t.B.Sub(t.A)
	edge2 := t.C.Sub(t.A)
	normal := cross(edge1, edge2)

	// 如果三角形退化（面积为0），使用包围盒检测
	if normal.Length() < 1e-10 {
		bounds := t.GetBounds()
		return bounds.Contains(point)
	}

	// 标准化法向量
	normal = normal.Scale(1.0 / normal.Length())

	// 计算点到平面的距离
	toPoint := point.Sub(t.A)
	distance := math.Abs(dot(toPoint, normal))

	// 允许小的误差（例如，由于浮点精度问题）
	const tolerance = 1e-6
	return distance < tolerance
}

// isPointInTriangle 使用重心坐标检测点是否在三角形内
func (t Triangle) isPointInTriangle(point Vector3) bool {
	// 计算向量
	v0 := t.C.Sub(t.A)
	v1 := t.B.Sub(t.A)
	v2 := point.Sub(t.A)

	// 计算点积
	dot00 := dot(v0, v0)
	dot01 := dot(v0, v1)
	dot02 := dot(v0, v2)
	dot11 := dot(v1, v1)
	dot12 := dot(v1, v2)

	// 计算重心坐标
	denom := dot00*dot11 - dot01*dot01
	if math.Abs(denom) < 1e-10 {
		return false // 退化三角形
	}

	invDenom := 1.0 / denom
	u := (dot11*dot02 - dot01*dot12) * invDenom
	v := (dot00*dot12 - dot01*dot02) * invDenom

	// 检查点是否在三角形内
	return (u >= 0) && (v >= 0) && (u+v <= 1)
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

// ConvexMesh 凸包网格几何体
type ConvexMesh struct {
	Vertices []Vector3 `json:"vertices"` // 凸包的顶点
	Faces    [][]int   `json:"faces"`    // 面，每个面是顶点索引的数组
}

func (cm ConvexMesh) GetBounds() AABB {
	if len(cm.Vertices) == 0 {
		return AABB{Vector3{0, 0, 0}, Vector3{0, 0, 0}}
	}

	min := cm.Vertices[0]
	max := cm.Vertices[0]

	for _, vertex := range cm.Vertices {
		if vertex.X < min.X {
			min.X = vertex.X
		}
		if vertex.Y < min.Y {
			min.Y = vertex.Y
		}
		if vertex.Z < min.Z {
			min.Z = vertex.Z
		}
		if vertex.X > max.X {
			max.X = vertex.X
		}
		if vertex.Y > max.Y {
			max.Y = vertex.Y
		}
		if vertex.Z > max.Z {
			max.Z = vertex.Z
		}
	}

	return AABB{min, max}
}

func (cm ConvexMesh) IntersectsAABB(aabb AABB) bool {
	bounds := cm.GetBounds()
	return bounds.Intersects(aabb)
}

func (cm ConvexMesh) ContainsPoint(point Vector3) bool {
	// 使用分离轴定理(SAT)的简化版本来检测点是否在凸包内
	// 对于凸包，如果点在所有面的内侧，则点在凸包内
	for _, face := range cm.Faces {
		if len(face) < 3 {
			continue // 跳过无效面
		}

		// 获取面的三个顶点来计算法向量
		v0 := cm.Vertices[face[0]]
		v1 := cm.Vertices[face[1]]
		v2 := cm.Vertices[face[2]]

		// 计算面的法向量（假设顶点按逆时针顺序排列）
		edge1 := v1.Sub(v0)
		edge2 := v2.Sub(v0)
		normal := cross(edge1, edge2)

		// 标准化法向量
		length := normal.Length()
		if length == 0 {
			continue // 退化面
		}
		normal = normal.Scale(1.0 / length)

		// 计算点到面的距离
		toPoint := point.Sub(v0)
		distance := dot(toPoint, normal)

		// 如果点在面的外侧（距离为正），则点不在凸包内
		if distance > 0 {
			return false
		}
	}

	return true
}

func (cm ConvexMesh) GetType() string {
	return "convex_mesh"
}

// cross 计算两个向量的叉积
func cross(a, b Vector3) Vector3 {
	return Vector3{
		a.Y*b.Z - a.Z*b.Y,
		a.Z*b.X - a.X*b.Z,
		a.X*b.Y - a.Y*b.X,
	}
}

func dot(a, b Vector3) float64 {
	return a.X*b.X + a.Y*b.Y + a.Z*b.Z
}

// Agent 表示寻路的智能体，用胶囊体表示
type Agent struct {
	Radius float64 `json:"radius"` // 胶囊体半径
	Height float64 `json:"height"` // 胶囊体高度（不包括两端的半球）
}

// NewAgent 创建新的Agent
func NewAgent(radius, height float64) *Agent {
	return &Agent{
		Radius: radius,
		Height: height,
	}
}

// GetCapsule 根据位置获取Agent的胶囊体表示
func (a *Agent) GetCapsule(position Vector3) Capsule {
	// 胶囊体的轴线沿Y轴方向
	halfHeight := a.Height / 2
	start := Vector3{position.X, position.Y - halfHeight, position.Z}
	end := Vector3{position.X, position.Y + halfHeight, position.Z}
	return Capsule{
		Start:  start,
		End:    end,
		Radius: a.Radius,
	}
}

func GetCapsuleAABB(position Vector3, radius float64, height float64) AABB {
	halfHeight := height / 2
	start := Vector3{position.X, position.Y - halfHeight, position.Z}
	end := Vector3{position.X, position.Y + halfHeight, position.Z}
	return AABB{
		Min: Vector3{start.X - radius, start.Y - radius, start.Z - radius},
		Max: Vector3{end.X + radius, end.Y + radius, end.Z + radius},
	}
}

// GetBounds 获取Agent在指定位置的包围盒
func (a *Agent) GetBounds(position Vector3) AABB {
	return a.GetCapsule(position).GetBounds()
}

// 改进的点到线段距离计算
func pointToLineSegmentDistance(point, lineStart, lineEnd Vector3) float64 {
	lineVec := lineEnd.Sub(lineStart)
	pointVec := point.Sub(lineStart)

	lineLength := lineVec.Length()
	if lineLength == 0 {
		return point.Distance(lineStart)
	}

	// 计算投影参数t
	t := dot(pointVec, lineVec) / dot(lineVec, lineVec)
	t = math.Max(0, math.Min(1, t))

	// 计算线段上最近点
	closestPoint := lineStart.Add(lineVec.Scale(t))
	return point.Distance(closestPoint)
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

// IsAgentOccupied 检查Agent在指定位置是否与几何体碰撞
func (o *Octree) IsAgentOccupied(agent *Agent, position Vector3) bool {
	if agent == nil {
		return o.IsOccupied(position)
	}

	agentCapsule := agent.GetCapsule(position)
	return o.isAgentOccupiedRecursive(o.Root, agentCapsule)
}

func (o *Octree) isAgentOccupiedRecursive(node *OctreeNode, agentCapsule Capsule) bool {
	// 检查Agent的包围盒是否与节点相交
	agentBounds := agentCapsule.GetBounds()
	if !o.aabbIntersects(node.Bounds, agentBounds) {
		return false
	}

	if node.IsLeaf {
		if !node.IsOccupied {
			return false
		}
		// 检查Agent胶囊体是否与节点中的几何体碰撞
		for _, geom := range node.Geometries {
			if o.capsuleIntersectsGeometry(agentCapsule, geom) {
				return true
			}
		}
		return false
	}

	// 递归检查子节点
	for _, child := range node.Children {
		if child != nil && o.isAgentOccupiedRecursive(child, agentCapsule) {
			return true
		}
	}

	return false
}

// aabbIntersects 检查两个AABB是否相交
func (o *Octree) aabbIntersects(a, b AABB) bool {
	return !(a.Max.X < b.Min.X || a.Min.X > b.Max.X ||
		a.Max.Y < b.Min.Y || a.Min.Y > b.Max.Y ||
		a.Max.Z < b.Min.Z || a.Min.Z > b.Max.Z)
}

// capsuleIntersectsGeometry 检查胶囊体是否与几何体相交
func (o *Octree) capsuleIntersectsGeometry(capsule Capsule, geom Geometry) bool {
	switch g := geom.(type) {
	case Triangle:
		return o.capsuleIntersectsTriangle(capsule, g)
	case Box:
		return o.capsuleIntersectsBox(capsule, g)
	case Capsule:
		return o.capsuleIntersectsCapsule(capsule, g)
	case ConvexMesh:
		return o.capsuleIntersectsConvexMesh(capsule, g)
	default:
		// 使用简化的包围盒检测
		return o.aabbIntersects(capsule.GetBounds(), geom.GetBounds())
	}
}

// capsuleIntersectsTriangle 胶囊体与三角形相交检测
func (o *Octree) capsuleIntersectsTriangle(capsule Capsule, triangle Triangle) bool {
	// 简化实现：检查胶囊体轴线到三角形各边的距离
	edges := []struct{ start, end Vector3 }{
		{triangle.A, triangle.B},
		{triangle.B, triangle.C},
		{triangle.C, triangle.A},
	}

	for _, edge := range edges {
		if o.lineSegmentDistance(capsule.Start, capsule.End, edge.start, edge.end) < capsule.Radius {
			return true
		}
	}

	// 检查三角形顶点到胶囊体轴线的距离
	vertices := []Vector3{triangle.A, triangle.B, triangle.C}
	for _, vertex := range vertices {
		if pointToLineSegmentDistance(vertex, capsule.Start, capsule.End) < capsule.Radius {
			return true
		}
	}

	return false
}

// capsuleIntersectsBox 胶囊体与立方体相交检测
func (o *Octree) capsuleIntersectsBox(capsule Capsule, box Box) bool {
	boxBounds := box.GetBounds()

	// 计算胶囊体轴线上最接近立方体的点
	closestPoint := o.closestPointOnLineSegmentToAABB(capsule.Start, capsule.End, boxBounds)

	// 计算立方体上最接近该点的点
	closestOnBox := Vector3{
		math.Max(boxBounds.Min.X, math.Min(closestPoint.X, boxBounds.Max.X)),
		math.Max(boxBounds.Min.Y, math.Min(closestPoint.Y, boxBounds.Max.Y)),
		math.Max(boxBounds.Min.Z, math.Min(closestPoint.Z, boxBounds.Max.Z)),
	}

	// 检查距离是否小于胶囊体半径
	return closestPoint.Distance(closestOnBox) < capsule.Radius
}

// capsuleIntersectsCapsule 胶囊体与胶囊体相交检测
func (o *Octree) capsuleIntersectsCapsule(capsule1, capsule2 Capsule) bool {
	// 计算两个线段之间的最短距离
	distance := o.lineSegmentDistance(capsule1.Start, capsule1.End, capsule2.Start, capsule2.End)
	return distance < (capsule1.Radius + capsule2.Radius)
}

// capsuleIntersectsConvexMesh 胶囊体与凸包网格相交检测
func (o *Octree) capsuleIntersectsConvexMesh(capsule Capsule, mesh ConvexMesh) bool {
	// 首先检查包围盒相交
	if !o.aabbIntersects(capsule.GetBounds(), mesh.GetBounds()) {
		return false
	}

	// 检查胶囊体的起点和终点是否在凸包内
	if mesh.ContainsPoint(capsule.Start) || mesh.ContainsPoint(capsule.End) {
		return true
	}

	// 检查胶囊体轴线是否与凸包的任何面相交，并且距离小于半径
	for _, face := range mesh.Faces {
		if len(face) < 3 {
			continue
		}

		// 检查胶囊轴线到面上各边的距离
		for i := 0; i < len(face); i++ {
			v1 := mesh.Vertices[face[i]]
			v2 := mesh.Vertices[face[(i+1)%len(face)]]

			// 计算胶囊轴线到面边的最短距离
			distance := o.lineSegmentDistance(capsule.Start, capsule.End, v1, v2)
			if distance < capsule.Radius {
				return true
			}
		}

		// 检查胶囊轴线上的点到面的距离
		if o.capsuleAxisIntersectsFace(capsule, mesh, face) {
			return true
		}
	}

	return false
}

// capsuleAxisIntersectsFace 检查胶囊轴线是否与面相交且距离小于半径
func (o *Octree) capsuleAxisIntersectsFace(capsule Capsule, mesh ConvexMesh, face []int) bool {
	if len(face) < 3 {
		return false
	}

	// 获取面的三个顶点计算法向量
	v0 := mesh.Vertices[face[0]]
	v1 := mesh.Vertices[face[1]]
	v2 := mesh.Vertices[face[2]]

	// 计算面的法向量
	edge1 := v1.Sub(v0)
	edge2 := v2.Sub(v0)
	normal := cross(edge1, edge2)

	length := normal.Length()
	if length == 0 {
		return false // 退化面
	}
	normal = normal.Scale(1.0 / length)

	// 计算胶囊轴线上多个点到面的距离
	axisLength := capsule.End.Sub(capsule.Start).Length()
	if axisLength == 0 {
		// 胶囊退化为球体
		distance := math.Abs(dot(capsule.Start.Sub(v0), normal))
		return distance < capsule.Radius
	}

	// 沿轴线采样多个点
	samples := int(math.Max(3, axisLength*2)) // 至少3个采样点
	for i := 0; i <= samples; i++ {
		t := float64(i) / float64(samples)
		point := capsule.Start.Add(capsule.End.Sub(capsule.Start).Scale(t))

		// 计算点到面的距离
		distance := math.Abs(dot(point.Sub(v0), normal))
		if distance < capsule.Radius {
			// 还需要检查点的投影是否在面的边界内
			if o.pointProjectionInFace(point, mesh, face, normal) {
				return true
			}
		}
	}

	return false
}

// pointProjectionInFace 检查点在面上的投影是否在面的边界内
func (o *Octree) pointProjectionInFace(point Vector3, mesh ConvexMesh, face []int, normal Vector3) bool {
	if len(face) < 3 {
		return false
	}

	// 将点投影到面上
	v0 := mesh.Vertices[face[0]]
	toPoint := point.Sub(v0)
	distance := dot(toPoint, normal)
	projectedPoint := point.Sub(normal.Scale(distance))

	// 使用重心坐标或者射线法检查投影点是否在多边形内
	// 这里使用简化的方法：检查点是否在所有边的内侧
	for i := 0; i < len(face); i++ {
		v1 := mesh.Vertices[face[i]]
		v2 := mesh.Vertices[face[(i+1)%len(face)]]

		// 计算边向量和从边起点到投影点的向量
		edge := v2.Sub(v1)
		toProj := projectedPoint.Sub(v1)

		// 计算叉积来判断点在边的哪一侧
		crossProd := cross(edge, toProj)
		if dot(crossProd, normal) < 0 {
			return false // 点在边的外侧
		}
	}

	return true
}

// lineSegmentDistance 计算两个线段之间的最短距离
func (o *Octree) lineSegmentDistance(seg1Start, seg1End, seg2Start, seg2End Vector3) float64 {
	// 简化实现，使用点到线段距离的近似
	dist1 := math.Min(
		pointToLineSegmentDistance(seg1Start, seg2Start, seg2End),
		pointToLineSegmentDistance(seg1End, seg2Start, seg2End),
	)
	dist2 := math.Min(
		pointToLineSegmentDistance(seg2Start, seg1Start, seg1End),
		pointToLineSegmentDistance(seg2End, seg1Start, seg1End),
	)
	return math.Min(dist1, dist2)
}

// closestPointOnLineSegmentToAABB 计算线段上最接近AABB的点
func (o *Octree) closestPointOnLineSegmentToAABB(lineStart, lineEnd Vector3, aabb AABB) Vector3 {
	center := aabb.Center()
	return o.closestPointOnLineSegment(center, lineStart, lineEnd)
}

// closestPointOnLineSegment 计算线段上最接近给定点的点
func (o *Octree) closestPointOnLineSegment(point, lineStart, lineEnd Vector3) Vector3 {
	lineVec := lineEnd.Sub(lineStart)
	pointVec := point.Sub(lineStart)

	lineLength := lineVec.Length()
	if lineLength == 0 {
		return lineStart
	}

	t := dot(pointVec, lineVec) / dot(lineVec, lineVec)
	t = math.Max(0, math.Min(1, t))

	return lineStart.Add(lineVec.Scale(t))
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
