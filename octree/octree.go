package octree

import (
	"encoding/json"
	"fmt"
	"math"
)

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
	start := Vector3{position.X, position.Y + a.Radius, position.Z}
	end := Vector3{position.X, position.Y + a.Height + a.Radius, position.Z}
	return Capsule{
		Start:  start,
		End:    end,
		Radius: a.Radius,
	}
}

func GetCapsuleAABB(position Vector3, radius float64, height float64) AABB {
	start := Vector3{position.X, position.Y, position.Z}
	end := Vector3{position.X, position.Y + height + 2*radius, position.Z}
	return AABB{
		Min: Vector3{start.X - radius, start.Y - radius, start.Z - radius},
		Max: Vector3{end.X + radius, end.Y + radius, end.Z + radius},
	}
}

// GetBounds 获取Agent在指定位置的包围盒
func (a *Agent) GetBounds(position Vector3) AABB {
	return a.GetCapsule(position).GetBounds()
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

var Log bool

// capsuleIntersectsTriangle 胶囊体与三角形相交检测
func (o *Octree) capsuleIntersectsTriangle(capsule Capsule, triangle Triangle) bool {
	intersect, _, _, _ := CapsuleTriangleIntersect(capsule, triangle.A, triangle.B, triangle.C)
	return intersect
	// // 简化实现：检查胶囊体轴线到三角形各边的距离
	// edges := []struct{ start, end Vector3 }{
	// 	{triangle.A, triangle.B},
	// 	{triangle.B, triangle.C},
	// 	{triangle.C, triangle.A},
	// }

	// for _, edge := range edges {
	// 	if o.lineSegmentDistance(capsule.Start, capsule.End, edge.start, edge.end) < capsule.Radius {
	// 		return true
	// 	}
	// }

	// // 检查三角形顶点到胶囊体轴线的距离
	// vertices := []Vector3{triangle.A, triangle.B, triangle.C}
	// for _, vertex := range vertices {
	// 	if pointToLineSegmentDistance(vertex, capsule.Start, capsule.End) < capsule.Radius {
	// 		return true
	// 	}
	// }

	// return false
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
	normal := edge1.Cross(edge2)

	length := normal.Length()
	if length == 0 {
		return false // 退化面
	}
	normal = normal.Scale(1.0 / length)

	// 计算胶囊轴线上多个点到面的距离
	axisLength := capsule.End.Sub(capsule.Start).Length()
	if axisLength == 0 {
		// 胶囊退化为球体
		distance := math.Abs(capsule.Start.Sub(v0).Dot(normal))
		return distance < capsule.Radius
	}

	// 沿轴线采样多个点
	samples := int(math.Max(3, axisLength*2)) // 至少3个采样点
	for i := 0; i <= samples; i++ {
		t := float64(i) / float64(samples)
		point := capsule.Start.Add(capsule.End.Sub(capsule.Start).Scale(t))

		// 计算点到面的距离
		distance := math.Abs(point.Sub(v0).Dot(normal))
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
	distance := toPoint.Dot(normal)
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
		crossProd := edge.Cross(toProj)
		if crossProd.Dot(normal) < 0 {
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

	t := pointVec.Dot(lineVec) / lineVec.Dot(lineVec)
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

// isPathClear 检查两点之间的路径是否畅通
func (o *Octree) isPathClear(agent *Agent, start, end Vector3) bool {

	// 计算方向向量和距离
	direction := end.Sub(start)
	distance := direction.Length()

	if distance < 0.001 { // 距离太近，认为是同一点
		return true
	}

	// 标准化方向向量
	direction = direction.Scale(1.0 / distance)

	agentRadius := 0.4
	if agent != nil {
		agentRadius = agent.Radius
	}
	// 使用适当的步长进行采样检查
	stepSize := math.Max(0.1, agentRadius*0.6)
	steps := int(math.Ceil(distance / stepSize))

	// 沿着路径进行采样检测
	for i := 0; i <= steps; i++ {
		t := float64(i) / float64(steps)
		samplePoint := start.Add(direction.Scale(distance * t))

		// 如果有Agent半径，还需要检查Agent碰撞
		occupied := o.IsAgentOccupied(agent, samplePoint)
		if Log {
			fmt.Println(start, end, agent, samplePoint, occupied)
		}
		if occupied {
			return false
		}

	}

	return true
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
