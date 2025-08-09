package octree

import (
	"encoding/json"

	"github.com/o0olele/octree-go/geometry"
	"github.com/o0olele/octree-go/math32"
)

// Agent 表示寻路的智能体，用胶囊体表示
type Agent struct {
	Radius  float32           `json:"radius"` // 胶囊体半径
	Height  float32           `json:"height"` // 胶囊体高度（不包括两端的半球）
	Capsule *geometry.Capsule `json:"capsule"`
}

// NewAgent 创建新的Agent
func NewAgent(radius, height float32) *Agent {
	return &Agent{
		Radius: radius,
		Height: height,
	}
}

// GetCapsule 根据位置获取Agent的胶囊体表示
func (a *Agent) GetCapsule(position math32.Vector3) *geometry.Capsule {
	if a.Capsule != nil {
		return a.Capsule
	}
	// 胶囊体的轴线沿Y轴方向
	start := math32.Vector3{
		X: position.X,
		Y: position.Y + a.Radius,
		Z: position.Z,
	}
	end := math32.Vector3{
		X: position.X,
		Y: position.Y + a.Height + a.Radius,
		Z: position.Z,
	}
	return &geometry.Capsule{
		Start:  start,
		End:    end,
		Radius: a.Radius,
	}
}

// GetBounds 获取Agent在指定位置的包围盒
func (a *Agent) GetBounds(position math32.Vector3) geometry.AABB {
	return a.GetCapsule(position).GetBounds()
}

// Octree 八叉树主结构
type Octree struct {
	Root        *OctreeNode `json:"root"`
	MaxDepth    uint8       `json:"max_depth"`
	MinSize     float32     `json:"min_size"`
	triangles   []geometry.Triangle
	buildHelper *OctreeBuildHelper
}

// NewOctree 创建新的八叉树
func NewOctree(bounds geometry.AABB, maxDepth uint8, minSize float32) *Octree {
	return &Octree{
		Root: &OctreeNode{
			Bounds: bounds,
			Flags:  FlagsDefault,
			Depth:  0,
		},
		MaxDepth:  maxDepth,
		MinSize:   minSize,
		triangles: make([]geometry.Triangle, 0),
	}
}

func (o *Octree) SetTriangles(triangles []geometry.Triangle) {
	o.triangles = triangles
}

func (o *Octree) SetHelper(helper *OctreeBuildHelper) {
	o.buildHelper = helper
}

// AddTriangle 添加三角形面到八叉树
func (o *Octree) AddTriangle(triangle geometry.Triangle) {
	o.triangles = append(o.triangles, triangle)
}

func (o *Octree) GetTrianglesNum() int {
	return len(o.triangles)
}

func (o *Octree) GetTriangles() []geometry.Triangle {
	return o.triangles
}

func (o *Octree) AddTriangles(triangles []geometry.Triangle) {
	o.triangles = append(o.triangles, triangles...)
}

func (o *Octree) insertTriangle(node *OctreeNode, triangleIndex int, triangle *geometry.Triangle) {
	if !triangle.IntersectsAABB(node.Bounds) {
		return
	}

	node.Triangles = append(node.Triangles, triangle)
	node.SetOccupied(true)

	if o.buildHelper != nil {
		o.buildHelper.AddTriangleIndex(node, uint32(triangleIndex))
	}

	// 如果节点是叶子且达到分割条件，则分割
	if node.IsLeaf() && node.Depth < o.MaxDepth && node.Bounds.Size().X > o.MinSize {
		o.subdivide(node)
	}

	// 如果不是叶子，递归插入到子节点
	if !node.IsLeaf() {
		for _, child := range node.Children {
			if child != nil {
				o.insertTriangle(child, triangleIndex, triangle)
			}
		}
	}
}

func (o *Octree) subdivide(node *OctreeNode) {
	center := node.Bounds.Center()
	size := node.Bounds.Size().Scale(0.5)

	positions := [8]math32.Vector3{
		{X: node.Bounds.Min.X, Y: node.Bounds.Min.Y, Z: node.Bounds.Min.Z}, // 000
		{X: center.X, Y: node.Bounds.Min.Y, Z: node.Bounds.Min.Z},          // 100
		{X: node.Bounds.Min.X, Y: center.Y, Z: node.Bounds.Min.Z},          // 010
		{X: center.X, Y: center.Y, Z: node.Bounds.Min.Z},                   // 110
		{X: node.Bounds.Min.X, Y: node.Bounds.Min.Y, Z: center.Z},          // 001
		{X: center.X, Y: node.Bounds.Min.Y, Z: center.Z},                   // 101
		{X: node.Bounds.Min.X, Y: center.Y, Z: center.Z},                   // 011
		{X: center.X, Y: center.Y, Z: center.Z},                            // 111
	}

	for i := 0; i < 8; i++ {
		childBounds := geometry.AABB{
			Min: positions[i],
			Max: positions[i].Add(size),
		}

		node.Children[i] = &OctreeNode{
			Bounds: childBounds,
			Flags:  FlagsDefault,
			Depth:  node.Depth + 1,
		}
	}
	if o.buildHelper != nil {
		o.buildHelper.AddChildren(node, node.Children)
	}

	node.SetLeaf(false)
}

// Build 构建八叉树
func (o *Octree) Build() {
	for idx := range o.triangles {
		o.insertTriangle(o.Root, idx, &o.triangles[idx])
	}
}

// IsOccupied 检查指定位置是否被占用
func (o *Octree) IsOccupied(point math32.Vector3) bool {
	return o.isOccupiedRecursive(o.Root, point)
}

// IsAgentOccupied 检查Agent在指定位置是否与几何体碰撞
func (o *Octree) IsAgentOccupied(agent *Agent, position math32.Vector3) bool {
	if agent == nil {
		return o.IsOccupied(position)
	}

	agentCapsule := agent.GetCapsule(position)
	return o.isAgentOccupiedRecursive(o.Root, agentCapsule)
}

func (o *Octree) IsAgentOccupiedByNodeGeometry(agentCapsule *geometry.Capsule, node *OctreeNode) bool {
	// 检查Agent胶囊体是否与节点中的几何体碰撞
	for _, triangle := range node.Triangles {
		if o.capsuleIntersectsTriangle(agentCapsule, triangle) {
			return true
		}
	}

	return false
}

func (o *Octree) isAgentOccupiedRecursive(node *OctreeNode, agentCapsule *geometry.Capsule) bool {
	// 检查Agent的包围盒是否与节点相交
	agentBounds := agentCapsule.GetBounds()
	if !o.aabbIntersects(node.Bounds, agentBounds) {
		return false
	}

	if node.IsLeaf() {
		if !node.IsOccupied() {
			return false
		}
		return o.IsAgentOccupiedByNodeGeometry(agentCapsule, node)
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
func (o *Octree) aabbIntersects(a, b geometry.AABB) bool {
	return !(a.Max.X < b.Min.X || a.Min.X > b.Max.X ||
		a.Max.Y < b.Min.Y || a.Min.Y > b.Max.Y ||
		a.Max.Z < b.Min.Z || a.Min.Z > b.Max.Z)
}

// capsuleIntersectsTriangle 胶囊体与三角形相交检测
func (o *Octree) capsuleIntersectsTriangle(capsule *geometry.Capsule, triangle *geometry.Triangle) bool {
	intersect, _, _, _ := geometry.CapsuleTriangleIntersect(capsule, triangle.A, triangle.B, triangle.C)
	return intersect
}

// capsuleIntersectsCapsule 胶囊体与胶囊体相交检测
func (o *Octree) capsuleIntersectsCapsule(capsule1, capsule2 geometry.Capsule) bool {
	// 计算两个线段之间的最短距离
	distance := o.lineSegmentDistance(capsule1.Start, capsule1.End, capsule2.Start, capsule2.End)
	return distance < (capsule1.Radius + capsule2.Radius)
}

// pointProjectionInFace 检查点在面上的投影是否在面的边界内
func (o *Octree) pointProjectionInFace(point math32.Vector3, mesh geometry.ConvexMesh, face []int, normal math32.Vector3) bool {
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
func (o *Octree) lineSegmentDistance(seg1Start, seg1End, seg2Start, seg2End math32.Vector3) float32 {
	// 简化实现，使用点到线段距离的近似
	dist1 := math32.Min(
		geometry.PointToLineSegmentDistance(seg1Start, seg2Start, seg2End),
		geometry.PointToLineSegmentDistance(seg1End, seg2Start, seg2End),
	)
	dist2 := math32.Min(
		geometry.PointToLineSegmentDistance(seg2Start, seg1Start, seg1End),
		geometry.PointToLineSegmentDistance(seg2End, seg1Start, seg1End),
	)
	return math32.Min(dist1, dist2)
}

// closestPointOnLineSegmentToAABB 计算线段上最接近AABB的点
func (o *Octree) closestPointOnLineSegmentToAABB(lineStart, lineEnd math32.Vector3, aabb geometry.AABB) math32.Vector3 {
	center := aabb.Center()
	return o.closestPointOnLineSegment(center, lineStart, lineEnd)
}

// closestPointOnLineSegment 计算线段上最接近给定点的点
func (o *Octree) closestPointOnLineSegment(point, lineStart, lineEnd math32.Vector3) math32.Vector3 {
	lineVec := lineEnd.Sub(lineStart)
	pointVec := point.Sub(lineStart)

	lineLength := lineVec.Length()
	if lineLength == 0 {
		return lineStart
	}

	t := pointVec.Dot(lineVec) / lineVec.Dot(lineVec)
	t = math32.Max(0, math32.Min(1, t))

	return lineStart.Add(lineVec.Scale(t))
}

func (o *Octree) isOccupiedRecursive(node *OctreeNode, point math32.Vector3) bool {
	if !node.Bounds.Contains(point) {
		return false
	}

	if node.IsLeaf() {
		if !node.IsOccupied() {
			return false
		}
		// 检查所有几何体
		for _, triangle := range node.Triangles {
			if triangle.ContainsPoint(point) {
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
func (o *Octree) IsPathClear(agent *Agent, start, end math32.Vector3) bool {

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
		occupied := o.IsAgentOccupied(agent, samplePoint)
		if occupied {
			return false
		}

	}

	return true
}

// 用于JSON序列化的简化结构
type OctreeExport struct {
	Root     *OctreeNodeExport `json:"root"`
	MaxDepth uint8             `json:"max_depth"`
	MinSize  float32           `json:"min_size"`
}

type OctreeNodeExport struct {
	Bounds     geometry.AABB       `json:"bounds"`
	Children   []*OctreeNodeExport `json:"children,omitempty"`
	IsLeaf     bool                `json:"is_leaf"`
	IsOccupied bool                `json:"is_occupied"`
	Depth      uint8               `json:"depth"`
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
		IsLeaf:     node.IsLeaf(),
		IsOccupied: node.IsOccupied(),
		Depth:      node.Depth,
	}

	if !node.IsLeaf() {
		for _, child := range node.Children {
			export.Children = append(export.Children, o.nodeToExport(child))
		}
	}

	return export
}

// Raycast 在八叉树中进行射线检测，返回是否命中、命中距离、命中点与命中三角形
// direction 可为任意长度，内部会归一化；maxDistance 为最大检测距离（<=0 则视为无上限）
func (o *Octree) Raycast(origin, direction math32.Vector3, maxDistance float32) (bool, float32, math32.Vector3, *geometry.Triangle) {
	dirLen := direction.Length()
	if dirLen <= 1e-6 {
		return false, 0, origin, nil
	}
	dir := direction.Scale(1.0 / dirLen)
	if maxDistance <= 0 {
		maxDistance = math32.MaxFloat32
	}

	// 快速剔除：与根节点不相交直接失败
	if _, _, ok := geometry.RayAABB(origin, dir, o.Root.Bounds); !ok {
		return false, 0, origin, nil
	}

	bestT := maxDistance
	var bestTri *geometry.Triangle
	o.raycastNode(o.Root, origin, dir, maxDistance, &bestT, &bestTri)
	if bestTri == nil {
		return false, 0, origin, nil
	}
	hitPoint := origin.Add(dir.Scale(bestT))
	return true, bestT, hitPoint, bestTri
}

func (o *Octree) raycastNode(node *OctreeNode, origin, dir math32.Vector3, maxT float32, bestT *float32, bestTri **geometry.Triangle) {
	tmin, _, ok := geometry.RayAABB(origin, dir, node.Bounds)
	if !ok || tmin > *bestT || tmin > maxT {
		return
	}
	if node.IsLeaf() {
		for _, tri := range node.Triangles {
			if hit, t := geometry.RayTriangle(origin, dir, tri); hit && t < *bestT && t <= maxT {
				*bestT = t
				*bestTri = tri
			}
		}
		return
	}
	for _, child := range node.Children {
		if child != nil {
			o.raycastNode(child, origin, dir, maxT, bestT, bestTri)
		}
	}
}
