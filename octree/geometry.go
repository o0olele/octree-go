package octree

import (
	"fmt"
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

// Dot 点积
func (v Vector3) Dot(other Vector3) float64 {
	return v.X*other.X + v.Y*other.Y + v.Z*other.Z
}

// Cross 叉积
func (v Vector3) Cross(other Vector3) Vector3 {
	return Vector3{
		v.Y*other.Z - v.Z*other.Y,
		v.Z*other.X - v.X*other.Z,
		v.X*other.Y - v.Y*other.X,
	}
}

// Normalize 向量归一化
func (v Vector3) Normalize() Vector3 {
	len := v.Length()
	if len == 0 {
		return Vector3{0, 0, 0}
	}
	return v.Mul(1.0 / len)
}

func (v Vector3) String() string {
	return fmt.Sprintf("[%2f,%2f,%2f]", v.X, v.Y, v.Z)
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
	t := pointVec.Dot(lineVec) / lineVec.Dot(lineVec)
	t = math.Max(0, math.Min(1, t))

	// 计算线段上最近点
	closestPoint := lineStart.Add(lineVec.Scale(t))
	return point.Distance(closestPoint)
}

// closestPointOnLineSegment 计算点到线段的最近点
func closestPointOnLineSegment(a, b, point Vector3) Vector3 {
	ab := b.Sub(a)
	ap := point.Sub(a)
	// t = (ap · ab) / (ab · ab)
	t := ap.Dot(ab) / ab.Dot(ab)
	// 限制 t 在 [0, 1] 范围内
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}
	return a.Add(ab.Mul(t))
}

// pointInTriangle 判断点是否在三角形内部 (使用重心坐标法的简化版)
// 假设 point 和三角形在同一平面
func pointInTriangle(point, p0, p1, p2, normal Vector3) bool {
	// 边法向量判断方法
	edge0 := p1.Sub(p0)
	edge1 := p2.Sub(p1)
	edge2 := p0.Sub(p2)

	c0 := point.Sub(p0).Cross(edge0)
	c1 := point.Sub(p1).Cross(edge1)
	c2 := point.Sub(p2).Cross(edge2)

	// 检查点是否在所有边的内侧
	dot0 := c0.Dot(normal)
	dot1 := c1.Dot(normal)
	dot2 := c2.Dot(normal)

	// 如果所有叉积点积法线都 <= 0，则在内部或边上
	return dot0 <= 0 && dot1 <= 0 && dot2 <= 0
}

// SphereTriangleIntersect 球体-三角形相交检测 (文章中的基础)
// 返回: 相交, 碰撞点, 碰撞法线, 穿透深度
func SphereTriangleIntersect(center Vector3, radius float64, p0, p1, p2 Vector3) (bool, Vector3, Vector3, float64) {
	// 1. 计算三角形平面
	edge0 := p1.Sub(p0)
	edge1 := p2.Sub(p0)
	normal := edge0.Cross(edge1).Normalize()

	// 2. 计算球心到平面的距离
	dist := center.Sub(p0).Dot(normal)
	if math.Abs(dist) > radius {
		return false, Vector3{}, Vector3{}, 0
	}

	// 3. 将球心投影到平面上
	projection := center.Sub(normal.Mul(dist))

	// 4. 判断投影点是否在三角形内
	var closestPoint Vector3
	var inside bool = pointInTriangle(projection, p0, p1, p2, normal)

	if inside {
		closestPoint = projection
	} else {
		// 5. 不在内部，则找离球心最近的边上的点
		point1 := closestPointOnLineSegment(p0, p1, center)
		point2 := closestPointOnLineSegment(p1, p2, center)
		point3 := closestPointOnLineSegment(p2, p0, center)

		// 找到三个点中离球心最近的一个
		dist1 := center.Sub(point1).Length()
		dist2 := center.Sub(point2).Length()
		dist3 := center.Sub(point3).Length()

		closestPoint = point1
		minDist := dist1
		if dist2 < minDist {
			closestPoint = point2
			minDist = dist2
		}
		if dist3 < minDist {
			closestPoint = point3
		}
	}

	// 6. 计算从最近点到球心的向量
	toCenter := center.Sub(closestPoint)
	distance := toCenter.Length()

	// 7. 判断是否相交
	if distance > radius {
		return false, Vector3{}, Vector3{}, 0
	}

	// 8. 计算碰撞信息
	var penetrationNormal Vector3
	if distance > 1e-6 { // 避免除以0
		penetrationNormal = toCenter.Mul(1.0 / distance) // 归一化
	} else {
		// 如果距离为0，使用三角形法线作为默认法线
		penetrationNormal = normal
	}
	penetrationDepth := radius - distance

	return true, closestPoint, penetrationNormal, penetrationDepth
}

// CapsuleTriangleIntersect 胶囊体-三角形相交检测 (文章的核心)
// 返回: 相交, 碰撞点, 碰撞法线, 穿透深度
func CapsuleTriangleIntersect(capsule Capsule, p0, p1, p2 Vector3) (bool, Vector3, Vector3, float64) {
	radius := capsule.Radius
	base := capsule.Start
	tip := capsule.End

	// 1. 计算胶囊体轴线方向
	capsuleNormal := tip.Sub(base).Normalize()
	if capsuleNormal.Length() == 0 {
		// 退化为球体
		return SphereTriangleIntersect(base, radius, p0, p1, p2)
	}

	// 2. 计算胶囊体“有效”轴线的端点 A 和 B
	// A 和 B 是球心可以移动的范围，排除了半球部分
	lineEndOffset := capsuleNormal.Mul(radius)
	A := base.Add(lineEndOffset)
	B := tip.Sub(lineEndOffset)

	// 3. 计算三角形平面法线
	edge0 := p1.Sub(p0)
	edge1 := p2.Sub(p0)
	triangleNormal := edge0.Cross(edge1).Normalize()

	// 4. 处理平行情况 (射线-平面相交的退化情况)
	dotNormal := triangleNormal.Dot(capsuleNormal)
	if math.Abs(dotNormal) < 1e-6 {
		// 轴线与平面平行，无法相交或需要特殊处理
		// 文章建议：取三角形一个顶点作为参考点
		referencePoint := p0 // 取第一个顶点
		// 将参考点投影到胶囊体轴线的有效段 [A, B] 上
		center := closestPointOnLineSegment(A, B, referencePoint)
		// 复用球体检测
		return SphereTriangleIntersect(center, radius, p0, p1, p2)
	}

	// 5. 射线-平面相交 (使用胶囊体轴线)
	// 参数 t: 从 base 出发，沿 capsuleNormal 移动 t 距离到达交点
	t := (p0.Sub(base)).Dot(triangleNormal) / dotNormal
	linePlaneIntersection := base.Add(capsuleNormal.Mul(t))

	// 6. 在三角形上找到离 linePlaneIntersection 最近的点 (参考点)
	var referencePoint Vector3
	// 判断 linePlaneIntersection 是否在三角形内
	if pointInTriangle(linePlaneIntersection, p0, p1, p2, triangleNormal) {
		referencePoint = linePlaneIntersection
	} else {
		// 不在内部，找最近边上的点
		point1 := closestPointOnLineSegment(p0, p1, linePlaneIntersection)
		point2 := closestPointOnLineSegment(p1, p2, linePlaneIntersection)
		point3 := closestPointOnLineSegment(p2, p0, linePlaneIntersection)

		dist1 := linePlaneIntersection.Sub(point1).Length()
		dist2 := linePlaneIntersection.Sub(point2).Length()
		dist3 := linePlaneIntersection.Sub(point3).Length()

		referencePoint = point1
		minDist := dist1
		if dist2 < minDist {
			referencePoint = point2
			minDist = dist2
		}
		if dist3 < minDist {
			referencePoint = point3
		}
	}

	// 7. 将参考点投影回胶囊体的轴线段 [A, B]，得到最终的球心
	center := closestPointOnLineSegment(A, B, referencePoint)

	// 8. 复用球体-三角形检测
	return SphereTriangleIntersect(center, radius, p0, p1, p2)
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
	normal := f0.Cross(f1)
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
	p0 := v0.Dot(axis)
	p1 := v1.Dot(axis)
	p2 := v2.Dot(axis)

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

	// // 首先检查点是否在三角形所在的平面上
	// if !t.isPointOnTrianglePlane(point) {
	// 	return false
	// }

	// // 使用重心坐标检测点是否在三角形内
	// return t.isPointInTriangle(point)
}

// isPointOnTrianglePlane 检查点是否在三角形所在的平面上（允许一定的误差）
func (t Triangle) isPointOnTrianglePlane(point Vector3) bool {
	// 计算三角形的法向量
	edge1 := t.B.Sub(t.A)
	edge2 := t.C.Sub(t.A)
	normal := edge1.Cross(edge2)

	// 如果三角形退化（面积为0），使用包围盒检测
	if normal.Length() < 1e-10 {
		bounds := t.GetBounds()
		return bounds.Contains(point)
	}

	// 标准化法向量
	normal = normal.Scale(1.0 / normal.Length())

	// 计算点到平面的距离
	toPoint := point.Sub(t.A)
	distance := math.Abs(toPoint.Dot(normal))

	// 允许小的误差（例如，由于浮点精度问题）
	const tolerance = 1e-6
	return distance < tolerance
}

func (t Triangle) GetNormal() Vector3 {
	edge1 := t.B.Sub(t.A)
	edge2 := t.C.Sub(t.A)
	return edge1.Cross(edge2).Normalize()
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

	t := math.Max(0, math.Min(1, toPoint.Dot(axis)/axis.Dot(axis)))
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
		normal := edge1.Cross(edge2)

		// 标准化法向量
		length := normal.Length()
		if length == 0 {
			continue // 退化面
		}
		normal = normal.Scale(1.0 / length)

		// 计算点到面的距离
		toPoint := point.Sub(v0)
		distance := toPoint.Dot(normal)

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
