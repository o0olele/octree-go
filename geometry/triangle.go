package geometry

import (
	"github.com/o0olele/octree-go/math32"
)

// Triangle 三角形几何体
type Triangle struct {
	A math32.Vector3 `json:"a"`
	B math32.Vector3 `json:"b"`
	C math32.Vector3 `json:"c"`
}

func (t *Triangle) GetBounds() AABB {
	minX := math32.Min(math32.Min(t.A.X, t.B.X), t.C.X)
	maxX := math32.Max(math32.Max(t.A.X, t.B.X), t.C.X)
	minY := math32.Min(math32.Min(t.A.Y, t.B.Y), t.C.Y)
	maxY := math32.Max(math32.Max(t.A.Y, t.B.Y), t.C.Y)
	minZ := math32.Min(math32.Min(t.A.Z, t.B.Z), t.C.Z)
	maxZ := math32.Max(math32.Max(t.A.Z, t.B.Z), t.C.Z)
	return AABB{
		Min: math32.Vector3{X: minX, Y: minY, Z: minZ},
		Max: math32.Vector3{X: maxX, Y: maxY, Z: maxZ},
	}
}

func (t *Triangle) IntersectsAABB(aabb AABB) bool {
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
	aabbAxes := []math32.Vector3{
		{X: 1, Y: 0, Z: 0},
		{X: 0, Y: 1, Z: 0},
		{X: 0, Y: 0, Z: 1},
	}

	for _, axis := range aabbAxes {
		if !t.testSeparatingAxis(axis, v0, v1, v2, halfSize) {
			return false
		}
	}

	// 测试9个轴（3个AABB面法向量 × 3个三角形边向量的叉积）
	crossAxes := []math32.Vector3{
		// 轴 u0 x f0, u0 x f1, u0 x f2
		{X: 0, Y: -f0.Z, Z: f0.Y},
		{X: 0, Y: -f1.Z, Z: f1.Y},
		{X: 0, Y: -f2.Z, Z: f2.Y},
		// 轴 u1 x f0, u1 x f1, u1 x f2
		{X: f0.Z, Y: 0, Z: -f0.X},
		{X: f1.Z, Y: 0, Z: -f1.X},
		{X: f2.Z, Y: 0, Z: -f2.X},
		// 轴 u2 x f0, u2 x f1, u2 x f2
		{X: -f0.Y, Y: f0.X, Z: 0},
		{X: -f1.Y, Y: f1.X, Z: 0},
		{X: -f2.Y, Y: f2.X, Z: 0},
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
func (t *Triangle) testSeparatingAxis(axis math32.Vector3, v0, v1, v2, halfSize math32.Vector3) bool {
	// 计算三角形顶点在轴上的投影
	p0 := v0.Dot(axis)
	p1 := v1.Dot(axis)
	p2 := v2.Dot(axis)

	// 找到三角形投影的最小值和最大值
	triMin := math32.Min(math32.Min(p0, p1), p2)
	triMax := math32.Max(math32.Max(p0, p1), p2)

	// 计算AABB在轴上的投影半径
	r := math32.Abs(halfSize.X*axis.X) + math32.Abs(halfSize.Y*axis.Y) + math32.Abs(halfSize.Z*axis.Z)

	// 检查是否分离
	return !(triMax < -r || triMin > r)
}

func (t *Triangle) ContainsPoint(point math32.Vector3) bool {
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
func (t *Triangle) isPointOnTrianglePlane(point math32.Vector3) bool {
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
	distance := math32.Abs(toPoint.Dot(normal))

	// 允许小的误差（例如，由于浮点精度问题）
	const tolerance = 1e-6
	return distance < tolerance
}

func (t *Triangle) GetNormal() math32.Vector3 {
	edge1 := t.B.Sub(t.A)
	edge2 := t.C.Sub(t.A)
	return edge1.Cross(edge2).Normalize()
}

func (t *Triangle) GetType() string {
	return "triangle"
}
