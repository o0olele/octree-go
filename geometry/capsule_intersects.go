package geometry

import (
	"github.com/o0olele/octree-go/math32"
)

// 改进的点到线段距离计算
func PointToLineSegmentDistance(point, lineStart, lineEnd math32.Vector3) float32 {
	lineVec := lineEnd.Sub(lineStart)
	pointVec := point.Sub(lineStart)

	lineLength := lineVec.Length()
	if lineLength == 0 {
		return point.Distance(lineStart)
	}

	// 计算投影参数t
	t := pointVec.Dot(lineVec) / lineVec.Dot(lineVec)
	t = math32.Max(0, math32.Min(1, t))

	// 计算线段上最近点
	closestPoint := lineStart.Add(lineVec.Scale(t))
	return point.Distance(closestPoint)
}

// closestPointOnLineSegment 计算点到线段的最近点
func closestPointOnLineSegment(a, b, point math32.Vector3) math32.Vector3 {
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
func pointInTriangle(point, p0, p1, p2, normal math32.Vector3) bool {
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
func SphereTriangleIntersect(center math32.Vector3, radius float32, p0, p1, p2 math32.Vector3) (bool, math32.Vector3, math32.Vector3, float32) {
	// 1. 计算三角形平面
	edge0 := p1.Sub(p0)
	edge1 := p2.Sub(p0)
	normal := edge0.Cross(edge1).Normalize()

	// 2. 计算球心到平面的距离
	dist := center.Sub(p0).Dot(normal)
	if math32.Abs(dist) > radius {
		return false, math32.Vector3{}, math32.Vector3{}, 0
	}

	// 3. 将球心投影到平面上
	projection := center.Sub(normal.Mul(dist))

	// 4. 判断投影点是否在三角形内
	var closestPoint math32.Vector3
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
		return false, math32.Vector3{}, math32.Vector3{}, 0
	}

	// 8. 计算碰撞信息
	var penetrationNormal math32.Vector3
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
func CapsuleTriangleIntersect(capsule Capsule, p0, p1, p2 math32.Vector3) (bool, math32.Vector3, math32.Vector3, float32) {
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
	if math32.Abs(dotNormal) < 1e-6 {
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
	var referencePoint math32.Vector3
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
