package geometry

import (
	"github.com/o0olele/octree-go/math32"
)

// PointToLineSegmentDistance calculates the distance from a point to a line segment
func PointToLineSegmentDistance(point, lineStart, lineEnd math32.Vector3) float32 {
	lineVec := lineEnd.Sub(lineStart)
	pointVec := point.Sub(lineStart)

	lineLength := lineVec.Length()
	if lineLength == 0 {
		return point.Distance(lineStart)
	}

	// calculate the projection parameter t
	t := pointVec.Dot(lineVec) / lineVec.Dot(lineVec)
	t = math32.Max(0, math32.Min(1, t))

	// calculate the closest point on the line segment
	closestPoint := lineStart.Add(lineVec.Scale(t))
	return point.Distance(closestPoint)
}

// closestPointOnLineSegment calculates the closest point on the line segment to the point
func closestPointOnLineSegment(a, b, point math32.Vector3) math32.Vector3 {
	ab := b.Sub(a)
	ap := point.Sub(a)
	// t = (ap · ab) / (ab · ab)
	t := ap.Dot(ab) / ab.Dot(ab)
	// limit t to [0, 1]
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}
	return a.Add(ab.Mul(t))
}

// pointInTriangle checks if the point is inside the triangle (using the simplified version of the barycentric coordinate method)
// assume the point and the triangle are in the same plane
func pointInTriangle(point, p0, p1, p2, normal math32.Vector3) bool {
	// edge normal vector judgment method
	edge0 := p1.Sub(p0)
	edge1 := p2.Sub(p1)
	edge2 := p0.Sub(p2)

	c0 := point.Sub(p0).Cross(edge0)
	c1 := point.Sub(p1).Cross(edge1)
	c2 := point.Sub(p2).Cross(edge2)

	// check if the point is inside all the edges
	dot0 := c0.Dot(normal)
	dot1 := c1.Dot(normal)
	dot2 := c2.Dot(normal)

	// if all the cross product dot product normals are <= 0, then it is inside or on the edge
	return dot0 <= 0 && dot1 <= 0 && dot2 <= 0
}

// SphereTriangleIntersect checks if the sphere intersects with the triangle (the basic of the article)
// return: intersect, collision point, collision normal, penetration depth
func SphereTriangleIntersect(center math32.Vector3, radius float32, p0, p1, p2 math32.Vector3) (bool, math32.Vector3, math32.Vector3, float32) {
	// 1. calculate the triangle plane
	edge0 := p1.Sub(p0)
	edge1 := p2.Sub(p0)
	normal := edge0.Cross(edge1).Normalize()

	// 2. calculate the distance from the sphere center to the plane
	dist := center.Sub(p0).Dot(normal)
	if math32.Abs(dist) > radius {
		return false, math32.Vector3{}, math32.Vector3{}, 0
	}

	// 3. project the sphere center to the plane
	projection := center.Sub(normal.Mul(dist))

	// 4. check if the projected point is inside the triangle
	var closestPoint math32.Vector3
	var inside bool = pointInTriangle(projection, p0, p1, p2, normal)

	if inside {
		closestPoint = projection
	} else {
		// 5. not inside, find the point on the edge closest to the sphere center
		point1 := closestPointOnLineSegment(p0, p1, center)
		point2 := closestPointOnLineSegment(p1, p2, center)
		point3 := closestPointOnLineSegment(p2, p0, center)

		// find the point closest to the sphere center among the three points
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

	// 6. calculate the vector from the closest point to the sphere center
	toCenter := center.Sub(closestPoint)
	distance := toCenter.Length()

	// 7. check if they intersect
	if distance > radius {
		return false, math32.Vector3{}, math32.Vector3{}, 0
	}

	// 8. calculate the collision information
	var penetrationNormal math32.Vector3
	if distance > 1e-6 { // avoid division by zero
		penetrationNormal = toCenter.Mul(1.0 / distance) // normalize
	} else {
		// if the distance is 0, use the triangle normal as the default normal
		penetrationNormal = normal
	}
	penetrationDepth := radius - distance

	return true, closestPoint, penetrationNormal, penetrationDepth
}

// CapsuleTriangleIntersect checks if the capsule intersects with the triangle (the core of the article)
// return: intersect, collision point, collision normal, penetration depth
func CapsuleTriangleIntersect(capsule *Capsule, p0, p1, p2 math32.Vector3) (bool, math32.Vector3, math32.Vector3, float32) {
	radius := capsule.Radius
	base := capsule.Start
	tip := capsule.End

	// 1. calculate the capsule axis direction
	capsuleNormal := tip.Sub(base).Normalize()
	if capsuleNormal.Length() == 0 {
		// degenerate into a sphere
		return SphereTriangleIntersect(base, radius, p0, p1, p2)
	}

	// 2. calculate the endpoints A and B of the "effective" axis of the capsule
	// A and B are the range of the sphere center that can be moved, excluding the hemisphere
	lineEndOffset := capsuleNormal.Mul(radius)
	A := base.Add(lineEndOffset)
	B := tip.Sub(lineEndOffset)

	// 3. calculate the triangle plane normal
	edge0 := p1.Sub(p0)
	edge1 := p2.Sub(p0)
	triangleNormal := edge0.Cross(edge1).Normalize()

	// 4. handle the parallel case (the degenerate case of the ray-plane intersection)
	dotNormal := triangleNormal.Dot(capsuleNormal)
	if math32.Abs(dotNormal) < 1e-6 {
		// the axis is parallel to the plane, cannot intersect or need special treatment
		// the article suggests: take one vertex of the triangle as the reference point
		referencePoint := p0 // take the first vertex
		// project the reference point to the effective segment of the capsule axis [A, B]
		center := closestPointOnLineSegment(A, B, referencePoint)
		// reuse the sphere-triangle intersection detection
		return SphereTriangleIntersect(center, radius, p0, p1, p2)
	}

	// 5. ray-plane intersection (using the capsule axis)
	// parameter t: move t distance from base along capsuleNormal to the intersection point
	t := (p0.Sub(base)).Dot(triangleNormal) / dotNormal
	linePlaneIntersection := base.Add(capsuleNormal.Mul(t))

	// 6. find the point on the triangle closest to linePlaneIntersection (reference point)
	var referencePoint math32.Vector3
	// check if linePlaneIntersection is inside the triangle
	if pointInTriangle(linePlaneIntersection, p0, p1, p2, triangleNormal) {
		referencePoint = linePlaneIntersection
	} else {
		// not inside, find the point on the edge closest to the linePlaneIntersection
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

	// 7. project the reference point back to the capsule axis segment [A, B], get the final sphere center
	center := closestPointOnLineSegment(A, B, referencePoint)

	// 8. reuse the sphere-triangle intersection detection
	return SphereTriangleIntersect(center, radius, p0, p1, p2)
}

// CapsuleIntersectsCapsule checks if the capsule intersects with another capsule
func CapsuleIntersectsCapsule(capsule1, capsule2 *Capsule) bool {
	// calculate the shortest distance between the two line segments
	distance := lineSegmentDistance(capsule1.Start, capsule1.End, capsule2.Start, capsule2.End)
	return distance < (capsule1.Radius + capsule2.Radius)
}

// PointProjectionInFace checks if the point's projection on the face is inside the face
func PointProjectionInFace(point math32.Vector3, mesh ConvexMesh, face []int, normal math32.Vector3) bool {
	if len(face) < 3 {
		return false
	}

	// project the point to the face
	v0 := mesh.Vertices[face[0]]
	toPoint := point.Sub(v0)
	distance := toPoint.Dot(normal)
	projectedPoint := point.Sub(normal.Scale(distance))

	// use the barycentric coordinate or ray method to check if the projected point is inside the polygon
	// here we use the simplified method: check if the point is inside all the edges
	for i := 0; i < len(face); i++ {
		v1 := mesh.Vertices[face[i]]
		v2 := mesh.Vertices[face[(i+1)%len(face)]]

		// calculate the edge vector and the vector from the edge start to the projected point
		edge := v2.Sub(v1)
		toProj := projectedPoint.Sub(v1)

		// calculate the cross product to determine which side of the edge the point is on
		crossProd := edge.Cross(toProj)
		if crossProd.Dot(normal) < 0 {
			return false // the point is outside the edge
		}
	}

	return true
}

// lineSegmentDistance calculates the shortest distance between two line segments
func lineSegmentDistance(seg1Start, seg1End, seg2Start, seg2End math32.Vector3) float32 {
	// simplified implementation, using approximate distance from point to line segment
	dist1 := math32.Min(
		PointToLineSegmentDistance(seg1Start, seg2Start, seg2End),
		PointToLineSegmentDistance(seg1End, seg2Start, seg2End),
	)
	dist2 := math32.Min(
		PointToLineSegmentDistance(seg2Start, seg1Start, seg1End),
		PointToLineSegmentDistance(seg2End, seg1Start, seg1End),
	)
	return math32.Min(dist1, dist2)
}

// ClosestPointOnLineSegmentToAABB calculates the closest point on the line segment to the AABB
func ClosestPointOnLineSegmentToAABB(lineStart, lineEnd math32.Vector3, aabb AABB) math32.Vector3 {
	center := aabb.Center()
	return closestPointOnLineSegment(center, lineStart, lineEnd)
}
