package geometry

import (
	"github.com/o0olele/octree-go/math32"
)

// Triangle is a triangle geometry
type Triangle struct {
	A math32.Vector3 `json:"a"`
	B math32.Vector3 `json:"b"`
	C math32.Vector3 `json:"c"`
}

// GetBounds returns the bounding box of the triangle
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

// IntersectsAABB checks if the triangle intersects with an AABB
func (t *Triangle) IntersectsAABB(aabb AABB) bool {
	// first perform the quick bounding box detection
	bounds := t.GetBounds()
	if !bounds.Intersects(aabb) {
		return false
	}

	// for simple cases, if the triangle is completely inside the AABB, return true
	if aabb.Contains(t.A) && aabb.Contains(t.B) && aabb.Contains(t.C) {
		return true
	}

	// use the Separating Axis Theorem (SAT) for precise detection
	// convert the AABB to center and half size
	center := aabb.Center()
	halfSize := aabb.Size().Scale(0.5)

	// convert the triangle vertices to coordinates relative to the AABB center
	v0 := t.A.Sub(center)
	v1 := t.B.Sub(center)
	v2 := t.C.Sub(center)

	// calculate the edge vectors of the triangle
	f0 := v1.Sub(v0) // edge 0
	f1 := v2.Sub(v1) // edge 1
	f2 := v0.Sub(v2) // edge 2

	// test the normal of the triangle
	normal := f0.Cross(f1)
	if normal.Length() > 1e-10 {
		if !t.testSeparatingAxis(normal, v0, v1, v2, halfSize) {
			return false
		}
	}

	// test the 3 faces of the AABB
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

	// test 9 axes (3 AABB face normals × 3 triangle edge vectors)
	crossAxes := []math32.Vector3{
		// axis u0 x f0, u0 x f1, u0 x f2
		{X: 0, Y: -f0.Z, Z: f0.Y},
		{X: 0, Y: -f1.Z, Z: f1.Y},
		{X: 0, Y: -f2.Z, Z: f2.Y},
		// axis u1 x f0, u1 x f1, u1 x f2
		{X: f0.Z, Y: 0, Z: -f0.X},
		{X: f1.Z, Y: 0, Z: -f1.X},
		{X: f2.Z, Y: 0, Z: -f2.X},
		// axis u2 x f0, u2 x f1, u2 x f2
		{X: -f0.Y, Y: f0.X, Z: 0},
		{X: -f1.Y, Y: f1.X, Z: 0},
		{X: -f2.Y, Y: f2.X, Z: 0},
	}

	for _, axis := range crossAxes {
		// skip zero vector
		if axis.Length() < 1e-10 {
			continue
		}
		if !t.testSeparatingAxis(axis, v0, v1, v2, halfSize) {
			return false
		}
	}

	return true
}

// testSeparatingAxis checks if the projection of the triangle on the given axis is separated
func (t *Triangle) testSeparatingAxis(axis math32.Vector3, v0, v1, v2, halfSize math32.Vector3) bool {
	// calculate the projection of the triangle vertices on the axis
	p0 := v0.Dot(axis)
	p1 := v1.Dot(axis)
	p2 := v2.Dot(axis)

	// find the minimum and maximum values of the triangle projection
	triMin := math32.Min(math32.Min(p0, p1), p2)
	triMax := math32.Max(math32.Max(p0, p1), p2)

	// calculate the radius of the AABB projection on the axis
	r := math32.Abs(halfSize.X*axis.X) + math32.Abs(halfSize.Y*axis.Y) + math32.Abs(halfSize.Z*axis.Z)

	// check if they are separated
	return !(triMax < -r || triMin > r)
}

// ContainsPoint checks if the point is inside the triangle
func (t *Triangle) ContainsPoint(point math32.Vector3) bool {
	bounds := t.GetBounds()
	return bounds.Contains(point)

	// // first check if the point is on the plane of the triangle
	// if !t.isPointOnTrianglePlane(point) {
	// 	return false
	// }

	// // use the barycentric coordinate to detect if the point is inside the triangle
	// return t.isPointInTriangle(point)
}

// IsPointOnTrianglePlane checks if the point is on the plane of the triangle (with a small error)
func (t *Triangle) IsPointOnTrianglePlane(point math32.Vector3) bool {
	// calculate the normal of the triangle
	edge1 := t.B.Sub(t.A)
	edge2 := t.C.Sub(t.A)
	normal := edge1.Cross(edge2)

	// if the triangle is degenerate (area is 0), use the bounding box detection
	if normal.Length() < 1e-10 {
		bounds := t.GetBounds()
		return bounds.Contains(point)
	}

	// normalize the normal
	normal = normal.Scale(1.0 / normal.Length())

	// calculate the distance from the point to the plane
	toPoint := point.Sub(t.A)
	distance := math32.Abs(toPoint.Dot(normal))

	// allow small errors (e.g., due to floating point precision issues)
	const tolerance = 1e-6
	return distance < tolerance
}

// GetNormal returns the normal of the triangle
func (t *Triangle) GetNormal() math32.Vector3 {
	edge1 := t.B.Sub(t.A)
	edge2 := t.C.Sub(t.A)
	return edge1.Cross(edge2).Normalize()
}
