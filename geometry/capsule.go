package geometry

import (
	"github.com/o0olele/octree-go/math32"
)

// Capsule is a capsule geometry
type Capsule struct {
	Start  math32.Vector3 `json:"start"`
	End    math32.Vector3 `json:"end"`
	Radius float32        `json:"radius"`
}

// GetBounds returns the bounding box of the capsule
func (c *Capsule) GetBounds() AABB {
	minX := math32.Min(c.Start.X, c.End.X) - c.Radius
	maxX := math32.Max(c.Start.X, c.End.X) + c.Radius
	minY := math32.Min(c.Start.Y, c.End.Y) - c.Radius
	maxY := math32.Max(c.Start.Y, c.End.Y) + c.Radius
	minZ := math32.Min(c.Start.Z, c.End.Z) - c.Radius
	maxZ := math32.Max(c.Start.Z, c.End.Z) + c.Radius
	return AABB{
		Min: math32.Vector3{X: minX, Y: minY, Z: minZ},
		Max: math32.Vector3{X: maxX, Y: maxY, Z: maxZ},
	}
}

// IntersectsAABB checks if the capsule intersects with an AABB
func (c *Capsule) IntersectsAABB(aabb AABB) bool {
	bounds := c.GetBounds()
	return !(bounds.Max.X < aabb.Min.X || bounds.Min.X > aabb.Max.X ||
		bounds.Max.Y < aabb.Min.Y || bounds.Min.Y > aabb.Max.Y ||
		bounds.Max.Z < aabb.Min.Z || bounds.Min.Z > aabb.Max.Z)
}

// ContainsPoint checks if the point is inside the capsule
func (c *Capsule) ContainsPoint(point math32.Vector3) bool {
	// calculate the distance from the point to the capsule axis
	axis := c.End.Sub(c.Start)
	toPoint := point.Sub(c.Start)

	if axis.Length() == 0 {
		return toPoint.Length() <= c.Radius
	}

	t := math32.Max(0, math32.Min(1, toPoint.Dot(axis)/axis.Dot(axis)))
	closest := c.Start.Add(axis.Scale(t))
	return point.Distance(closest) <= c.Radius
}
