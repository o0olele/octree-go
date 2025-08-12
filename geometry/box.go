package geometry

import "github.com/o0olele/octree-go/math32"

// Box is a box geometry
type Box struct {
	Center math32.Vector3 `json:"center"`
	Size   math32.Vector3 `json:"size"`
}

// GetBounds returns the bounding box of the box
func (b *Box) GetBounds() AABB {
	halfSize := b.Size.Scale(0.5)
	return AABB{
		b.Center.Sub(halfSize),
		b.Center.Add(halfSize),
	}
}

// IntersectsAABB checks if the box intersects with an AABB
func (b *Box) IntersectsAABB(aabb AABB) bool {
	bounds := b.GetBounds()
	return !(bounds.Max.X < aabb.Min.X || bounds.Min.X > aabb.Max.X ||
		bounds.Max.Y < aabb.Min.Y || bounds.Min.Y > aabb.Max.Y ||
		bounds.Max.Z < aabb.Min.Z || bounds.Min.Z > aabb.Max.Z)
}

// ContainsPoint checks if the point is inside the box
func (b *Box) ContainsPoint(point math32.Vector3) bool {
	bounds := b.GetBounds()
	return bounds.Contains(point)
}
