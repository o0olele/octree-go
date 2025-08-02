package geometry

import "github.com/o0olele/octree-go/math32"

// Box 立方体几何体
type Box struct {
	Center math32.Vector3 `json:"center"`
	Size   math32.Vector3 `json:"size"`
}

func (b *Box) GetBounds() AABB {
	halfSize := b.Size.Scale(0.5)
	return AABB{
		b.Center.Sub(halfSize),
		b.Center.Add(halfSize),
	}
}

func (b *Box) IntersectsAABB(aabb AABB) bool {
	bounds := b.GetBounds()
	return !(bounds.Max.X < aabb.Min.X || bounds.Min.X > aabb.Max.X ||
		bounds.Max.Y < aabb.Min.Y || bounds.Min.Y > aabb.Max.Y ||
		bounds.Max.Z < aabb.Min.Z || bounds.Min.Z > aabb.Max.Z)
}

func (b *Box) ContainsPoint(point math32.Vector3) bool {
	bounds := b.GetBounds()
	return bounds.Contains(point)
}

func (b *Box) GetType() string {
	return "box"
}
