package geometry

import "github.com/o0olele/octree-go/math32"

// AABB 轴对齐包围盒
type AABB struct {
	Min math32.Vector3 `json:"min"`
	Max math32.Vector3 `json:"max"`
}

func (aabb AABB) Contains(point math32.Vector3) bool {
	return point.X >= aabb.Min.X && point.X <= aabb.Max.X &&
		point.Y >= aabb.Min.Y && point.Y <= aabb.Max.Y &&
		point.Z >= aabb.Min.Z && point.Z <= aabb.Max.Z
}

func (aabb AABB) Center() math32.Vector3 {
	return math32.Vector3{
		X: (aabb.Min.X + aabb.Max.X) / 2,
		Y: (aabb.Min.Y + aabb.Max.Y) / 2,
		Z: (aabb.Min.Z + aabb.Max.Z) / 2,
	}
}

func (aabb AABB) Size() math32.Vector3 {
	return aabb.Max.Sub(aabb.Min)
}

func (aabb AABB) Intersects(other AABB) bool {
	return aabb.Min.X <= other.Max.X && aabb.Max.X >= other.Min.X &&
		aabb.Min.Y <= other.Max.Y && aabb.Max.Y >= other.Min.Y &&
		aabb.Min.Z <= other.Max.Z && aabb.Max.Z >= other.Min.Z
}

func GetCapsuleAABB(position math32.Vector3, radius float32, height float32) AABB {
	start := math32.Vector3{
		X: position.X,
		Y: position.Y,
		Z: position.Z,
	}
	end := math32.Vector3{
		X: position.X,
		Y: position.Y + height + 2*radius,
		Z: position.Z,
	}
	return AABB{
		Min: math32.Vector3{
			X: start.X - radius,
			Y: start.Y - radius,
			Z: start.Z - radius,
		},
		Max: math32.Vector3{
			X: end.X + radius,
			Y: end.Y + radius,
			Z: end.Z + radius,
		},
	}
}
