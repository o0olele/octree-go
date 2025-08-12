package geometry

import (
	"encoding/gob"

	"github.com/o0olele/octree-go/math32"
)

// Geometry interface defines the common methods for all the geometry types
type Geometry interface {
	GetBounds() AABB
	IntersectsAABB(aabb AABB) bool
	ContainsPoint(point math32.Vector3) bool
}

func init() {
	// register the gob types
	gob.Register(Triangle{})
	gob.Register(Box{})
	gob.Register(Capsule{})
	gob.Register(ConvexMesh{})
}
