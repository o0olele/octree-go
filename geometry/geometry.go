package geometry

import (
	"encoding/gob"

	"github.com/o0olele/octree-go/math32"
)

// Geometry 接口定义
type Geometry interface {
	GetBounds() AABB
	IntersectsAABB(aabb AABB) bool
	ContainsPoint(point math32.Vector3) bool
	GetType() string
}

func init() {
	// 注册gob类型
	gob.Register(Triangle{})
	gob.Register(Box{})
	gob.Register(Capsule{})
	gob.Register(ConvexMesh{})
}
