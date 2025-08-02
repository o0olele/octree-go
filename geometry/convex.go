package geometry

import "github.com/o0olele/octree-go/math32"

// ConvexMesh 凸包网格几何体
type ConvexMesh struct {
	Vertices []math32.Vector3 `json:"vertices"` // 凸包的顶点
	Faces    [][]int          `json:"faces"`    // 面，每个面是顶点索引的数组
}

func (cm *ConvexMesh) GetBounds() AABB {
	if len(cm.Vertices) == 0 {
		return AABB{
			Min: math32.Vector3{X: 0, Y: 0, Z: 0},
			Max: math32.Vector3{X: 0, Y: 0, Z: 0},
		}
	}

	min := cm.Vertices[0]
	max := cm.Vertices[0]

	for _, vertex := range cm.Vertices {
		if vertex.X < min.X {
			min.X = vertex.X
		}
		if vertex.Y < min.Y {
			min.Y = vertex.Y
		}
		if vertex.Z < min.Z {
			min.Z = vertex.Z
		}
		if vertex.X > max.X {
			max.X = vertex.X
		}
		if vertex.Y > max.Y {
			max.Y = vertex.Y
		}
		if vertex.Z > max.Z {
			max.Z = vertex.Z
		}
	}

	return AABB{min, max}
}

func (cm *ConvexMesh) IntersectsAABB(aabb AABB) bool {
	bounds := cm.GetBounds()
	return bounds.Intersects(aabb)
}

func (cm *ConvexMesh) ContainsPoint(point math32.Vector3) bool {
	// 使用分离轴定理(SAT)的简化版本来检测点是否在凸包内
	// 对于凸包，如果点在所有面的内侧，则点在凸包内
	for _, face := range cm.Faces {
		if len(face) < 3 {
			continue // 跳过无效面
		}

		// 获取面的三个顶点来计算法向量
		v0 := cm.Vertices[face[0]]
		v1 := cm.Vertices[face[1]]
		v2 := cm.Vertices[face[2]]

		// 计算面的法向量（假设顶点按逆时针顺序排列）
		edge1 := v1.Sub(v0)
		edge2 := v2.Sub(v0)
		normal := edge1.Cross(edge2)

		// 标准化法向量
		length := normal.Length()
		if length == 0 {
			continue // 退化面
		}
		normal = normal.Scale(1.0 / length)

		// 计算点到面的距离
		toPoint := point.Sub(v0)
		distance := toPoint.Dot(normal)

		// 如果点在面的外侧（距离为正），则点不在凸包内
		if distance > 0 {
			return false
		}
	}

	return true
}

func (cm *ConvexMesh) GetType() string {
	return "convex_mesh"
}
