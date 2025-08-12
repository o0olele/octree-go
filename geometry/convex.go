package geometry

import "github.com/o0olele/octree-go/math32"

// ConvexMesh is a convex mesh geometry
type ConvexMesh struct {
	Vertices []math32.Vector3 `json:"vertices"` // the vertices of the convex mesh
	Faces    [][]int          `json:"faces"`    // the faces of the convex mesh, each face is an array of vertex indices
}

// GetBounds returns the bounding box of the convex mesh
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

// IntersectsAABB checks if the convex mesh intersects with an AABB
func (cm *ConvexMesh) IntersectsAABB(aabb AABB) bool {
	bounds := cm.GetBounds()
	return bounds.Intersects(aabb)
}

// ContainsPoint checks if the point is inside the convex mesh
func (cm *ConvexMesh) ContainsPoint(point math32.Vector3) bool {
	// use the simplified version of the separating axis theorem (SAT) to detect if the point is inside the convex mesh
	// for the convex mesh, if the point is inside all the faces, then the point is inside the convex mesh
	for _, face := range cm.Faces {
		if len(face) < 3 {
			continue // skip invalid face
		}

		// get the three vertices of the face to calculate the normal
		v0 := cm.Vertices[face[0]]
		v1 := cm.Vertices[face[1]]
		v2 := cm.Vertices[face[2]]

		// calculate the normal of the face (assuming the vertices are in counter-clockwise order)
		edge1 := v1.Sub(v0)
		edge2 := v2.Sub(v0)
		normal := edge1.Cross(edge2)

		// normalize the normal
		length := normal.Length()
		if length == 0 {
			continue // degenerate face
		}
		normal = normal.Scale(1.0 / length)

		// calculate the distance from the point to the face
		toPoint := point.Sub(v0)
		distance := toPoint.Dot(normal)

		// if the point is outside the face (distance is positive), then the point is not inside the convex mesh
		if distance > 0 {
			return false
		}
	}

	return true
}
