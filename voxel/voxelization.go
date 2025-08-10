package voxel

import (
	"math"

	"github.com/o0olele/octree-go/geometry"
	"github.com/o0olele/octree-go/math32"
)

// VoxelizeTriangles converts a list of triangles to voxels in the grid
func (g *VoxelGrid) VoxelizeTriangles(triangles []geometry.Triangle) {
	for i := range triangles {
		g.voxelizeTriangle(triangles[i])
	}
}

// voxelizeTriangle converts a single triangle to voxels
func (g *VoxelGrid) voxelizeTriangle(triangle geometry.Triangle) {

	// Get triangle bounding box in world coordinates
	bounds := triangle.GetBounds()
	minWorld := bounds.Min
	maxWorld := bounds.Max

	// Convert to voxel coordinates
	minVoxel := g.WorldToVoxel(minWorld)
	maxVoxel := g.WorldToVoxel(maxWorld)

	// Clamp to grid bounds
	minVoxel = minVoxel.Max(math32.Vector3i{X: 0, Y: 0, Z: 0})
	maxVoxel = maxVoxel.Min(math32.Vector3i{X: g.Size.X - 1, Y: g.Size.Y - 1, Z: g.Size.Z - 1})

	// Count voxels that will be tested
	voxelsToTest := (maxVoxel.X - minVoxel.X + 1) * (maxVoxel.Y - minVoxel.Y + 1) * (maxVoxel.Z - minVoxel.Z + 1)
	solidVoxelsFound := 0

	// Test each voxel in the bounding box
	for z := minVoxel.Z; z <= maxVoxel.Z; z++ {
		for y := minVoxel.Y; y <= maxVoxel.Y; y++ {
			for x := minVoxel.X; x <= maxVoxel.X; x++ {
				voxelCoord := math32.Vector3i{X: x, Y: y, Z: z}

				if g.voxelTriangleIntersect(voxelCoord, triangle) {
					voxel := NewVoxel(VoxelSolid)
					if g.SetVoxel(voxelCoord, voxel) {
						solidVoxelsFound++
					}
				}
			}
		}
	}

	// Log debug info for the first few triangles
	if solidVoxelsFound > 0 {
		// Only log if we found solid voxels to avoid spam
		_ = voxelsToTest // Use the variable to avoid compiler warning
		// log.Printf("Voxelized triangle with %d solid voxels", solidVoxelsFound)
	}
}

// voxelTriangleIntersect tests if a voxel intersects with a triangle
func (g *VoxelGrid) voxelTriangleIntersect(voxelCoord math32.Vector3i, triangle geometry.Triangle) bool {
	// Get voxel bounds in world coordinates
	voxelMin := g.VoxelToWorldCorner(voxelCoord)
	voxelMax := voxelMin.Add(math32.Vector3{X: g.VoxelSize, Y: g.VoxelSize, Z: g.VoxelSize})

	// Use separating axis theorem (SAT) for triangle-box intersection
	return g.triangleBoxIntersect(triangle, voxelMin, voxelMax)
}

// triangleBoxIntersect implements triangle-box intersection using SAT
func (g *VoxelGrid) triangleBoxIntersect(triangle geometry.Triangle, boxMin, boxMax math32.Vector3) bool {
	// Box center and half-extents
	boxCenter := boxMin.Add(boxMax).Mul(0.5)
	boxHalfSize := boxMax.Sub(boxMin).Mul(0.5)

	// Translate triangle so box is centered at origin
	v0 := triangle.A.Sub(boxCenter)
	v1 := triangle.B.Sub(boxCenter)
	v2 := triangle.C.Sub(boxCenter)

	// Triangle edges
	e0 := v1.Sub(v0)
	e1 := v2.Sub(v1)
	e2 := v0.Sub(v2)

	// Test 1: Separating axis = box face normals (x, y, z axes)
	if !g.testAxis(math32.Vector3{X: 1, Y: 0, Z: 0}, v0, v1, v2, boxHalfSize) {
		return false
	}
	if !g.testAxis(math32.Vector3{X: 0, Y: 1, Z: 0}, v0, v1, v2, boxHalfSize) {
		return false
	}
	if !g.testAxis(math32.Vector3{X: 0, Y: 0, Z: 1}, v0, v1, v2, boxHalfSize) {
		return false
	}

	// Test 2: Separating axis = triangle normal
	if !g.testAxis(triangle.GetNormal(), v0, v1, v2, boxHalfSize) {
		return false
	}

	// Test 3: Separating axes = cross products of box edges and triangle edges
	boxAxes := []math32.Vector3{
		{X: 1, Y: 0, Z: 0},
		{X: 0, Y: 1, Z: 0},
		{X: 0, Y: 0, Z: 1},
	}

	triangleEdges := []math32.Vector3{e0, e1, e2}

	for _, boxAxis := range boxAxes {
		for _, triEdge := range triangleEdges {
			axis := boxAxis.Cross(triEdge)
			if axis.LengthSquared() > 1e-6 { // Avoid degenerate cases
				axis = axis.Normalize()
				if !g.testAxis(axis, v0, v1, v2, boxHalfSize) {
					return false
				}
			}
		}
	}

	return true
}

// testAxis tests separation along a given axis
func (g *VoxelGrid) testAxis(axis math32.Vector3, v0, v1, v2, boxHalfSize math32.Vector3) bool {
	// Project triangle vertices onto axis
	p0 := v0.Dot(axis)
	p1 := v1.Dot(axis)
	p2 := v2.Dot(axis)

	// Get triangle projection range
	triMin := math32.Min(p0, math32.Min(p1, p2))
	triMax := math32.Max(p0, math32.Max(p1, p2))

	// Project box onto axis
	boxProjection := math32.Abs(boxHalfSize.X*axis.X) +
		math32.Abs(boxHalfSize.Y*axis.Y) +
		math32.Abs(boxHalfSize.Z*axis.Z)

	// Test for separation
	return !(triMax < -boxProjection || triMin > boxProjection)
}

// VoxelizeWithPadding adds padding around solid voxels for agent collision
func (g *VoxelGrid) VoxelizeWithPadding(triangles []geometry.Triangle, agentRadius, agentHeight float32) {

	// Voxelize the triangles normally
	g.VoxelizeTriangles(triangles)

	// Calculate padding in voxels
	paddingRadiusVoxels := int32(math.Ceil(float64(agentRadius * 0.9 / g.VoxelSize)))
	paddingHeightVoxels := int32(math.Ceil(float64(agentHeight * 0.9 / g.VoxelSize)))

	// Create a copy of the current grid to avoid modifying while iterating
	originalVoxels := make([]Voxel, len(g.Voxels))
	copy(originalVoxels, g.Voxels)

	// For each solid voxel, mark surrounding voxels as solid too
	for z := int32(0); z < g.Size.Z; z++ {
		for y := int32(0); y < g.Size.Y; y++ {
			for x := int32(0); x < g.Size.X; x++ {
				coord := math32.Vector3i{X: x, Y: y, Z: z}
				index := g.GetIndex(coord)

				if originalVoxels[index] == Voxel(VoxelSolid) {
					g.addPaddingAroundVoxel(coord, paddingRadiusVoxels, paddingHeightVoxels)
				}
			}
		}
	}
}

// addPaddingAroundVoxel adds padding around a solid voxel
func (g *VoxelGrid) addPaddingAroundVoxel(center math32.Vector3i, paddingRadiusVoxels, paddingHeightVoxels int32) {
	// Use a sphere-like pattern for padding
	for dz := -paddingHeightVoxels; dz <= paddingHeightVoxels; dz++ {
		for dy := -paddingHeightVoxels; dy <= 0; dy++ {
			for dx := -paddingRadiusVoxels; dx <= paddingRadiusVoxels; dx++ {
				// Check if within padding radius
				distanceXZ := math32.Sqrt(float32(dx*dx + dz*dz))
				if distanceXZ <= float32(paddingRadiusVoxels) {
					if math32.Abs(float32(dy)) <= float32(paddingHeightVoxels) {
						paddedCoord := center.Add(math32.Vector3i{X: int32(dx), Y: int32(dy), Z: int32(dz)})

						if g.IsValidCoordinate(paddedCoord) {
							currentVoxel := g.GetVoxel(paddedCoord)
							if currentVoxel != nil && *currentVoxel == Voxel(VoxelEmpty) {
								g.SetVoxel(paddedCoord, Voxel(VoxelSolid))
							}
						}
					}
				}
			}
		}
	}
}

// VoxelizeConservative uses a more conservative approach for critical applications
func (g *VoxelGrid) VoxelizeConservative(triangles []geometry.Triangle) {
	for _, triangle := range triangles {
		g.voxelizeTriangleConservative(triangle)
	}
}

// voxelizeTriangleConservative uses surface voxelization with flood fill
func (g *VoxelGrid) voxelizeTriangleConservative(triangle geometry.Triangle) {
	// First, voxelize the triangle surface
	g.voxelizeTriangleSurface(triangle)

	// Then, use flood fill to mark interior voxels
	// This is more expensive but more accurate for complex geometry
}

// voxelizeTriangleSurface voxelizes only the surface of the triangle
func (g *VoxelGrid) voxelizeTriangleSurface(triangle geometry.Triangle) {
	// Use 3D line drawing algorithm for each edge
	g.voxelizeLine(triangle.A, triangle.B)
	g.voxelizeLine(triangle.B, triangle.C)
	g.voxelizeLine(triangle.C, triangle.A)

	// Fill the triangle using scanline algorithm
	g.scanlineTriangle(triangle)
}

// voxelizeLine uses 3D DDA algorithm to voxelize a line
func (g *VoxelGrid) voxelizeLine(start, end math32.Vector3) {
	startVoxel := g.WorldToVoxel(start)
	endVoxel := g.WorldToVoxel(end)

	// 3D DDA algorithm
	dx := int(math.Abs(float64(endVoxel.X - startVoxel.X)))
	dy := int(math.Abs(float64(endVoxel.Y - startVoxel.Y)))
	dz := int(math.Abs(float64(endVoxel.Z - startVoxel.Z)))

	stepX := int32(1)
	if startVoxel.X > endVoxel.X {
		stepX = -1
	}
	stepY := int32(1)
	if startVoxel.Y > endVoxel.Y {
		stepY = -1
	}
	stepZ := int32(1)
	if startVoxel.Z > endVoxel.Z {
		stepZ = -1
	}

	x, y, z := startVoxel.X, startVoxel.Y, startVoxel.Z

	// Determine the dominant axis
	if dx >= dy && dx >= dz {
		// X is dominant
		err1 := dx / 2
		err2 := dx / 2

		for {
			coord := math32.Vector3i{X: x, Y: y, Z: z}
			if g.IsValidCoordinate(coord) {
				voxel := NewVoxel(VoxelSolid)
				g.SetVoxel(coord, voxel)
			}

			if x == endVoxel.X {
				break
			}

			err1 -= dy
			err2 -= dz

			if err1 < 0 {
				y += stepY
				err1 += dx
			}
			if err2 < 0 {
				z += stepZ
				err2 += dx
			}

			x += stepX
		}
	} else if dy >= dz {
		// Y is dominant
		err1 := dy / 2
		err2 := dy / 2

		for {
			coord := math32.Vector3i{X: x, Y: y, Z: z}
			if g.IsValidCoordinate(coord) {
				voxel := NewVoxel(VoxelSolid)
				g.SetVoxel(coord, voxel)
			}

			if y == endVoxel.Y {
				break
			}

			err1 -= dx
			err2 -= dz

			if err1 < 0 {
				x += stepX
				err1 += dy
			}
			if err2 < 0 {
				z += stepZ
				err2 += dy
			}

			y += stepY
		}
	} else {
		// Z is dominant
		err1 := dz / 2
		err2 := dz / 2

		for {
			coord := math32.Vector3i{X: x, Y: y, Z: z}
			if g.IsValidCoordinate(coord) {
				voxel := NewVoxel(VoxelSolid)
				g.SetVoxel(coord, voxel)
			}

			if z == endVoxel.Z {
				break
			}

			err1 -= dx
			err2 -= dy

			if err1 < 0 {
				x += stepX
				err1 += dz
			}
			if err2 < 0 {
				y += stepY
				err2 += dz
			}

			z += stepZ
		}
	}
}

// scanlineTriangle fills the interior of a triangle using scanline algorithm
func (g *VoxelGrid) scanlineTriangle(triangle geometry.Triangle) {
	// Get triangle bounding box in voxel coordinates
	bounds := triangle.GetBounds()
	minWorld := bounds.Min
	maxWorld := bounds.Max
	minVoxel := g.WorldToVoxel(minWorld)
	maxVoxel := g.WorldToVoxel(maxWorld)

	// For each Z slice, fill the triangle
	for z := minVoxel.Z; z <= maxVoxel.Z; z++ {
		g.scanlineTriangleSlice(triangle, z)
	}
}

// scanlineTriangleSlice fills a triangle in a specific Z slice
func (g *VoxelGrid) scanlineTriangleSlice(triangle geometry.Triangle, z int32) {
	// This is a simplified version - in practice, you'd want more sophisticated
	// triangle rasterization for better accuracy

	worldZ := g.VoxelToWorld(math32.Vector3i{X: 0, Y: 0, Z: z}).Z

	// Check if this Z plane intersects the triangle
	minZ := math32.Min(triangle.A.Z, math32.Min(triangle.B.Z, triangle.C.Z))
	maxZ := math32.Max(triangle.A.Z, math32.Max(triangle.B.Z, triangle.C.Z))

	if worldZ < minZ || worldZ > maxZ {
		return
	}

	// Simple point-in-triangle test for each voxel in the slice
	bounds := triangle.GetBounds()
	minWorld := bounds.Min
	maxWorld := bounds.Max
	minVoxel := g.WorldToVoxel(minWorld)
	maxVoxel := g.WorldToVoxel(maxWorld)

	for y := minVoxel.Y; y <= maxVoxel.Y; y++ {
		for x := minVoxel.X; x <= maxVoxel.X; x++ {
			coord := math32.Vector3i{X: x, Y: y, Z: z}
			if g.IsValidCoordinate(coord) {
				worldPos := g.VoxelToWorld(coord)
				if g.pointInTriangle2D(worldPos, triangle) {
					voxel := NewVoxel(VoxelSolid)

					g.SetVoxel(coord, voxel)
				}
			}
		}
	}
}

// pointInTriangle2D tests if a point is inside a triangle (projected to 2D)
func (g *VoxelGrid) pointInTriangle2D(point math32.Vector3, triangle geometry.Triangle) bool {
	// Use barycentric coordinates for point-in-triangle test
	// Project to the plane with the largest normal component

	normal := triangle.GetNormal()
	absNormal := math32.Vector3{
		X: math32.Abs(normal.X),
		Y: math32.Abs(normal.Y),
		Z: math32.Abs(normal.Z),
	}

	var u, v float32
	var p0, p1, p2 math32.Vector3

	if absNormal.X >= absNormal.Y && absNormal.X >= absNormal.Z {
		// Project to YZ plane
		u = point.Y
		v = point.Z
		p0 = math32.Vector3{X: 0, Y: triangle.A.Y, Z: triangle.A.Z}
		p1 = math32.Vector3{X: 0, Y: triangle.B.Y, Z: triangle.B.Z}
		p2 = math32.Vector3{X: 0, Y: triangle.C.Y, Z: triangle.C.Z}
	} else if absNormal.Y >= absNormal.Z {
		// Project to XZ plane
		u = point.X
		v = point.Z
		p0 = math32.Vector3{X: triangle.A.X, Y: 0, Z: triangle.A.Z}
		p1 = math32.Vector3{X: triangle.B.X, Y: 0, Z: triangle.B.Z}
		p2 = math32.Vector3{X: triangle.C.X, Y: 0, Z: triangle.C.Z}
	} else {
		// Project to XY plane
		u = point.X
		v = point.Y
		p0 = math32.Vector3{X: triangle.A.X, Y: triangle.A.Y, Z: 0}
		p1 = math32.Vector3{X: triangle.B.X, Y: triangle.B.Y, Z: 0}
		p2 = math32.Vector3{X: triangle.C.X, Y: triangle.C.Y, Z: 0}
	}

	// Barycentric coordinate test
	denom := (p1.Y-p2.Y)*(p0.X-p2.X) + (p2.X-p1.X)*(p0.Y-p2.Y)
	if math32.Abs(denom) < 1e-10 {
		return false // Degenerate triangle
	}

	a := ((p1.Y-p2.Y)*(u-p2.X) + (p2.X-p1.X)*(v-p2.Y)) / denom
	b := ((p2.Y-p0.Y)*(u-p2.X) + (p0.X-p2.X)*(v-p2.Y)) / denom
	c := 1 - a - b

	return a >= 0 && b >= 0 && c >= 0
}

// Helper function for absolute value
func abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}
