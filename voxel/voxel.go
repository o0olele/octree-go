package voxel

import (
	"unsafe"

	"github.com/o0olele/octree-go/math32"
)

// VoxelType represents different types of voxels
type VoxelType uint8

const (
	VoxelEmpty VoxelType = iota // Empty space (walkable)
	VoxelSolid                  // Solid obstacle (not walkable)
	VoxelWater                  // Water (walkable with different cost)
	VoxelStart                  // Start position marker
	VoxelGoal                   // Goal position marker
	VoxelPath                   // Path visualization
)

// Voxel represents a single voxel in the grid
type Voxel VoxelType

// NewVoxel creates a new voxel
func NewVoxel(voxelType VoxelType) Voxel {
	return Voxel(voxelType)
}

// IsWalkable returns true if the voxel can be traversed
func (v Voxel) IsWalkable() bool {
	return v == Voxel(VoxelEmpty) || v == Voxel(VoxelWater) || v == Voxel(VoxelStart) || v == Voxel(VoxelGoal)
}

// VoxelGrid represents a 3D grid of voxels
type VoxelGrid struct {
	Size        math32.Vector3i `json:"size"`       // Grid dimensions (width, height, depth)
	VoxelSize   float32         `json:"voxel_size"` // Size of each voxel in world units
	Origin      math32.Vector3  `json:"origin"`     // World position of grid origin (0,0,0)
	Voxels      []Voxel         `json:"voxels"`     // Flattened 3D array of voxels
	BoundingBox struct {
		Min math32.Vector3 `json:"min"`
		Max math32.Vector3 `json:"max"`
	} `json:"bounding_box"`
}

// NewVoxelGrid creates a new voxel grid
func NewVoxelGrid(size math32.Vector3i, voxelSize float32, origin math32.Vector3) *VoxelGrid {
	totalVoxels := size.X * size.Y * size.Z
	voxels := make([]Voxel, totalVoxels)

	// Initialize all voxels as empty
	for i := range voxels {
		voxels[i] = NewVoxel(VoxelEmpty)
	}

	grid := &VoxelGrid{
		Size:      size,
		VoxelSize: voxelSize,
		Origin:    origin,
		Voxels:    voxels,
	}

	// Calculate bounding box
	grid.updateBoundingBox()

	return grid
}

// updateBoundingBox calculates the world-space bounding box of the grid
func (g *VoxelGrid) updateBoundingBox() {
	worldSize := math32.Vector3{
		X: float32(g.Size.X) * g.VoxelSize,
		Y: float32(g.Size.Y) * g.VoxelSize,
		Z: float32(g.Size.Z) * g.VoxelSize,
	}

	g.BoundingBox.Min = g.Origin
	g.BoundingBox.Max = g.Origin.Add(worldSize)
}

// GetIndex converts 3D coordinates to 1D array index
func (g *VoxelGrid) GetIndex(coord math32.Vector3i) int {
	if !g.IsValidCoordinate(coord) {
		return -1
	}
	return int(coord.Z*g.Size.X*g.Size.Y + coord.Y*g.Size.X + coord.X)
}

// GetCoordinate converts 1D array index to 3D coordinates
func (g *VoxelGrid) GetCoordinate(index int) math32.Vector3i {
	if index < 0 || index >= len(g.Voxels) {
		return math32.Vector3i{X: -1, Y: -1, Z: -1}
	}

	z := int32(index / int(g.Size.X*g.Size.Y))
	remainder := int32(index % int(g.Size.X*g.Size.Y))
	y := remainder / g.Size.X
	x := remainder % g.Size.X

	return math32.Vector3i{X: int32(x), Y: int32(y), Z: int32(z)}
}

// IsValidCoordinate checks if the coordinate is within grid bounds
func (g *VoxelGrid) IsValidCoordinate(coord math32.Vector3i) bool {
	return coord.X >= 0 && coord.X < g.Size.X &&
		coord.Y >= 0 && coord.Y < g.Size.Y &&
		coord.Z >= 0 && coord.Z < g.Size.Z
}

// GetVoxel returns the voxel at the given coordinate
func (g *VoxelGrid) GetVoxel(coord math32.Vector3i) *Voxel {
	index := g.GetIndex(coord)
	if index == -1 {
		return nil
	}
	return &g.Voxels[index]
}

// SetVoxel sets the voxel at the given coordinate
func (g *VoxelGrid) SetVoxel(coord math32.Vector3i, voxel Voxel) bool {
	index := g.GetIndex(coord)
	if index == -1 {
		return false
	}
	g.Voxels[index] = voxel
	return true
}

// WorldToVoxel converts world coordinates to voxel coordinates
func (g *VoxelGrid) WorldToVoxel(worldPos math32.Vector3) math32.Vector3i {
	localPos := worldPos.Sub(g.Origin)
	return math32.Vector3i{
		X: int32(localPos.X / g.VoxelSize),
		Y: int32(localPos.Y / g.VoxelSize),
		Z: int32(localPos.Z / g.VoxelSize),
	}
}

// VoxelToWorld converts voxel coordinates to world coordinates (center of voxel)
func (g *VoxelGrid) VoxelToWorld(voxelCoord math32.Vector3i) math32.Vector3 {
	return g.Origin.Add(math32.Vector3{
		X: (float32(voxelCoord.X) + 0.5) * g.VoxelSize,
		Y: (float32(voxelCoord.Y) + 0.5) * g.VoxelSize,
		Z: (float32(voxelCoord.Z) + 0.5) * g.VoxelSize,
	})
}

// VoxelToWorldCorner converts voxel coordinates to world coordinates (corner of voxel)
func (g *VoxelGrid) VoxelToWorldCorner(voxelCoord math32.Vector3i) math32.Vector3 {
	return g.Origin.Add(math32.Vector3{
		X: float32(voxelCoord.X) * g.VoxelSize,
		Y: float32(voxelCoord.Y) * g.VoxelSize,
		Z: float32(voxelCoord.Z) * g.VoxelSize,
	})
}

func (g *VoxelGrid) IsWorldWalkable(worldPos math32.Vector3) bool {
	voxelCoord := g.WorldToVoxel(worldPos)
	return g.IsWalkable(voxelCoord)
}

// IsWalkable checks if a voxel coordinate is walkable
func (g *VoxelGrid) IsWalkable(coord math32.Vector3i) bool {
	voxel := g.GetVoxel(coord)
	if voxel == nil {
		return false
	}
	return voxel.IsWalkable()
}

// Clear resets all voxels to empty
func (g *VoxelGrid) Clear() {
	for i := range g.Voxels {
		g.Voxels[i] = NewVoxel(VoxelEmpty)
	}
}

// GetVoxelCount returns the total number of voxels
func (g *VoxelGrid) GetVoxelCount() int {
	return len(g.Voxels)
}

// GetMemoryUsage returns approximate memory usage in bytes
func (g *VoxelGrid) GetMemoryUsage() int {
	return len(g.Voxels) * int(unsafe.Sizeof(Voxel(0)))
}

func (g *VoxelGrid) ToBitmap() math32.Bitmap {
	bitmap := math32.Bitmap{}
	for i, voxel := range g.Voxels {
		if voxel == Voxel(VoxelSolid) {
			bitmap.Set(uint32(i))
		}
	}
	return bitmap
}
