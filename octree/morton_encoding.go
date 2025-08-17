package octree

import (
	"math/bits"

	"github.com/o0olele/octree-go/geometry"
	"github.com/o0olele/octree-go/math32"
)

// MortonCode represents the Morton code of a 3D point
type MortonCode uint64

// EncodeMorton3D encodes a 3D coordinate to a Morton code
func EncodeMorton3D(x, y, z uint32) MortonCode {
	return MortonCode(splitBy3(x) | (splitBy3(y) << 1) | (splitBy3(z) << 2))
}

// splitBy3 expands a 10-bit integer to 30 bits, inserting 2 zeros after each bit
func splitBy3(v uint32) uint64 {
	x := uint64(v) & 0x1fffff // Only look at the first 21 bits
	x = (x | x<<32) & 0x1f00000000ffff
	x = (x | x<<16) & 0x1f0000ff0000ff
	x = (x | x<<8) & 0x100f00f00f00f00f
	x = (x | x<<4) & 0x10c30c30c30c30c3
	x = (x | x<<2) & 0x1249249249249249
	return x
}

// DecodeMorton3D decodes a Morton code to a 3D coordinate
func DecodeMorton3D(morton MortonCode) (uint32, uint32, uint32) {
	x := compact1By2(uint64(morton))
	y := compact1By2(uint64(morton) >> 1)
	z := compact1By2(uint64(morton) >> 2)
	return uint32(x), uint32(y), uint32(z)
}

// compact1By2 is the reverse operation, extracting 1 bit from each 3 bits
func compact1By2(x uint64) uint64 {
	x &= 0x1249249249249249
	x = (x ^ (x >> 2)) & 0x10c30c30c30c30c3
	x = (x ^ (x >> 4)) & 0x100f00f00f00f00f
	x = (x ^ (x >> 8)) & 0x1f0000ff0000ff
	x = (x ^ (x >> 16)) & 0x1f00000000ffff
	x = (x ^ (x >> 32)) & 0x1fffff
	return x
}

// Vector3ToMorton converts a Vector3 to a Morton code
func Vector3ToMorton(pos math32.Vector3, bounds geometry.AABB, resolution uint32) MortonCode {
	// Normalize to the [0,1] range
	size := bounds.Size()
	normalizedX := (pos.X - bounds.Min.X) / size.X
	normalizedY := (pos.Y - bounds.Min.Y) / size.Y
	normalizedZ := (pos.Z - bounds.Min.Z) / size.Z

	// Convert to integer coordinates
	x := uint32(normalizedX * float32(resolution))
	y := uint32(normalizedY * float32(resolution))
	z := uint32(normalizedZ * float32(resolution))

	// Ensure within range
	if x >= resolution {
		x = resolution - 1
	}
	if y >= resolution {
		y = resolution - 1
	}
	if z >= resolution {
		z = resolution - 1
	}

	return EncodeMorton3D(x, y, z)
}

// MortonToVector3 converts a Morton code to a Vector3
func MortonToVector3(morton MortonCode, bounds geometry.AABB, resolution uint32) math32.Vector3 {
	x, y, z := DecodeMorton3D(morton)

	size := bounds.Size()
	return math32.Vector3{
		X: bounds.Min.X + (float32(x)/float32(resolution))*size.X,
		Y: bounds.Min.Y + (float32(y)/float32(resolution))*size.Y,
		Z: bounds.Min.Z + (float32(z)/float32(resolution))*size.Z,
	}
}

// MortonDistance calculates the "distance" between two Morton codes
func MortonDistance(a, b MortonCode) uint64 {
	return uint64(a ^ b)
}

// GetMortonNeighbors gets the neighboring codes of a Morton code (6 faces)
func GetMortonNeighbors(morton MortonCode, bounds geometry.AABB, resolution uint32) []MortonCode {
	x, y, z := DecodeMorton3D(morton)
	neighbors := make([]MortonCode, 0, 6)

	// Offsets for 6 face directions
	offsets := []struct{ dx, dy, dz int32 }{
		{1, 0, 0}, {-1, 0, 0}, // X axis
		{0, 1, 0}, {0, -1, 0}, // Y axis
		{0, 0, 1}, {0, 0, -1}, // Z axis
	}

	for _, offset := range offsets {
		nx := int32(x) + offset.dx
		ny := int32(y) + offset.dy
		nz := int32(z) + offset.dz

		// Boundary check
		if nx >= 0 && nx < int32(resolution) &&
			ny >= 0 && ny < int32(resolution) &&
			nz >= 0 && nz < int32(resolution) {
			neighbors = append(neighbors, EncodeMorton3D(uint32(nx), uint32(ny), uint32(nz)))
		}
	}

	return neighbors
}

// CommonPrefixLength calculates the common prefix length of two Morton codes
func CommonPrefixLength(a, b MortonCode) int {
	if a == b {
		return 64 // Completely the same
	}
	return bits.LeadingZeros64(uint64(a ^ b))
}
