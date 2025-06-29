package octree

import "math/bits"

// MortonCode 表示3D点的Morton编码
type MortonCode uint64

// EncodeMorton3D 将3D坐标编码为Morton码
func EncodeMorton3D(x, y, z uint32) MortonCode {
	return MortonCode(splitBy3(x) | (splitBy3(y) << 1) | (splitBy3(z) << 2))
}

// splitBy3 将10位整数扩展为30位，在每位后插入2个零
func splitBy3(v uint32) uint64 {
	x := uint64(v) & 0x1fffff // 只看前21位
	x = (x | x<<32) & 0x1f00000000ffff
	x = (x | x<<16) & 0x1f0000ff0000ff
	x = (x | x<<8) & 0x100f00f00f00f00f
	x = (x | x<<4) & 0x10c30c30c30c30c3
	x = (x | x<<2) & 0x1249249249249249
	return x
}

// DecodeMorton3D 将Morton码解码为3D坐标
func DecodeMorton3D(morton MortonCode) (uint32, uint32, uint32) {
	x := compact1By2(uint64(morton))
	y := compact1By2(uint64(morton) >> 1)
	z := compact1By2(uint64(morton) >> 2)
	return uint32(x), uint32(y), uint32(z)
}

// compact1By2 反向操作，从每3位中提取1位
func compact1By2(x uint64) uint64 {
	x &= 0x1249249249249249
	x = (x ^ (x >> 2)) & 0x10c30c30c30c30c3
	x = (x ^ (x >> 4)) & 0x100f00f00f00f00f
	x = (x ^ (x >> 8)) & 0x1f0000ff0000ff
	x = (x ^ (x >> 16)) & 0x1f00000000ffff
	x = (x ^ (x >> 32)) & 0x1fffff
	return x
}

// Vector3ToMorton 将Vector3转换为Morton码
func Vector3ToMorton(pos Vector3, bounds AABB, resolution uint32) MortonCode {
	// 归一化到[0,1]范围
	size := bounds.Size()
	normalizedX := (pos.X - bounds.Min.X) / size.X
	normalizedY := (pos.Y - bounds.Min.Y) / size.Y
	normalizedZ := (pos.Z - bounds.Min.Z) / size.Z

	// 转换为整数坐标
	x := uint32(normalizedX * float64(resolution))
	y := uint32(normalizedY * float64(resolution))
	z := uint32(normalizedZ * float64(resolution))

	// 确保在范围内
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

// MortonToVector3 将Morton码转换为Vector3
func MortonToVector3(morton MortonCode, bounds AABB, resolution uint32) Vector3 {
	x, y, z := DecodeMorton3D(morton)

	size := bounds.Size()
	return Vector3{
		X: bounds.Min.X + (float64(x)/float64(resolution))*size.X,
		Y: bounds.Min.Y + (float64(y)/float64(resolution))*size.Y,
		Z: bounds.Min.Z + (float64(z)/float64(resolution))*size.Z,
	}
}

// MortonDistance 计算两个Morton码之间的"距离"
func MortonDistance(a, b MortonCode) uint64 {
	return uint64(a ^ b)
}

// GetMortonNeighbors 获取Morton码的相邻码（6个面方向）
func GetMortonNeighbors(morton MortonCode, bounds AABB, resolution uint32) []MortonCode {
	x, y, z := DecodeMorton3D(morton)
	neighbors := make([]MortonCode, 0, 6)

	// 6个面方向的偏移
	offsets := []struct{ dx, dy, dz int32 }{
		{1, 0, 0}, {-1, 0, 0}, // X轴
		{0, 1, 0}, {0, -1, 0}, // Y轴
		{0, 0, 1}, {0, 0, -1}, // Z轴
	}

	for _, offset := range offsets {
		nx := int32(x) + offset.dx
		ny := int32(y) + offset.dy
		nz := int32(z) + offset.dz

		// 边界检查
		if nx >= 0 && nx < int32(resolution) &&
			ny >= 0 && ny < int32(resolution) &&
			nz >= 0 && nz < int32(resolution) {
			neighbors = append(neighbors, EncodeMorton3D(uint32(nx), uint32(ny), uint32(nz)))
		}
	}

	return neighbors
}

// CommonPrefixLength 计算两个Morton码的公共前缀长度
func CommonPrefixLength(a, b MortonCode) int {
	if a == b {
		return 64 // 完全相同
	}
	return bits.LeadingZeros64(uint64(a ^ b))
}
