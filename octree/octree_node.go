package octree

import "github.com/o0olele/octree-go/geometry"

const (
	// 1bit isLeaf 1bit isOccupied
	FlagsDefault  uint8 = 0b10
	FlagsLeaf     uint8 = 0b10
	FlagsOccupied uint8 = 0b01
)

// OctreeNode 八叉树节点
type OctreeNode struct {
	Bounds    geometry.AABB        `json:"bounds"`
	Flags     uint8                `json:"flags"` // 1bit isLeaf 1bit isOccupied
	Depth     uint8                `json:"depth"`
	Triangles []*geometry.Triangle `json:"-"`
	Children  [8]*OctreeNode       `json:"children,omitempty"`
}

func (node *OctreeNode) SetOccupied(occupied bool) {
	if occupied {
		node.Flags |= FlagsOccupied
	} else {
		node.Flags &= 0b11111110
	}
}

func (node *OctreeNode) SetLeaf(isLeaf bool) {
	if isLeaf {
		node.Flags |= FlagsLeaf
	} else {
		node.Flags &= 0b11111101
	}
}

func (node *OctreeNode) IsLeaf() bool {
	return node.Flags&FlagsLeaf == FlagsLeaf
}

func (node *OctreeNode) IsOccupied() bool {
	return node.Flags&FlagsOccupied == FlagsOccupied
}
