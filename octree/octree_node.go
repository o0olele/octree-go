package octree

import "github.com/o0olele/octree-go/geometry"

const (
	// 1bit isLeaf 1bit isOccupied
	FlagsDefault  uint8 = 0b10
	FlagsLeaf     uint8 = 0b10
	FlagsOccupied uint8 = 0b01
)

// OctreeNode is the octree node
type OctreeNode struct {
	Bounds    geometry.AABB        `json:"bounds"`             // bounds of the node
	Flags     uint8                `json:"flags"`              // 1bit isLeaf 1bit isOccupied
	Depth     uint8                `json:"depth"`              // depth of the node
	Triangles []*geometry.Triangle `json:"-"`                  // triangles in the node
	Children  [8]*OctreeNode       `json:"children,omitempty"` // children nodes
}

// SetOccupied sets the occupied flag
func (node *OctreeNode) SetOccupied(occupied bool) {
	if occupied {
		node.Flags |= FlagsOccupied
	} else {
		node.Flags &= 0b11111110
	}
}

// SetLeaf sets the leaf flag
func (node *OctreeNode) SetLeaf(isLeaf bool) {
	if isLeaf {
		node.Flags |= FlagsLeaf
	} else {
		node.Flags &= 0b11111101
	}
}

// IsLeaf checks if the node is a leaf
func (node *OctreeNode) IsLeaf() bool {
	return node.Flags&FlagsLeaf == FlagsLeaf
}

// IsOccupied checks if the node is occupied
func (node *OctreeNode) IsOccupied() bool {
	return node.Flags&FlagsOccupied == FlagsOccupied
}
