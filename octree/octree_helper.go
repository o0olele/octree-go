package octree

import (
	"encoding/json"

	"github.com/o0olele/octree-go/geometry"
)

type OctreeNodeBuildHelper struct {
	TriangleIndexes []uint32
	ChildrenIndexes [8]uint32
	Index           int
}

type OctreeBuildHelper struct {
	Nodes map[*OctreeNode]*OctreeNodeBuildHelper
}

func NewOctreeBuildHelper() *OctreeBuildHelper {
	return &OctreeBuildHelper{
		Nodes: make(map[*OctreeNode]*OctreeNodeBuildHelper),
	}
}

func (h *OctreeBuildHelper) AddTriangleIndex(node *OctreeNode, triangleIndex uint32) {
	nodeHelper := h.GetNode(node)
	nodeHelper.TriangleIndexes = append(nodeHelper.TriangleIndexes, triangleIndex)
}

func (h *OctreeBuildHelper) AddChildren(parent *OctreeNode, children [8]*OctreeNode) {
	parentNodeHelper := h.Nodes[parent]
	if parentNodeHelper == nil {
		panic("parent node not found")
	}
	for i, child := range children {
		nodeHelper := h.GetNode(child)
		parentNodeHelper.ChildrenIndexes[i] = uint32(nodeHelper.Index)
	}
}

func (h *OctreeBuildHelper) GetNode(node *OctreeNode) *OctreeNodeBuildHelper {
	nodeHelper, ok := h.Nodes[node]
	if !ok {
		nodeHelper = &OctreeNodeBuildHelper{
			TriangleIndexes: make([]uint32, 0),
			Index:           len(h.Nodes),
		}
		h.Nodes[node] = nodeHelper
	}
	return nodeHelper
}

// 用于JSON序列化的简化结构
type OctreeExport struct {
	Root     *OctreeNodeExport `json:"root"`
	MaxDepth uint8             `json:"max_depth"`
	MinSize  float32           `json:"min_size"`
}

type OctreeNodeExport struct {
	Bounds     geometry.AABB       `json:"bounds"`
	Children   []*OctreeNodeExport `json:"children,omitempty"`
	IsLeaf     bool                `json:"is_leaf"`
	IsOccupied bool                `json:"is_occupied"`
	Depth      uint8               `json:"depth"`
}

// ToJSON 导出八叉树为JSON
func (o *Octree) ToJSON() ([]byte, error) {
	export := &OctreeExport{
		Root:     o.nodeToExport(o.Root),
		MaxDepth: o.MaxDepth,
		MinSize:  o.MinSize,
	}
	return json.Marshal(export)
}

func (o *Octree) nodeToExport(node *OctreeNode) *OctreeNodeExport {
	if node == nil {
		return nil
	}

	export := &OctreeNodeExport{
		Bounds:     node.Bounds,
		IsLeaf:     node.IsLeaf(),
		IsOccupied: node.IsOccupied(),
		Depth:      node.Depth,
	}

	if !node.IsLeaf() {
		for _, child := range node.Children {
			export.Children = append(export.Children, o.nodeToExport(child))
		}
	}

	return export
}
