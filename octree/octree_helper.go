package octree

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
