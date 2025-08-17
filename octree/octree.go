package octree

import (
	"github.com/o0olele/octree-go/geometry"
	"github.com/o0olele/octree-go/math32"
)

// Agent represents the agent for pathfinding, represented by a capsule
type Agent struct {
	Radius  float32           `json:"radius"` // The radius of the capsule
	Height  float32           `json:"height"` // The height of the capsule (excluding the hemispheres at the ends)
	Capsule *geometry.Capsule `json:"capsule"`
}

// NewAgent creates a new Agent
func NewAgent(radius, height float32) *Agent {
	return &Agent{
		Radius: radius,
		Height: height,
	}
}

// GetCapsule gets the capsule representation of the Agent at the given position
func (a *Agent) GetCapsule(position math32.Vector3) *geometry.Capsule {
	if a.Capsule != nil {
		return a.Capsule
	}
	// The axis of the capsule is along the Y axis
	start := math32.Vector3{
		X: position.X,
		Y: position.Y + a.Radius,
		Z: position.Z,
	}
	end := math32.Vector3{
		X: position.X,
		Y: position.Y + a.Height + a.Radius,
		Z: position.Z,
	}
	return &geometry.Capsule{
		Start:  start,
		End:    end,
		Radius: a.Radius,
	}
}

// GetBounds gets the bounding box of the Agent at the given position
func (a *Agent) GetBounds(position math32.Vector3) geometry.AABB {
	return a.GetCapsule(position).GetBounds()
}

// Octree is the main structure of the octree
type Octree struct {
	Root        *OctreeNode `json:"root"`
	MaxDepth    uint8       `json:"max_depth"`
	MinSize     float32     `json:"min_size"`
	triangles   []geometry.Triangle
	buildHelper *OctreeBuildHelper
}

// NewOctree creates a new octree
func NewOctree(bounds geometry.AABB, maxDepth uint8, minSize float32) *Octree {
	return &Octree{
		Root: &OctreeNode{
			Bounds: bounds,
			Flags:  FlagsDefault,
			Depth:  0,
		},
		MaxDepth:  maxDepth,
		MinSize:   minSize,
		triangles: make([]geometry.Triangle, 0),
	}
}

// SetTriangles sets the triangles of the octree
func (o *Octree) SetTriangles(triangles []geometry.Triangle) {
	o.triangles = triangles
}

// SetHelper sets the helper of the octree
func (o *Octree) SetHelper(helper *OctreeBuildHelper) {
	o.buildHelper = helper
}

// AddTriangle adds a triangle to the octree
func (o *Octree) AddTriangle(triangle geometry.Triangle) {
	o.triangles = append(o.triangles, triangle)
}

// GetTrianglesNum gets the number of triangles in the octree
func (o *Octree) GetTrianglesNum() int {
	return len(o.triangles)
}

// GetTriangles gets the triangles in the octree
func (o *Octree) GetTriangles() []geometry.Triangle {
	return o.triangles
}

// AddTriangles adds triangles to the octree
func (o *Octree) AddTriangles(triangles []geometry.Triangle) {
	o.triangles = append(o.triangles, triangles...)
}

// insertTriangle inserts a triangle into the octree
func (o *Octree) insertTriangle(node *OctreeNode, triangleIndex int, triangle *geometry.Triangle) {
	if !triangle.IntersectsAABB(node.Bounds) {
		return
	}

	node.Triangles = append(node.Triangles, triangle)
	node.SetOccupied(true)

	if o.buildHelper != nil {
		o.buildHelper.AddTriangleIndex(node, uint32(triangleIndex))
	}

	// If the node is a leaf and the subdivision condition is met, subdivide
	if node.IsLeaf() && node.Depth < o.MaxDepth && node.Bounds.Size().X > o.MinSize {
		o.subdivide(node)
	}

	// If the node is not a leaf, recursively insert into the child nodes
	if !node.IsLeaf() {
		for _, child := range node.Children {
			if child != nil {
				o.insertTriangle(child, triangleIndex, triangle)
			}
		}
	}
}

// subdivide subdivides the node
func (o *Octree) subdivide(node *OctreeNode) {
	center := node.Bounds.Center()
	size := node.Bounds.Size().Scale(0.5)

	positions := [8]math32.Vector3{
		{X: node.Bounds.Min.X, Y: node.Bounds.Min.Y, Z: node.Bounds.Min.Z}, // 000
		{X: center.X, Y: node.Bounds.Min.Y, Z: node.Bounds.Min.Z},          // 100
		{X: node.Bounds.Min.X, Y: center.Y, Z: node.Bounds.Min.Z},          // 010
		{X: center.X, Y: center.Y, Z: node.Bounds.Min.Z},                   // 110
		{X: node.Bounds.Min.X, Y: node.Bounds.Min.Y, Z: center.Z},          // 001
		{X: center.X, Y: node.Bounds.Min.Y, Z: center.Z},                   // 101
		{X: node.Bounds.Min.X, Y: center.Y, Z: center.Z},                   // 011
		{X: center.X, Y: center.Y, Z: center.Z},                            // 111
	}

	for i := 0; i < 8; i++ {
		childBounds := geometry.AABB{
			Min: positions[i],
			Max: positions[i].Add(size),
		}

		node.Children[i] = &OctreeNode{
			Bounds: childBounds,
			Flags:  FlagsDefault,
			Depth:  node.Depth + 1,
		}
	}
	if o.buildHelper != nil {
		o.buildHelper.AddChildren(node, node.Children)
	}

	node.SetLeaf(false)
}

// Build builds the octree
func (o *Octree) Build() {
	for idx := range o.triangles {
		o.insertTriangle(o.Root, idx, &o.triangles[idx])
	}
}

// IsOccupied checks if the specified position is occupied
func (o *Octree) IsOccupied(point math32.Vector3) bool {
	return o.isOccupiedRecursive(o.Root, point)
}

// IsAgentOccupied checks if the Agent at the specified position collides with the geometry
func (o *Octree) IsAgentOccupied(agent *Agent, position math32.Vector3) bool {
	if agent == nil {
		return o.IsOccupied(position)
	}

	agentCapsule := agent.GetCapsule(position)
	return o.isAgentOccupiedRecursive(o.Root, agentCapsule)
}

// IsAgentOccupiedByNodeGeometry checks if the Agent capsule collides with the geometry in the node
func (o *Octree) IsAgentOccupiedByNodeGeometry(agentCapsule *geometry.Capsule, node *OctreeNode) bool {
	// Check if the Agent capsule collides with the geometry in the node
	for _, triangle := range node.Triangles {
		if o.capsuleIntersectsTriangle(agentCapsule, triangle) {
			return true
		}
	}

	return false
}

// isAgentOccupiedRecursive checks if the Agent's bounding box intersects with the node
func (o *Octree) isAgentOccupiedRecursive(node *OctreeNode, agentCapsule *geometry.Capsule) bool {
	// Check if the Agent's bounding box intersects with the node
	agentBounds := agentCapsule.GetBounds()
	if !o.aabbIntersects(node.Bounds, agentBounds) {
		return false
	}

	if node.IsLeaf() {
		if !node.IsOccupied() {
			return false
		}
		return o.IsAgentOccupiedByNodeGeometry(agentCapsule, node)
	}

	// Recursively check the child nodes
	for _, child := range node.Children {
		if child != nil && o.isAgentOccupiedRecursive(child, agentCapsule) {
			return true
		}
	}

	return false
}

// aabbIntersects checks if two AABB intersect
func (o *Octree) aabbIntersects(a, b geometry.AABB) bool {
	return !(a.Max.X < b.Min.X || a.Min.X > b.Max.X ||
		a.Max.Y < b.Min.Y || a.Min.Y > b.Max.Y ||
		a.Max.Z < b.Min.Z || a.Min.Z > b.Max.Z)
}

// capsuleIntersectsTriangle checks if the capsule intersects with the triangle
func (o *Octree) capsuleIntersectsTriangle(capsule *geometry.Capsule, triangle *geometry.Triangle) bool {
	intersect, _, _, _ := geometry.CapsuleTriangleIntersect(capsule, triangle.A, triangle.B, triangle.C)
	return intersect
}

// isOccupiedRecursive checks if the specified position is occupied
func (o *Octree) isOccupiedRecursive(node *OctreeNode, point math32.Vector3) bool {
	if !node.Bounds.Contains(point) {
		return false
	}

	if node.IsLeaf() {
		if !node.IsOccupied() {
			return false
		}
		// 检查所有几何体
		for _, triangle := range node.Triangles {
			if triangle.ContainsPoint(point) {
				return true
			}
		}
		return false
	}

	// Recursively check the child nodes
	for _, child := range node.Children {
		if child != nil && o.isOccupiedRecursive(child, point) {
			return true
		}
	}

	return false
}

// isPathClear checks if the path between two points is clear
func (o *Octree) IsPathClear(agent *Agent, start, end math32.Vector3) bool {

	// Calculate the direction vector and distance
	direction := end.Sub(start)
	distance := direction.Length()

	if distance < 0.001 { // The distance is too close, considered to be the same point
		return true
	}

	// Normalize the direction vector
	direction = direction.Scale(1.0 / distance)

	agentRadius := float32(0.4)
	if agent != nil {
		agentRadius = agent.Radius
	}
	// Use an appropriate step size for sampling checks
	stepSize := math32.Max(0.1, agentRadius*0.6)
	steps := math32.CeilToInt(distance / stepSize)

	// Sample along the path for detection
	for i := 0; i <= steps; i++ {
		t := float32(i) / float32(steps)
		samplePoint := start.Add(direction.Scale(distance * t))

		// 如果有Agent半径，还需要检查Agent碰撞
		occupied := o.IsAgentOccupied(agent, samplePoint)
		if occupied {
			return false
		}

	}

	return true
}

// Raycast performs raycasting in the octree, returning whether it hits, the hit distance, the hit point, and the hit triangle
// direction can be of any length, it will be normalized internally; maxDistance is the maximum detection distance (<=0 is considered unlimited)
func (o *Octree) Raycast(origin, direction math32.Vector3, maxDistance float32) (bool, float32, math32.Vector3, *geometry.Triangle) {
	dirLen := direction.Length()
	if dirLen <= 1e-6 {
		return false, 0, origin, nil
	}
	dir := direction.Scale(1.0 / dirLen)
	if maxDistance <= 0 {
		maxDistance = math32.MaxFloat32
	}

	// Fast rejection: if it does not intersect with the root node, it fails directly
	if _, _, ok := geometry.RayAABB(origin, dir, o.Root.Bounds); !ok {
		return false, 0, origin, nil
	}

	bestT := maxDistance
	var bestTri *geometry.Triangle
	o.raycastNode(o.Root, origin, dir, maxDistance, &bestT, &bestTri)
	if bestTri == nil {
		return false, 0, origin, nil
	}
	hitPoint := origin.Add(dir.Scale(bestT))
	return true, bestT, hitPoint, bestTri
}

// raycastNode performs raycasting in the node
func (o *Octree) raycastNode(node *OctreeNode, origin, dir math32.Vector3, maxT float32, bestT *float32, bestTri **geometry.Triangle) {
	tmin, _, ok := geometry.RayAABB(origin, dir, node.Bounds)
	if !ok || tmin > *bestT || tmin > maxT {
		return
	}
	if node.IsLeaf() {
		for _, tri := range node.Triangles {
			if hit, t := geometry.RayTriangle(origin, dir, tri); hit && t < *bestT && t <= maxT {
				*bestT = t
				*bestTri = tri
			}
		}
		return
	}

	// Recursively check the child nodes
	for _, child := range node.Children {
		if child != nil {
			o.raycastNode(child, origin, dir, maxT, bestT, bestTri)
		}
	}
}
