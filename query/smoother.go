package query

import (
	"fmt"

	"github.com/o0olele/octree-go/geometry"
	"github.com/o0olele/octree-go/math32"
	"github.com/o0olele/octree-go/octree"
)

// SmoothPath smooths the path
func (nq *NavigationQuery) SmoothPath(pathNodes []*octree.PathNode) []math32.Vector3 {

	// Using cross point
	if nq.pathPreferences.UsingCrossPoint {
		return nq.optimizePathWithLineOfSight(nq.getCrossingPath(pathNodes))
	}

	// Using Douglas-Peucker
	if nq.pathPreferences.UsingDouglasPeucker {
		return nq.optimizePathWithLineOfSight(nq.douglasPeucker(nq.getCenterPath(pathNodes), 0.2))
	}

	if len(pathNodes) <= 2 {
		result := make([]math32.Vector3, len(pathNodes))
		for i, node := range pathNodes {
			result[i] = node.Center
		}
		return result
	}

	smoothed := []math32.Vector3{pathNodes[0].Center}
	current := 0

	navData := nq.navData
	for current < len(pathNodes)-1 {
		// Using binary search to find the farthest visible node, reducing the number of checks
		low := current + 1
		high := len(pathNodes) - 1
		farthest := current + 1
		for low <= high {
			mid := (low + high) / 2
			if navData.IsPathClear(nq.agent, pathNodes[current].Center, pathNodes[mid].Center) {
				farthest = mid
				low = mid + 1
			} else {
				high = mid - 1
			}
		}

		smoothed = append(smoothed, pathNodes[farthest].Center)
		current = farthest
	}

	return smoothed
}

// getCenterPath gets the center path
func (nq *NavigationQuery) getCenterPath(pathNodes []*octree.PathNode) []math32.Vector3 {
	result := make([]math32.Vector3, len(pathNodes))
	for i, node := range pathNodes {
		result[i] = node.Center
	}
	return result
}

// optimizePathWithLineOfSight smooths the path with line of sight
func (nq *NavigationQuery) optimizePathWithLineOfSight(pathNodes []math32.Vector3) []math32.Vector3 {
	if len(pathNodes) <= 2 {
		return pathNodes
	}

	smoothed := []math32.Vector3{pathNodes[0]}
	current := 0

	navData := nq.navData
	for current < len(pathNodes)-1 {
		// Using binary search to find the farthest reachable node, reducing the number of checks
		low := current + 1
		high := len(pathNodes) - 1
		farthest := current + 1
		for low <= high {
			mid := (low + high) / 2
			if navData.IsPathClear(nq.agent, pathNodes[current], pathNodes[mid]) {
				farthest = mid
				low = mid + 1
			} else {
				high = mid - 1
			}
		}

		smoothed = append(smoothed, pathNodes[farthest])
		current = farthest
	}

	return smoothed
}

// getCrossingPath converts the node path to the world coordinate path, using the boundary crossing point instead of the center point
func (nq *NavigationQuery) getCrossingPath(nodePath []*octree.PathNode) []math32.Vector3 {
	if len(nodePath) == 0 {
		return []math32.Vector3{}
	}

	var path []math32.Vector3

	// 1. Add the start point
	path = append(path, nodePath[0].Center)

	// 2. If there is only one node, directly connect the start point to the end point
	if len(nodePath) <= 3 {
		path = append(path, nodePath[1].Center)
		path = append(path, nodePath[2].Center)
		return path
	}

	// 3. Process the boundary crossing points between the intermediate nodes
	for i := 1; i < len(nodePath)-2; i++ {
		node1 := nodePath[i]
		node2 := nodePath[i+1]

		// Calculate the boundary crossing point between the two nodes
		crossingPoint := nq.calculateCrossingPoint(node1, node2)
		path = append(path, crossingPoint)
	}

	// 4. Add the end point
	path = append(path, nodePath[len(nodePath)-1].Center)

	return path
}

// calculateCrossingPoint calculates the boundary crossing point between the two adjacent nodes
func (nq *NavigationQuery) calculateCrossingPoint(node1, node2 *octree.PathNode) math32.Vector3 {
	// Determine the shared boundary plane
	planeAxis := -1 // 0=X, 1=Y, 2=Z
	planePos := float32(0.0)

	if floatEqual(node1.Bounds.Max.X, node2.Bounds.Min.X) { // Check the x direction
		planeAxis = 0
		planePos = node1.Bounds.Max.X
	} else if floatEqual(node1.Bounds.Min.X, node2.Bounds.Max.X) {
		planeAxis = 0
		planePos = node1.Bounds.Min.X
	} else if floatEqual(node1.Bounds.Max.Y, node2.Bounds.Min.Y) { // Check the y direction
		planeAxis = 1
		planePos = node1.Bounds.Max.Y
	} else if floatEqual(node1.Bounds.Min.Y, node2.Bounds.Max.Y) {
		planeAxis = 1
		planePos = node1.Bounds.Min.Y
	} else if floatEqual(node1.Bounds.Max.Z, node2.Bounds.Min.Z) { // Check the z direction
		planeAxis = 2
		planePos = node1.Bounds.Max.Z
	} else if floatEqual(node1.Bounds.Min.Z, node2.Bounds.Max.Z) {
		planeAxis = 2
		planePos = node1.Bounds.Min.Z
	} else {
		// If no shared plane is found, it may be an error, return node1.Center as the default
		fmt.Println("No shared plane found", node1.Center, node2.Center)
		return node1.Center
	}

	// Calculate the intersection of the ray from node1.Center to node2.Center with the plane
	dir := node2.Center.Sub(node1.Center)

	var t float32
	switch planeAxis {
	case 0: // X plane
		if math32.Abs(dir.X) < 1e-6 {
			// Avoid division by zero
			return node1.Center
		}
		t = (planePos - node1.Center.X) / dir.X
	case 1: // Y plane
		if math32.Abs(dir.Y) < 1e-6 {
			return node1.Center
		}
		t = (planePos - node1.Center.Y) / dir.Y
	case 2: // Z plane
		if math32.Abs(dir.Z) < 1e-6 {
			return node1.Center
		}
		t = (planePos - node1.Center.Z) / dir.Z
	}

	// Ensure t is within the range [0,1]
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}

	// Calculate the intersection point
	point := node1.Center.Add(dir.Mul(t))

	// Check if the intersection point is within the actual shared area of the two nodes (handling different node sizes)
	switch planeAxis {
	case 0: // X plane, check the Y and Z ranges
		minY := math32.Max(node1.Bounds.Min.Y, node2.Bounds.Min.Y)
		maxY := math32.Min(node1.Bounds.Max.Y, node2.Bounds.Max.Y)
		minZ := math32.Max(node1.Bounds.Min.Z, node2.Bounds.Min.Z)
		maxZ := math32.Min(node1.Bounds.Max.Z, node2.Bounds.Max.Z)

		if point.Y < minY || point.Y > maxY || point.Z < minZ || point.Z > maxZ {
			// The intersection point is not within the shared area, adjust to the nearest valid position
			point.Y = clamp(point.Y, minY, maxY)
			point.Z = clamp(point.Z, minZ, maxZ)
		}
	case 1: // Y plane, check the X and Z ranges
		minX := math32.Max(node1.Bounds.Min.X, node2.Bounds.Min.X)
		maxX := math32.Min(node1.Bounds.Max.X, node2.Bounds.Max.X)
		minZ := math32.Max(node1.Bounds.Min.Z, node2.Bounds.Min.Z)
		maxZ := math32.Min(node1.Bounds.Max.Z, node2.Bounds.Max.Z)

		if point.X < minX || point.X > maxX || point.Z < minZ || point.Z > maxZ {
			point.X = clamp(point.X, minX, maxX)
			point.Z = clamp(point.Z, minZ, maxZ)
		}
	case 2: // Z plane, check the X and Y ranges
		minX := math32.Max(node1.Bounds.Min.X, node2.Bounds.Min.X)
		maxX := math32.Min(node1.Bounds.Max.X, node2.Bounds.Max.X)
		minY := math32.Max(node1.Bounds.Min.Y, node2.Bounds.Min.Y)
		maxY := math32.Min(node1.Bounds.Max.Y, node2.Bounds.Max.Y)

		if point.X < minX || point.X > maxX || point.Y < minY || point.Y > maxY {
			point.X = clamp(point.X, minX, maxX)
			point.Y = clamp(point.Y, minY, maxY)
		}
	}

	return point
}

// floatEqual compares two float32 values (considering precision issues)
func floatEqual(a, b float32) bool {
	return math32.Abs(a-b) < 1e-6
}

// clamp limits the value within the range [min, max]
func clamp(value, min, max float32) float32 {
	if value < min {
		return min
	}
	if value > max {
		return max
	}
	return value
}

// douglasPeucker smooths the path with Douglas-Peucker algorithm
func (nq *NavigationQuery) douglasPeucker(points []math32.Vector3, epsilon float32) []math32.Vector3 {
	n := len(points)
	if n <= 2 {
		return points
	}

	// Iterative version, avoiding stack overflow caused by deep recursion
	keep := make([]bool, n)
	keep[0] = true
	keep[n-1] = true

	type segment struct{ start, end int }
	stack := []segment{{0, n - 1}}

	for len(stack) > 0 {
		seg := stack[len(stack)-1]
		stack = stack[:len(stack)-1]

		s := seg.start
		e := seg.end
		if e <= s+1 {
			continue
		}

		// Find the point farthest from the line segment connecting the start and end
		var maxDist float32
		maxIdx := -1
		for i := s + 1; i < e; i++ {
			dist := geometry.PointToLineSegmentDistance(points[i], points[s], points[e])
			if dist > maxDist {
				maxDist = dist
				maxIdx = i
			}
		}

		// Line simplification judgment: distance threshold + line of sight reachable
		if maxDist < epsilon && nq.navData.IsPathClear(nq.agent, points[s], points[e]) {
			// Keep the endpoints, discard the intermediate points
			continue
		}

		if maxIdx != -1 {
			keep[maxIdx] = true
			// Split into two segments, push to the stack for further processing
			stack = append(stack, segment{s, maxIdx}, segment{maxIdx, e})
		}
	}

	// Output the retained points (keeping the order)
	result := make([]math32.Vector3, 0, n)
	for i := 0; i < n; i++ {
		if keep[i] {
			result = append(result, points[i])
		}
	}
	return result
}
