package main

import (
	"log"

	"github.com/o0olele/octree-go/builder"
	"github.com/o0olele/octree-go/geometry"
	"github.com/o0olele/octree-go/math32"
	"github.com/o0olele/octree-go/octree"
)

func main() {

	bounds := geometry.AABB{
		Min: math32.Vector3{X: -10, Y: -10, Z: -10},
		Max: math32.Vector3{X: 10, Y: 10, Z: 10},
	}

	// Create a new navigation builder
	navBuilder := builder.NewBuilder(bounds, 10, 1, 1)

	// Use voxel for collision detection
	navBuilder.SetUseVoxel(true)

	// Add obstacles
	navBuilder.AddTriangle(geometry.Triangle{
		A: math32.Vector3{X: 0, Y: 0, Z: 0},
		B: math32.Vector3{X: 1, Y: 0, Z: 0},
		C: math32.Vector3{X: 0, Y: 1, Z: 0},
	})

	// Create a new agent
	agent := octree.NewAgent(0.4, 1)

	// Build the navigation data
	navData, err := navBuilder.Build(agent)
	if err != nil {
		log.Fatalf("Failed to build navigation data: %v", err)
	}

	// Save the navigation data to a file
	builder.Save(navData, "navigation.bin")
}
