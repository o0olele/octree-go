package main

import (
	"fmt"
	"log"

	"github.com/o0olele/octree-go/builder"
	"github.com/o0olele/octree-go/math32"
	"github.com/o0olele/octree-go/octree"
	"github.com/o0olele/octree-go/query"
)

func main() {
	navData, err := builder.Load("navigation.bin")
	if err != nil {
		log.Fatalf("Failed to load navigation data: %v", err)
	}

	q, err := query.NewNavigationQuery(navData)
	if err != nil {
		log.Fatalf("Failed to create navigation query: %v", err)
	}
	q.SetAgent(octree.NewAgent(0.4, 1))

	path := q.FindPath(math32.Vector3{X: 0, Y: 0, Z: 0}, math32.Vector3{X: 10, Y: 10, Z: 10})

	fmt.Println(path)
}
