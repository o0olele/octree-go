package query

import (
	"fmt"

	"github.com/o0olele/octree-go/builder"
)

// LoadAndQuery loads the navigation data and creates the queryer (one-stop)
func LoadAndQuery(filename string) (*NavigationQuery, error) {
	// Load the navigation data
	navData, err := builder.Load(filename)
	if err != nil {
		return nil, fmt.Errorf("failed to load navigation data: %v", err)
	}

	// Create the queryer
	query, err := NewNavigationQuery(navData)
	if err != nil {
		return nil, fmt.Errorf("failed to create navigation query: %v", err)
	}

	return query, nil
}
