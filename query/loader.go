package query

import (
	"fmt"

	"github.com/o0olele/octree-go/builder"
)

// LoadAndQuery 加载导航数据并创建查询器（一步到位）
func LoadAndQuery(filename string) (*NavigationQuery, error) {
	// 加载导航数据
	navData, err := builder.Load(filename)
	if err != nil {
		return nil, fmt.Errorf("failed to load navigation data: %v", err)
	}

	// 创建查询器
	query, err := NewNavigationQuery(navData)
	if err != nil {
		return nil, fmt.Errorf("failed to create navigation query: %v", err)
	}

	return query, nil
}
