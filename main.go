package main

import (
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"strconv"
	"time"

	_ "net/http/pprof"

	"github.com/gorilla/mux"
	"github.com/o0olele/octree-go/builder"
	"github.com/o0olele/octree-go/geometry"
	"github.com/o0olele/octree-go/math32"
	"github.com/o0olele/octree-go/octree"
	"github.com/o0olele/octree-go/query"
	"github.com/rs/cors"
)

// 假设octree包在同一个项目中
// import "./octree"

// 全局实例
var navigationBuilder *builder.Builder
var navigationQuery *query.NavigationQuery
var globalAgent *octree.Agent

// 初始化请求结构
type InitRequest struct {
	Bounds   geometry.AABB `json:"bounds"`
	MaxDepth uint8         `json:"max_depth"`
	MinSize  float32       `json:"min_size"`
}

// 添加几何体请求结构
type AddGeometryRequest struct {
	Type string          `json:"type"`
	Data json.RawMessage `json:"data"`
}

// 路径查找请求结构
type PathfindRequest struct {
	Start       math32.Vector3 `json:"start"`
	End         math32.Vector3 `json:"end"`
	StepSize    float32        `json:"step_size"`
	AgentRadius float32        `json:"agent_radius,omitempty"` // Agent半径，可选（向后兼容）
	AgentHeight float32        `json:"agent_height,omitempty"` // Agent高度，可选（向后兼容）
	Agent       *octree.Agent  `json:"agent,omitempty"`        // Agent对象，可选
}

// 路径查找响应结构
type PathfindResponse struct {
	Path   []math32.Vector3 `json:"path"`
	Found  bool             `json:"found"`
	Length int              `json:"length"`
	Debug  interface{}      `json:"debug,omitempty"`
}

// 初始化导航构建器
func initOctreeHandler(w http.ResponseWriter, r *http.Request) {
	fmt.Println("initOctreeHandler")
	var req InitRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid JSON", http.StatusBadRequest)
		return
	}

	// 创建导航构建器
	navigationBuilder = builder.NewBuilder(req.Bounds, req.MaxDepth, req.MinSize, req.MinSize)

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "initialized"})
	fmt.Println("Navigation builder initialized")
}

// 构建导航网格
func buildOctreeHandler(w http.ResponseWriter, r *http.Request) {
	if navigationBuilder == nil {
		http.Error(w, "Navigation builder not initialized", http.StatusBadRequest)
		return
	}

	navigationBuilder.SetUseVoxel(true)
	// 构建导航数据
	navData, err := navigationBuilder.Build(globalAgent)
	if err != nil {
		http.Error(w, fmt.Sprintf("Failed to build navigation data: %v", err), http.StatusInternalServerError)
		return
	}

	// 创建导航查询器
	navigationQuery, err = query.NewNavigationQuery(navData)
	if err != nil {
		http.Error(w, fmt.Sprintf("Failed to create navigation query: %v", err), http.StatusInternalServerError)
		return
	}

	err = builder.Save(navData, "test.nav")
	if err != nil {
		fmt.Println("Failed to save navigation data: ", err)
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "built"})
}

// 获取八叉树结构
func getOctreeHandler(w http.ResponseWriter, r *http.Request) {
	if navigationBuilder == nil {
		http.Error(w, "Octree not initialized", http.StatusBadRequest)
		return
	}

	data, err := navigationBuilder.GetOctree().ToJSON()
	if err != nil {
		http.Error(w, "Failed to serialize octree", http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	w.Write(data)
}

// 路径查找
func findPathHandler(w http.ResponseWriter, r *http.Request) {
	var req PathfindRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid JSON", http.StatusBadRequest)
		return
	}

	var path []math32.Vector3
	var debugInfo map[string]interface{}

	// 优先使用新的NavigationQuery
	if navigationQuery != nil {
		// 更新寻路器的步长
		if req.StepSize > 0 {
			navigationQuery.SetStepSize(req.StepSize)
		}

		// 设置Agent（如果提供了Agent参数）
		if req.AgentRadius > 0 && req.AgentHeight > 0 {
			agent := octree.NewAgent(req.AgentRadius, req.AgentHeight)
			navigationQuery.SetAgent(agent)
		} else {
			navigationQuery.SetAgent(nil) // 清除Agent，使用点检测
		}

		begTime := time.Now()
		path = navigationQuery.FindPath(req.Start, req.End)
		endTime := time.Now()

		fmt.Printf("New pathfinding time: %v\n", endTime.Sub(begTime))

		// 添加调试信息
		stats := navigationQuery.GetStats()
		debugInfo = map[string]interface{}{
			"stepSize":    navigationQuery.GetStepSize(),
			"agentRadius": 0.0,
			"agentHeight": 0.0,
			"nodeCount":   stats.NodeCount,
			"edgeCount":   stats.EdgeCount,
			"cacheSize":   stats.CacheSize,
			"dataSize":    stats.DataSize,
			"method":      "NavigationQuery",
		}

		if navigationQuery.GetAgent() != nil {
			agent := navigationQuery.GetAgent()
			debugInfo["agentRadius"] = agent.Radius
			debugInfo["agentHeight"] = agent.Height
		}

	} else {
		http.Error(w, "No pathfinder available", http.StatusBadRequest)
		return
	}

	response := PathfindResponse{
		Path:   path,
		Found:  path != nil,
		Length: len(path),
		Debug:  debugInfo,
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(response)
}

// 检查点是否被占用
func checkOccupiedHandler(w http.ResponseWriter, r *http.Request) {
	if navigationQuery == nil {
		http.Error(w, "Octree not initialized", http.StatusBadRequest)
		return
	}

	// 从查询参数获取坐标
	x, _ := strconv.ParseFloat(r.URL.Query().Get("x"), 64)
	y, _ := strconv.ParseFloat(r.URL.Query().Get("y"), 64)
	z, _ := strconv.ParseFloat(r.URL.Query().Get("z"), 64)

	point := math32.Vector3{X: float32(x), Y: float32(y), Z: float32(z)}

	// 检查是否有Agent参数
	agentRadius, _ := strconv.ParseFloat(r.URL.Query().Get("agent_radius"), 64)
	agentHeight, _ := strconv.ParseFloat(r.URL.Query().Get("agent_height"), 64)

	var occupied bool
	if agentRadius > 0 && agentHeight > 0 {
		agent := octree.NewAgent(float32(agentRadius), float32(agentHeight))
		occupied = navigationBuilder.GetOctree().IsAgentOccupied(agent, point)
	} else {
		occupied = navigationBuilder.GetOctree().IsOccupied(point)
	}

	var gridOccupied bool
	if navigationQuery != nil {
		gridOccupied = navigationQuery.GetNavigationData().IsWorldOccupiedByVoxel(point)
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]bool{"occupied": occupied, "grid_occupied": gridOccupied})
}

// 获取路径图数据
func getPathGraphHandler(w http.ResponseWriter, r *http.Request) {
	if navigationBuilder == nil {
		http.Error(w, "Node-based pathfinder not initialized", http.StatusBadRequest)
		return
	}

	pathGraphData := navigationBuilder.GetPathFinder().ToPathGraphData()

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(pathGraphData)
}

// 批量添加三角形mesh
func addMeshHandler(w http.ResponseWriter, r *http.Request) {
	if navigationBuilder == nil {
		http.Error(w, "Octree not initialized", http.StatusBadRequest)
		return
	}

	var triangles []geometry.Triangle
	if err := json.NewDecoder(r.Body).Decode(&triangles); err != nil {
		http.Error(w, "Invalid JSON", http.StatusBadRequest)
		return
	}

	for _, triangle := range triangles {
		navigationBuilder.AddTriangle(triangle)
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status": "added",
		"count":  len(triangles),
	})
}

// 静态文件服务
func staticFileHandler(w http.ResponseWriter, r *http.Request) {
	http.ServeFile(w, r, "./web/"+r.URL.Path[1:])
}

// 保存和加载请求结构
type SaveRequest struct {
	NavigationFilename string `json:"navigation_filename"`
	OptimizeData       bool   `json:"optimize_data,omitempty"`
}

type LoadRequest struct {
	NavigationFilename string `json:"navigation_filename"`
}

// 保存导航数据
func saveHandler(w http.ResponseWriter, r *http.Request) {
	if navigationBuilder == nil {
		http.Error(w, "Navigation builder not initialized", http.StatusBadRequest)
		return
	}

	var req SaveRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid JSON", http.StatusBadRequest)
		return
	}

	// 构建导航数据
	navData, err := navigationBuilder.Build(globalAgent)
	if err != nil {
		http.Error(w, fmt.Sprintf("Failed to build navigation data: %v", err), http.StatusInternalServerError)
		return
	}

	// 如果需要优化数据
	// if req.OptimizeData {
	// 	optimizedData := octree.OptimizeNavigationData(navData)

	// 	// 分析优化效果
	// 	analysis := octree.AnalyzeOptimization(navData, optimizedData)
	// 	fmt.Printf("Optimization analysis: %+v\n", analysis)

	// 	// 转换回标准格式进行保存
	// 	navData = octree.DeoptimizeNavigationData(optimizedData)
	// }

	// 保存到文件
	if err := builder.Save(navData, req.NavigationFilename); err != nil {
		http.Error(w, fmt.Sprintf("Failed to save navigation data: %v", err), http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "saved"})
}

// 加载导航数据
func loadHandler(w http.ResponseWriter, r *http.Request) {
	var req LoadRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid JSON", http.StatusBadRequest)
		return
	}

	begTime := time.Now()
	// 加载导航数据并创建查询器
	query, err := query.LoadAndQuery(req.NavigationFilename)
	if err != nil {
		http.Error(w, fmt.Sprintf("Failed to load navigation data: %v", err), http.StatusInternalServerError)
		return
	}
	endTime := time.Now()
	fmt.Printf("Load navigation data time: %v\n", endTime.Sub(begTime))

	navigationQuery = query

	// 获取统计信息
	stats := query.GetStats()
	fmt.Printf("Loaded navigation data: %+v\n", stats)

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status": "loaded",
		"stats":  stats,
	})
}

func addAgentHandler(w http.ResponseWriter, r *http.Request) {
	var req octree.Agent
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid JSON", http.StatusBadRequest)
		return
	}

	globalAgent = octree.NewAgent(req.Radius, req.Height)

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "added", "agent": fmt.Sprintf("%+v", globalAgent)})
}

func main() {
	r := mux.NewRouter()

	go func() {
		log.Println(http.ListenAndServe("localhost:6060", nil))
	}()

	// API 路由
	api := r.PathPrefix("/api").Subrouter()
	api.HandleFunc("/init", initOctreeHandler).Methods("POST")
	api.HandleFunc("/agent", addAgentHandler).Methods("POST")
	api.HandleFunc("/mesh", addMeshHandler).Methods("POST")
	api.HandleFunc("/build", buildOctreeHandler).Methods("POST")
	api.HandleFunc("/octree", getOctreeHandler).Methods("GET")
	api.HandleFunc("/pathfind", findPathHandler).Methods("POST")
	api.HandleFunc("/occupied", checkOccupiedHandler).Methods("GET")
	api.HandleFunc("/pathgraph", getPathGraphHandler).Methods("GET")
	api.HandleFunc("/save", saveHandler).Methods("POST")
	api.HandleFunc("/load", loadHandler).Methods("POST")

	// 静态文件路由
	r.PathPrefix("/").Handler(http.HandlerFunc(staticFileHandler))

	// 配置CORS
	c := cors.New(cors.Options{
		AllowedOrigins: []string{"*"},
		AllowedMethods: []string{"GET", "POST", "PUT", "DELETE", "OPTIONS"},
		AllowedHeaders: []string{"*"},
	})

	handler := c.Handler(r)

	fmt.Println("Server starting on http://localhost:8080")
	log.Fatal(http.ListenAndServe(":8080", handler))
}
