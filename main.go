package main

import (
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"strconv"
	"time"

	"github.com/gorilla/mux"
	"github.com/o0olele/octree-go/octree"
	"github.com/rs/cors"
)

// 假设octree包在同一个项目中
// import "./octree"

// 全局八叉树实例
var globalOctree *octree.Octree
var jpsPathfinder *octree.OptimizedJPSPathfinder
var astarPathfinder *octree.AStarPathfinder
var nodeBasedAstarPathfinder *octree.NodeBasedAStarPathfinder

// 寻路算法接口
type Pathfinder interface {
	SetAgent(agent *octree.Agent)
	GetAgent() *octree.Agent
	SetStepSize(stepSize float64)
	GetStepSize() float64
	ToGridCoord(pos octree.Vector3) (int, int, int)
	FindPath(start, end octree.Vector3) []octree.Vector3
	SmoothPath(path []octree.Vector3) []octree.Vector3
}

var currentPathfinder Pathfinder

// 初始化请求结构
type InitRequest struct {
	Bounds   octree.AABB `json:"bounds"`
	MaxDepth int         `json:"max_depth"`
	MinSize  float64     `json:"min_size"`
}

// 添加几何体请求结构
type AddGeometryRequest struct {
	Type string          `json:"type"`
	Data json.RawMessage `json:"data"`
}

// 路径查找请求结构
type PathfindRequest struct {
	Start       octree.Vector3 `json:"start"`
	End         octree.Vector3 `json:"end"`
	StepSize    float64        `json:"step_size"`
	AgentRadius float64        `json:"agent_radius,omitempty"` // Agent半径，可选（向后兼容）
	AgentHeight float64        `json:"agent_height,omitempty"` // Agent高度，可选（向后兼容）
	Agent       *octree.Agent  `json:"agent,omitempty"`        // Agent对象，可选
	Algorithm   string         `json:"algorithm,omitempty"`    // 算法选择: "jps", "astar", "astar-node"
}

// 路径查找响应结构
type PathfindResponse struct {
	Path   []octree.Vector3 `json:"path"`
	Found  bool             `json:"found"`
	Length int              `json:"length"`
	Debug  interface{}      `json:"debug,omitempty"`
}

// 初始化八叉树
func initOctreeHandler(w http.ResponseWriter, r *http.Request) {
	fmt.Println("initOctreeHandler")
	var req InitRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid JSON", http.StatusBadRequest)
		return
	}

	globalOctree = octree.NewOctree(req.Bounds, req.MaxDepth, req.MinSize)
	jpsPathfinder = octree.NewOptimizedJPSPathfinder(globalOctree, req.MinSize)
	astarPathfinder = octree.NewAStarPathfinder(globalOctree, req.MinSize)
	currentPathfinder = jpsPathfinder // 默认使用JPS

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "initialized"})
	fmt.Println("Octree initialized")
}

// 添加几何体
func addGeometryHandler(w http.ResponseWriter, r *http.Request) {
	if globalOctree == nil {
		http.Error(w, "Octree not initialized", http.StatusBadRequest)
		return
	}

	var req AddGeometryRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid JSON", http.StatusBadRequest)
		return
	}

	var geom octree.Geometry
	switch req.Type {
	case "triangle":
		var triangle octree.Triangle
		if err := json.Unmarshal(req.Data, &triangle); err != nil {
			http.Error(w, "Invalid triangle data", http.StatusBadRequest)
			return
		}
		geom = triangle

	case "box":
		var box octree.Box
		if err := json.Unmarshal(req.Data, &box); err != nil {
			http.Error(w, "Invalid box data", http.StatusBadRequest)
			return
		}
		geom = box

	case "capsule":
		var capsule octree.Capsule
		if err := json.Unmarshal(req.Data, &capsule); err != nil {
			http.Error(w, "Invalid capsule data", http.StatusBadRequest)
			return
		}
		geom = capsule

	default:
		http.Error(w, "Unknown geometry type", http.StatusBadRequest)
		return
	}

	globalOctree.AddGeometry(geom)

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "added"})
}

// 构建八叉树
func buildOctreeHandler(w http.ResponseWriter, r *http.Request) {
	if globalOctree == nil {
		http.Error(w, "Octree not initialized", http.StatusBadRequest)
		return
	}

	globalOctree.Build()

	// 构建基于节点的A*寻路器
	nodeBasedAstarPathfinder = octree.NewNodeBasedAStarPathfinder(globalOctree, globalOctree.MinSize)

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "built"})
}

// 获取八叉树结构
func getOctreeHandler(w http.ResponseWriter, r *http.Request) {
	if globalOctree == nil {
		http.Error(w, "Octree not initialized", http.StatusBadRequest)
		return
	}

	data, err := globalOctree.ToJSON()
	if err != nil {
		http.Error(w, "Failed to serialize octree", http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	w.Write(data)
}

// 路径查找
func findPathHandler(w http.ResponseWriter, r *http.Request) {
	if globalOctree == nil || currentPathfinder == nil {
		http.Error(w, "Octree not initialized", http.StatusBadRequest)
		return
	}

	var req PathfindRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid JSON", http.StatusBadRequest)
		return
	}

	// 选择算法
	if req.Algorithm != "" {
		switch req.Algorithm {
		case "jps":
			currentPathfinder = jpsPathfinder
		case "astar":
			currentPathfinder = astarPathfinder
		case "astar-node":
			if nodeBasedAstarPathfinder != nil {
				currentPathfinder = nodeBasedAstarPathfinder
			} else {
				currentPathfinder = astarPathfinder
			}
		default:
			currentPathfinder = jpsPathfinder // 默认使用JPS
		}
	}

	// 更新寻路器的步长
	if req.StepSize > 0 {
		currentPathfinder.SetStepSize(req.StepSize)
	}

	// 设置Agent（如果提供了Agent参数）
	if req.AgentRadius > 0 && req.AgentHeight > 0 {
		agent := octree.NewAgent(req.AgentRadius, req.AgentHeight)
		currentPathfinder.SetAgent(agent)
	} else {
		currentPathfinder.SetAgent(nil) // 清除Agent，使用点检测
	}

	begTime := time.Now()
	path := currentPathfinder.FindPath(req.Start, req.End)
	endTime := time.Now()

	fmt.Printf("Pathfinding time (%s): %v\n", req.Algorithm, endTime.Sub(begTime))

	// 添加调试信息
	debugInfo := map[string]interface{}{
		"stepSize":    currentPathfinder.GetStepSize(),
		"algorithm":   req.Algorithm,
		"agentRadius": 0.0,
		"agentHeight": 0.0,
		"startValid":  !globalOctree.IsOccupied(req.Start),
		"endValid":    !globalOctree.IsOccupied(req.End),
	}

	if currentPathfinder.GetAgent() != nil {
		agent := currentPathfinder.GetAgent()
		debugInfo["agentRadius"] = agent.Radius
		debugInfo["agentHeight"] = agent.Height
		debugInfo["startValidAgent"] = !globalOctree.IsAgentOccupied(agent, req.Start)
		debugInfo["endValidAgent"] = !globalOctree.IsAgentOccupied(agent, req.End)
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
	if globalOctree == nil {
		http.Error(w, "Octree not initialized", http.StatusBadRequest)
		return
	}

	// 从查询参数获取坐标
	x, _ := strconv.ParseFloat(r.URL.Query().Get("x"), 64)
	y, _ := strconv.ParseFloat(r.URL.Query().Get("y"), 64)
	z, _ := strconv.ParseFloat(r.URL.Query().Get("z"), 64)

	point := octree.Vector3{X: x, Y: y, Z: z}

	// 检查是否有Agent参数
	agentRadius, _ := strconv.ParseFloat(r.URL.Query().Get("agent_radius"), 64)
	agentHeight, _ := strconv.ParseFloat(r.URL.Query().Get("agent_height"), 64)

	var occupied bool
	if agentRadius > 0 && agentHeight > 0 {
		agent := octree.NewAgent(agentRadius, agentHeight)
		occupied = globalOctree.IsAgentOccupied(agent, point)
	} else {
		occupied = globalOctree.IsOccupied(point)
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]bool{"occupied": occupied})
}

// 调试处理函数
func debugHandler(w http.ResponseWriter, r *http.Request) {
	if globalOctree == nil || currentPathfinder == nil {
		http.Error(w, "Octree not initialized", http.StatusBadRequest)
		return
	}

	var req PathfindRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid JSON", http.StatusBadRequest)
		return
	}

	// 选择算法
	if req.Algorithm != "" {
		switch req.Algorithm {
		case "jps":
			currentPathfinder = jpsPathfinder
		case "astar":
			currentPathfinder = astarPathfinder
		case "astar-node":
			if nodeBasedAstarPathfinder != nil {
				currentPathfinder = nodeBasedAstarPathfinder
			} else {
				currentPathfinder = astarPathfinder
			}
		default:
			currentPathfinder = jpsPathfinder // 默认使用JPS
		}
	}

	// 更新寻路器的步长
	if req.StepSize > 0 {
		currentPathfinder.SetStepSize(req.StepSize)
	}

	// 设置Agent（支持新旧两种格式）
	var agent *octree.Agent
	if req.Agent != nil {
		// 使用新格式的Agent对象
		agent = req.Agent
	} else if req.AgentRadius > 0 && req.AgentHeight > 0 {
		// 向后兼容旧格式
		agent = octree.NewAgent(req.AgentRadius, req.AgentHeight)
	}

	currentPathfinder.SetAgent(agent)

	// 获取网格坐标
	startGridX, startGridY, startGridZ := currentPathfinder.ToGridCoord(req.Start)
	endGridX, endGridY, endGridZ := currentPathfinder.ToGridCoord(req.End)

	// 详细调试信息
	debugInfo := map[string]interface{}{
		"startPos":   req.Start,
		"endPos":     req.End,
		"startGrid":  []int{startGridX, startGridY, startGridZ},
		"endGrid":    []int{endGridX, endGridY, endGridZ},
		"stepSize":   currentPathfinder.GetStepSize(),
		"algorithm":  req.Algorithm,
		"startValid": !globalOctree.IsOccupied(req.Start),
		"endValid":   !globalOctree.IsOccupied(req.End),
		"gridDiff":   []int{endGridX - startGridX, endGridY - startGridY, endGridZ - startGridZ},
	}

	// 只有JPS算法才有maxJumpDist
	if jpsTyped, ok := currentPathfinder.(*octree.OptimizedJPSPathfinder); ok {
		debugInfo["maxJumpDist"] = jpsTyped.GetMaxJumpDist()
	}

	if currentPathfinder.GetAgent() != nil {
		agent := currentPathfinder.GetAgent()
		debugInfo["agentRadius"] = agent.Radius
		debugInfo["agentHeight"] = agent.Height
		debugInfo["startValidAgent"] = !globalOctree.IsAgentOccupied(agent, req.Start)
		debugInfo["endValidAgent"] = !globalOctree.IsAgentOccupied(agent, req.End)
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(debugInfo)
}

// 批量添加三角形mesh
func addMeshHandler(w http.ResponseWriter, r *http.Request) {
	if globalOctree == nil {
		http.Error(w, "Octree not initialized", http.StatusBadRequest)
		return
	}

	var triangles []octree.Triangle
	if err := json.NewDecoder(r.Body).Decode(&triangles); err != nil {
		http.Error(w, "Invalid JSON", http.StatusBadRequest)
		return
	}

	for _, triangle := range triangles {
		globalOctree.AddGeometry(triangle)
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

func main() {
	r := mux.NewRouter()

	// API 路由
	api := r.PathPrefix("/api").Subrouter()
	api.HandleFunc("/init", initOctreeHandler).Methods("POST")
	api.HandleFunc("/geometry", addGeometryHandler).Methods("POST")
	api.HandleFunc("/mesh", addMeshHandler).Methods("POST")
	api.HandleFunc("/build", buildOctreeHandler).Methods("POST")
	api.HandleFunc("/octree", getOctreeHandler).Methods("GET")
	api.HandleFunc("/pathfind", findPathHandler).Methods("POST")
	api.HandleFunc("/occupied", checkOccupiedHandler).Methods("GET")
	api.HandleFunc("/debug", debugHandler).Methods("POST")

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
