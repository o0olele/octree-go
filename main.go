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
var pathfinder *octree.OptimizedJPSPathfinder

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
	AgentRadius float64        `json:"agent_radius,omitempty"` // Agent半径，可选
	AgentHeight float64        `json:"agent_height,omitempty"` // Agent高度，可选
}

// 路径查找响应结构
type PathfindResponse struct {
	Path   []octree.Vector3 `json:"path"`
	Found  bool             `json:"found"`
	Length int              `json:"length"`
}

// 初始化八叉树
func initOctreeHandler(w http.ResponseWriter, r *http.Request) {
	var req InitRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid JSON", http.StatusBadRequest)
		return
	}

	globalOctree = octree.NewOctree(req.Bounds, req.MaxDepth, req.MinSize)
	pathfinder = octree.NewOptimizedJPSPathfinder(globalOctree, req.MinSize)

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "initialized"})
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
	if globalOctree == nil || pathfinder == nil {
		http.Error(w, "Octree not initialized", http.StatusBadRequest)
		return
	}

	var req PathfindRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid JSON", http.StatusBadRequest)
		return
	}

	// 更新寻路器的步长
	if req.StepSize > 0 {
		pathfinder.SetStepSize(req.StepSize)
	}

	// 设置Agent（如果提供了Agent参数）
	if req.AgentRadius > 0 && req.AgentHeight > 0 {
		agent := octree.NewAgent(req.AgentRadius, req.AgentHeight)
		pathfinder.SetAgent(agent)
	} else {
		pathfinder.SetAgent(nil) // 清除Agent，使用点检测
	}

	begTime := time.Now()
	path := pathfinder.FindPath(req.Start, req.End)
	endTime := time.Now()

	fmt.Println("Pathfinding time:", endTime.Sub(begTime))

	response := PathfindResponse{
		Path:   path,
		Found:  path != nil,
		Length: len(path),
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
