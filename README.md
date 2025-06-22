# octree-go

一个使用Go语言实现的八叉树(Octree)空间划分和寻路库，支持3D环境中的高效碰撞检测和路径规划。

## 特性

- **八叉树空间划分**: 高效的3D空间分割和管理
- **智能体寻路**: 支持胶囊体形状的智能体寻路（考虑智能体的大小）
- **JPS寻路算法**: 优化的Jump Point Search算法，提高寻路效率
- **多种几何体支持**: 三角形、立方体、胶囊体等几何形状
- **Web API**: 提供RESTful API接口，方便集成
- **实时可视化**: 包含Web界面用于可视化八叉树和路径

## 新增Agent寻路功能

### Agent概念
智能体(Agent)被表示为一个胶囊体(Capsule)，具有以下属性：
- **半径(Radius)**: 胶囊体的横截面半径
- **高度(Height)**: 胶囊体的柱状部分高度（不包括两端的半球）

### API更新

#### 路径查找请求 (POST /findpath)
```json
{
    "start": {"x": -8.0, "y": 0.0, "z": 0.0},
    "end": {"x": 8.0, "y": 0.0, "z": 0.0},
    "step_size": 0.5,
    "agent_radius": 0.5,    // 可选：智能体半径
    "agent_height": 1.8     // 可选：智能体高度
}
```

#### 碰撞检测 (GET /checkoccupied)
```
GET /checkoccupied?x=0&y=0&z=0&agent_radius=0.5&agent_height=1.8
```

### 使用示例

#### 基本寻路（点寻路）
```go
// 创建八叉树
bounds := octree.AABB{
    Min: octree.Vector3{X: -10, Y: -10, Z: -10},
    Max: octree.Vector3{X: 10, Y: 10, Z: 10},
}
tree := octree.NewOctree(bounds, 6, 0.5)

// 添加障碍物
box := octree.Box{
    Center: octree.Vector3{X: 0, Y: 0, Z: 0},
    Size: octree.Vector3{X: 2, Y: 4, Z: 2},
}
tree.AddGeometry(box)
tree.Build()

// 创建寻路器
pathfinder := octree.NewOptimizedJPSPathfinder(tree, 0.5)

// 点寻路（不考虑智能体大小）
start := octree.Vector3{X: -8, Y: 0, Z: 0}
end := octree.Vector3{X: 8, Y: 0, Z: 0}
path := pathfinder.FindPath(start, end)
```

#### Agent寻路（考虑智能体大小）
```go
// 创建智能体
agent := octree.NewAgent(0.5, 1.8) // 半径0.5，高度1.8
pathfinder.SetAgent(agent)

// 智能体寻路
path := pathfinder.FindPath(start, end)

// 检查智能体在指定位置是否发生碰撞
position := octree.Vector3{X: 1, Y: 0, Z: 0}
occupied := tree.IsAgentOccupied(agent, position)
```

### 碰撞检测原理

#### 胶囊体vs几何体碰撞检测
- **胶囊体vs三角形**: 计算胶囊体轴线到三角形边的最短距离
- **胶囊体vs立方体**: 计算胶囊体轴线上最近点到立方体表面的距离
- **胶囊体vs胶囊体**: 计算两个胶囊体轴线之间的最短距离

#### 优化策略
- 使用AABB包围盒进行粗略碰撞检测
- 只对通过粗略检测的几何体进行精确碰撞检测
- 利用八叉树的空间分割减少需要检测的几何体数量

## 运行测试

```bash

# 编译并运行Web服务器
go build -o octree-server .
./octree-server
```

## Web界面
访问 `http://localhost:8080` 查看八叉树可视化界面。

## API文档

### 初始化八叉树
- **POST** `/init`
- **Body**: `{"bounds": {...}, "max_depth": 6, "min_size": 0.5}`

### 添加几何体
- **POST** `/addgeometry`
- **Body**: `{"type": "box|triangle|capsule", "data": {...}}`

### 构建八叉树
- **POST** `/build`

### 寻路
- **POST** `/findpath`
- **Body**: `{"start": {...}, "end": {...}, "step_size": 0.5, "agent_radius": 0.5, "agent_height": 1.8}`

### 检查占用状态
- **GET** `/checkoccupied?x=0&y=0&z=0&agent_radius=0.5&agent_height=1.8`

## 性能特点

1. **空间效率**: 八叉树动态分割，只在需要时创建子节点
2. **查询效率**: O(log n)的空间查询复杂度
3. **寻路优化**: JPS算法减少搜索节点数量
4. **碰撞优化**: 分层碰撞检测，先粗略后精确

## 应用场景

- 游戏AI寻路
- 机器人路径规划
- 3D场景导航
- 虚拟现实环境
- 自动驾驶仿真

# Screenshots
![](./img/example.png)
