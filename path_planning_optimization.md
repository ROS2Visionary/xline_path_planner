# 路径规划优化方案

## 一、需求分析

### 1.1 需求概述

| 需求编号 | 需求描述 | 处理阶段 |
|---------|---------|---------|
| REQ-01 | 多段线（Polyline）拆分为独立线段（Line）后参与路径规划 | CAD解析/预处理阶段 |
| REQ-02 | 共线且相连的直线线段合并为一条路径 | 路径规划阶段 |

### 1.2 现有代码结构分析

根据 `output_formatter.cpp` 的实现，当前系统处理以下几何类型：

```
GeometryType:
├── LINE        → 单条直线
├── POLYLINE    → 多段线（顶点序列）
├── CIRCLE      → 圆
├── ARC         → 圆弧
└── TEXT        → 文字
```

当前的 `RouteSegment` 结构包含：
- `type`: RouteType（TRANSITION_PATH / DRAWING_PATH）
- `points`: 路径点序列
- `line_id`: 关联的原始几何ID
- `printer_type`: 打印机类型
- `ink_mode`: 喷墨模式
- `execute_backward`: 是否后退执行（仅对转场路径有意义）

---

## 二、优化方案设计

### 2.1 整体架构调整

```
┌─────────────────────────────────────────────────────────────────────┐
│                         优化后的处理流程                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────┐    ┌──────────────┐    ┌──────────────┐    ┌────────┐│
│  │ CAD解析  │ -> │ 几何预处理   │ -> │  路径规划    │ -> │ 输出   ││
│  │          │    │              │    │              │    │        ││
│  │ • 读取   │    │ • Polyline   │    │ • 共线合并   │    │ • JSON ││
│  │ • 转换   │    │   拆分       │    │ • 路径排序   │    │ • 可视化││
│  └──────────┘    └──────────────┘    └──────────────┘    └────────┘│
│                         ↑                    ↑                      │
│                    [REQ-01]             [REQ-02]                    │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 三、REQ-01：Polyline 拆分实现

### 3.1 设计思路

在 CAD 数据解析完成后、路径规划之前，增加几何预处理阶段，将 Polyline 拆分为独立的 Line 对象。

### 3.2 数据结构扩展

```cpp
// 在 cad_types.hpp 中添加
namespace path_planner {

// 拆分后的线段需要保留与原始Polyline的关联关系
struct LineSegment : public Line {
    int32_t parent_polyline_id;  // 原始Polyline的ID（-1表示非Polyline来源）
    int32_t segment_index;       // 在原始Polyline中的索引
    bool is_from_polyline;       // 是否来自Polyline拆分
    
    LineSegment() 
        : parent_polyline_id(-1)
        , segment_index(-1)
        , is_from_polyline(false) {}
};

} // namespace path_planner
```

### 3.3 核心实现：Polyline 拆分器

```cpp
// geometry_preprocessor.hpp
#pragma once

#include "cad_types.hpp"
#include <vector>
#include <memory>

namespace path_planner {

class GeometryPreprocessor {
public:
    /**
     * @brief 预处理CAD数据，执行Polyline拆分
     * @param cad_data 原始CAD数据
     * @return 处理后的CAD数据（Polyline已拆分为Line）
     */
    static CADData preprocess(const CADData& cad_data);

private:
    /**
     * @brief 将单个Polyline拆分为多个Line
     * @param polyline 待拆分的Polyline
     * @param start_id 起始ID（用于生成新Line的ID）
     * @return 拆分后的Line列表
     */
    static std::vector<std::unique_ptr<Line>> splitPolyline(
        const Polyline& polyline, 
        int32_t& start_id);
    
    /**
     * @brief 生成拆分后Line的唯一ID
     * @param parent_id 原始Polyline的ID
     * @param segment_idx 线段索引
     * @return 新的唯一ID
     */
    static int32_t generateSegmentId(int32_t parent_id, int32_t segment_idx);
};

} // namespace path_planner
```

```cpp
// geometry_preprocessor.cpp
#include "geometry_preprocessor.hpp"
#include <algorithm>

namespace path_planner {

CADData GeometryPreprocessor::preprocess(const CADData& cad_data) {
    CADData result;
    
    // 复制非path_lines字段
    result.bounds = cad_data.bounds;
    // ... 复制其他必要字段
    
    int32_t next_id = 10000000;  // 拆分线段的ID起始值，避免与原始ID冲突
    
    for (const auto& line_ptr : cad_data.path_lines) {
        if (!line_ptr) continue;
        
        if (line_ptr->type == GeometryType::POLYLINE) {
            // 拆分Polyline
            const Polyline* polyline = dynamic_cast<const Polyline*>(line_ptr.get());
            if (polyline && polyline->vertices.size() >= 2) {
                auto segments = splitPolyline(*polyline, next_id);
                for (auto& seg : segments) {
                    result.path_lines.push_back(std::move(seg));
                }
            }
        } else {
            // 非Polyline直接复制
            result.path_lines.push_back(
                std::unique_ptr<Line>(line_ptr->clone())
            );
        }
    }
    
    return result;
}

std::vector<std::unique_ptr<Line>> GeometryPreprocessor::splitPolyline(
    const Polyline& polyline, 
    int32_t& start_id) 
{
    std::vector<std::unique_ptr<Line>> segments;
    
    const auto& vertices = polyline.vertices;
    if (vertices.size() < 2) {
        return segments;
    }
    
    for (size_t i = 0; i + 1 < vertices.size(); ++i) {
        auto segment = std::make_unique<LineSegment>();
        
        // 基本属性继承自原始Polyline
        segment->id = generateSegmentId(polyline.id, static_cast<int32_t>(i));
        segment->type = GeometryType::LINE;
        segment->layer = polyline.layer;
        segment->layer_id = polyline.layer_id;
        segment->color = polyline.color;
        segment->line_type = polyline.line_type;
        segment->thickness = polyline.thickness;
        segment->hidden = polyline.hidden;
        segment->selected = polyline.selected;
        
        // 设置起点和终点
        segment->start = vertices[i];
        segment->end = vertices[i + 1];
        
        // 记录Polyline来源信息
        segment->parent_polyline_id = polyline.id;
        segment->segment_index = static_cast<int32_t>(i);
        segment->is_from_polyline = true;
        
        segments.push_back(std::move(segment));
        start_id++;
    }
    
    // 处理闭合Polyline
    if (polyline.closed && vertices.size() > 2) {
        auto closing_segment = std::make_unique<LineSegment>();
        
        closing_segment->id = generateSegmentId(polyline.id, static_cast<int32_t>(vertices.size() - 1));
        closing_segment->type = GeometryType::LINE;
        closing_segment->layer = polyline.layer;
        closing_segment->layer_id = polyline.layer_id;
        closing_segment->color = polyline.color;
        closing_segment->line_type = polyline.line_type;
        closing_segment->thickness = polyline.thickness;
        closing_segment->hidden = polyline.hidden;
        closing_segment->selected = polyline.selected;
        
        // 闭合线段：从最后一个顶点连回第一个顶点
        closing_segment->start = vertices.back();
        closing_segment->end = vertices.front();
        
        closing_segment->parent_polyline_id = polyline.id;
        closing_segment->segment_index = static_cast<int32_t>(vertices.size() - 1);
        closing_segment->is_from_polyline = true;
        
        segments.push_back(std::move(closing_segment));
    }
    
    return segments;
}

int32_t GeometryPreprocessor::generateSegmentId(int32_t parent_id, int32_t segment_idx) {
    // ID格式: parent_id * 10000 + segment_idx
    // 确保拆分后的线段ID唯一且可追溯
    return parent_id * 10000 + segment_idx;
}

} // namespace path_planner
```

### 3.4 集成到主流程

在 `path_planner.cpp` 或 `main.cpp` 中的调用位置：

```cpp
// 在路径规划之前调用预处理
CADData raw_cad_data = cad_parser.parse(cad_file_path);

// [新增] 几何预处理：拆分Polyline
CADData preprocessed_data = GeometryPreprocessor::preprocess(raw_cad_data);

// 使用预处理后的数据进行路径规划
auto planned_paths = path_planner.plan(preprocessed_data);
```

---

## 四、REQ-02：共线直线合并实现

### 4.1 设计思路

在路径规划阶段，检测所有直线线段，将满足以下条件的线段合并：
1. **共线性**：两条线段位于同一直线上
2. **相连性**：一条线段的端点与另一条线段的端点重合或足够接近

### 4.2 数学基础

#### 4.2.1 共线性判断

两条线段 AB 和 CD 共线的条件：
- 向量 AB、AC、AD 共面且共线
- 使用叉积判断：`|AB × AC| < ε` 且 `|AB × AD| < ε`

```
共线判断公式：
设 AB = (x1, y1), AC = (x2, y2)
叉积 AB × AC = x1*y2 - y1*x2
若 |AB × AC| / |AB| < tolerance，则 C 在直线 AB 上
```

#### 4.2.2 相连性判断

端点距离小于阈值即认为相连：
- `distance(A.end, B.start) < tolerance` 或
- `distance(A.end, B.end) < tolerance` 或  
- `distance(A.start, B.start) < tolerance` 或
- `distance(A.start, B.end) < tolerance`

### 4.3 数据结构

```cpp
// collinear_merger.hpp
#pragma once

#include "cad_types.hpp"
#include <vector>
#include <set>

namespace path_planner {

// 线段连接关系
enum class ConnectionType {
    END_TO_START,    // A.end -> B.start
    END_TO_END,      // A.end -> B.end (需反转B)
    START_TO_START,  // A.start -> B.start (需反转A)
    START_TO_END,    // A.start -> B.end (需反转A，或将B接到A前面)
    NONE             // 无连接
};

// 合并组信息
struct MergeGroup {
    std::vector<int32_t> line_ids;           // 组内线段ID（按连接顺序）
    std::vector<bool> need_reverse;          // 对应线段是否需要反转
    Point3D merged_start;                    // 合并后起点
    Point3D merged_end;                      // 合并后终点
};

class CollinearMerger {
public:
    explicit CollinearMerger(double distance_tolerance = 0.001,  // 1mm
                             double angle_tolerance = 0.001);     // 弧度
    
    /**
     * @brief 合并共线且相连的直线线段
     * @param lines 输入线段列表
     * @return 合并后的路径列表（每个MergeGroup代表一条合并路径）
     */
    std::vector<MergeGroup> merge(const std::vector<const Line*>& lines);

private:
    double distance_tolerance_;
    double angle_tolerance_;
    
    // 判断两条线段是否共线
    bool areCollinear(const Line& a, const Line& b) const;
    
    // 判断两条线段的连接关系
    ConnectionType getConnectionType(const Line& a, const Line& b) const;
    
    // 计算点到直线的距离
    double pointToLineDistance(const Point3D& point, 
                               const Point3D& line_start, 
                               const Point3D& line_end) const;
    
    // 计算两点距离
    double pointDistance(const Point3D& a, const Point3D& b) const;
    
    // 计算线段方向向量（归一化）
    Point3D getDirection(const Line& line) const;
    
    // 使用并查集进行分组
    int32_t findRoot(std::vector<int32_t>& parent, int32_t i);
    void unionSets(std::vector<int32_t>& parent, std::vector<int32_t>& rank, 
                   int32_t a, int32_t b);
};

} // namespace path_planner
```

### 4.4 核心实现

```cpp
// collinear_merger.cpp
#include "collinear_merger.hpp"
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <queue>

namespace path_planner {

CollinearMerger::CollinearMerger(double distance_tolerance, double angle_tolerance)
    : distance_tolerance_(distance_tolerance)
    , angle_tolerance_(angle_tolerance) {}

double CollinearMerger::pointDistance(const Point3D& a, const Point3D& b) const {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

Point3D CollinearMerger::getDirection(const Line& line) const {
    double dx = line.end.x - line.start.x;
    double dy = line.end.y - line.start.y;
    double dz = line.end.z - line.start.z;
    double len = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    if (len < 1e-10) {
        return {0, 0, 0};
    }
    return {dx/len, dy/len, dz/len};
}

double CollinearMerger::pointToLineDistance(const Point3D& point,
                                            const Point3D& line_start,
                                            const Point3D& line_end) const {
    // 向量 AB 和 AP
    double abx = line_end.x - line_start.x;
    double aby = line_end.y - line_start.y;
    double abz = line_end.z - line_start.z;
    
    double apx = point.x - line_start.x;
    double apy = point.y - line_start.y;
    double apz = point.z - line_start.z;
    
    // 叉积 AB × AP
    double cx = aby * apz - abz * apy;
    double cy = abz * apx - abx * apz;
    double cz = abx * apy - aby * apx;
    
    double cross_len = std::sqrt(cx*cx + cy*cy + cz*cz);
    double ab_len = std::sqrt(abx*abx + aby*aby + abz*abz);
    
    if (ab_len < 1e-10) {
        return pointDistance(point, line_start);
    }
    
    return cross_len / ab_len;
}

bool CollinearMerger::areCollinear(const Line& a, const Line& b) const {
    // 检查 B 的两个端点到直线 A 的距离
    double dist_b_start = pointToLineDistance(b.start, a.start, a.end);
    double dist_b_end = pointToLineDistance(b.end, a.start, a.end);
    
    if (dist_b_start > distance_tolerance_ || dist_b_end > distance_tolerance_) {
        return false;
    }
    
    // 检查方向向量是否平行（允许反向）
    Point3D dir_a = getDirection(a);
    Point3D dir_b = getDirection(b);
    
    // 点积的绝对值应接近1（平行或反平行）
    double dot = std::abs(dir_a.x * dir_b.x + dir_a.y * dir_b.y + dir_a.z * dir_b.z);
    
    return dot > (1.0 - angle_tolerance_);
}

ConnectionType CollinearMerger::getConnectionType(const Line& a, const Line& b) const {
    double d_end_start = pointDistance(a.end, b.start);
    double d_end_end = pointDistance(a.end, b.end);
    double d_start_start = pointDistance(a.start, b.start);
    double d_start_end = pointDistance(a.start, b.end);
    
    // 找最小距离
    double min_dist = std::min({d_end_start, d_end_end, d_start_start, d_start_end});
    
    if (min_dist > distance_tolerance_) {
        return ConnectionType::NONE;
    }
    
    if (d_end_start <= distance_tolerance_) {
        return ConnectionType::END_TO_START;
    }
    if (d_end_end <= distance_tolerance_) {
        return ConnectionType::END_TO_END;
    }
    if (d_start_start <= distance_tolerance_) {
        return ConnectionType::START_TO_START;
    }
    if (d_start_end <= distance_tolerance_) {
        return ConnectionType::START_TO_END;
    }
    
    return ConnectionType::NONE;
}

int32_t CollinearMerger::findRoot(std::vector<int32_t>& parent, int32_t i) {
    if (parent[i] != i) {
        parent[i] = findRoot(parent, parent[i]);
    }
    return parent[i];
}

void CollinearMerger::unionSets(std::vector<int32_t>& parent, 
                                std::vector<int32_t>& rank,
                                int32_t a, int32_t b) {
    int32_t root_a = findRoot(parent, a);
    int32_t root_b = findRoot(parent, b);
    
    if (root_a != root_b) {
        if (rank[root_a] < rank[root_b]) {
            parent[root_a] = root_b;
        } else if (rank[root_a] > rank[root_b]) {
            parent[root_b] = root_a;
        } else {
            parent[root_b] = root_a;
            rank[root_a]++;
        }
    }
}

std::vector<MergeGroup> CollinearMerger::merge(const std::vector<const Line*>& lines) {
    const size_t n = lines.size();
    if (n == 0) return {};
    
    // 并查集初始化
    std::vector<int32_t> parent(n);
    std::vector<int32_t> rank(n, 0);
    for (size_t i = 0; i < n; ++i) {
        parent[i] = static_cast<int32_t>(i);
    }
    
    // 存储连接关系
    std::vector<std::vector<std::pair<int32_t, ConnectionType>>> adj(n);
    
    // 检查所有线段对
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            if (!lines[i] || !lines[j]) continue;
            
            // 只处理LINE类型
            if (lines[i]->type != GeometryType::LINE || 
                lines[j]->type != GeometryType::LINE) {
                continue;
            }
            
            // 检查是否共线且相连
            if (areCollinear(*lines[i], *lines[j])) {
                ConnectionType conn = getConnectionType(*lines[i], *lines[j]);
                if (conn != ConnectionType::NONE) {
                    unionSets(parent, rank, static_cast<int32_t>(i), static_cast<int32_t>(j));
                    adj[i].emplace_back(static_cast<int32_t>(j), conn);
                    // 添加反向连接
                    ConnectionType reverse_conn;
                    switch (conn) {
                        case ConnectionType::END_TO_START:
                            reverse_conn = ConnectionType::START_TO_END;
                            break;
                        case ConnectionType::END_TO_END:
                            reverse_conn = ConnectionType::START_TO_START;
                            break;
                        case ConnectionType::START_TO_START:
                            reverse_conn = ConnectionType::END_TO_END;
                            break;
                        case ConnectionType::START_TO_END:
                            reverse_conn = ConnectionType::END_TO_START;
                            break;
                        default:
                            reverse_conn = ConnectionType::NONE;
                    }
                    adj[j].emplace_back(static_cast<int32_t>(i), reverse_conn);
                }
            }
        }
    }
    
    // 按组收集线段
    std::unordered_map<int32_t, std::vector<int32_t>> groups;
    for (size_t i = 0; i < n; ++i) {
        int32_t root = findRoot(parent, static_cast<int32_t>(i));
        groups[root].push_back(static_cast<int32_t>(i));
    }
    
    // 为每个组构建有序的合并路径
    std::vector<MergeGroup> result;
    
    for (auto& [root, indices] : groups) {
        MergeGroup group;
        
        if (indices.size() == 1) {
            // 单个线段，无需合并
            int32_t idx = indices[0];
            group.line_ids.push_back(lines[idx]->id);
            group.need_reverse.push_back(false);
            group.merged_start = lines[idx]->start;
            group.merged_end = lines[idx]->end;
        } else {
            // 多个线段，需要确定顺序
            // 使用BFS从某个端点线段开始遍历
            std::vector<bool> visited(n, false);
            
            // 找到一个端点线段（只有一个连接的线段）
            int32_t start_idx = indices[0];
            for (int32_t idx : indices) {
                int conn_count = 0;
                for (const auto& [neighbor, _] : adj[idx]) {
                    if (findRoot(parent, neighbor) == root) {
                        conn_count++;
                    }
                }
                if (conn_count == 1) {
                    start_idx = idx;
                    break;
                }
            }
            
            // BFS遍历构建顺序
            std::queue<std::pair<int32_t, bool>> q;  // (index, need_reverse)
            q.push({start_idx, false});
            visited[start_idx] = true;
            
            while (!q.empty()) {
                auto [curr_idx, curr_reverse] = q.front();
                q.pop();
                
                group.line_ids.push_back(lines[curr_idx]->id);
                group.need_reverse.push_back(curr_reverse);
                
                for (const auto& [next_idx, conn_type] : adj[curr_idx]) {
                    if (visited[next_idx]) continue;
                    if (findRoot(parent, next_idx) != root) continue;
                    
                    visited[next_idx] = true;
                    
                    // 根据连接类型决定下一个线段是否需要反转
                    bool next_reverse = false;
                    if (curr_reverse) {
                        // 当前线段已反转，连接关系也要相应调整
                        switch (conn_type) {
                            case ConnectionType::END_TO_START:
                                next_reverse = true;  // 原来end相连，反转后start相连
                                break;
                            case ConnectionType::END_TO_END:
                                next_reverse = false;
                                break;
                            case ConnectionType::START_TO_START:
                                next_reverse = false;
                                break;
                            case ConnectionType::START_TO_END:
                                next_reverse = true;
                                break;
                            default:
                                break;
                        }
                    } else {
                        switch (conn_type) {
                            case ConnectionType::END_TO_START:
                                next_reverse = false;
                                break;
                            case ConnectionType::END_TO_END:
                                next_reverse = true;
                                break;
                            case ConnectionType::START_TO_START:
                                next_reverse = true;
                                break;
                            case ConnectionType::START_TO_END:
                                next_reverse = false;
                                break;
                            default:
                                break;
                        }
                    }
                    
                    q.push({next_idx, next_reverse});
                }
            }
            
            // 计算合并后的起点和终点
            if (!group.line_ids.empty()) {
                // 找第一个线段
                int32_t first_idx = -1;
                for (size_t i = 0; i < indices.size(); ++i) {
                    if (lines[indices[i]]->id == group.line_ids[0]) {
                        first_idx = indices[i];
                        break;
                    }
                }
                
                // 找最后一个线段
                int32_t last_idx = -1;
                for (size_t i = 0; i < indices.size(); ++i) {
                    if (lines[indices[i]]->id == group.line_ids.back()) {
                        last_idx = indices[i];
                        break;
                    }
                }
                
                if (first_idx >= 0 && last_idx >= 0) {
                    group.merged_start = group.need_reverse[0] ? 
                        lines[first_idx]->end : lines[first_idx]->start;
                    group.merged_end = group.need_reverse.back() ?
                        lines[last_idx]->start : lines[last_idx]->end;
                }
            }
        }
        
        result.push_back(std::move(group));
    }
    
    return result;
}

} // namespace path_planner
```

### 4.5 集成到路径规划器

```cpp
// 在 path_planner.cpp 中使用

std::vector<RouteSegment> PathPlanner::planPaths(const CADData& cad_data) {
    // 1. 预处理：拆分Polyline
    CADData preprocessed = GeometryPreprocessor::preprocess(cad_data);
    
    // 2. 提取所有LINE类型的几何
    std::vector<const Line*> all_lines;
    for (const auto& line_ptr : preprocessed.path_lines) {
        if (line_ptr && line_ptr->type == GeometryType::LINE) {
            all_lines.push_back(line_ptr.get());
        }
    }
    
    // 3. 共线合并
    CollinearMerger merger(config_.distance_tolerance, config_.angle_tolerance);
    auto merge_groups = merger.merge(all_lines);
    
    // 4. 基于合并组进行路径规划
    std::vector<RouteSegment> routes;
    
    for (const auto& group : merge_groups) {
        RouteSegment segment;
        segment.type = RouteType::DRAWING_PATH;
        
        // 使用合并后的起点和终点
        segment.points.push_back(group.merged_start);
        segment.points.push_back(group.merged_end);
        
        // 保存原始线段ID（用于追溯）
        if (!group.line_ids.empty()) {
            segment.line_id = group.line_ids[0];  // 主ID
            segment.merged_line_ids = group.line_ids;  // 所有合并的ID
        }
        
        routes.push_back(segment);
    }
    
    // 5. 路径排序和转场路径生成
    // ... 后续处理
    
    return routes;
}
```

---

## 五、配置参数扩展

在 `config/planner.yaml` 中添加：

```yaml
geometry_preprocessing:
  # Polyline拆分配置
  split_polyline: true               # 是否启用Polyline拆分
  preserve_polyline_info: true       # 是否保留原始Polyline信息
  
  # 共线合并配置
  merge_collinear: true              # 是否启用共线合并
  distance_tolerance: 0.001          # 端点距离容差(米) - 1mm
  angle_tolerance: 0.001             # 方向角度容差(弧度) - 约0.057度
  min_segment_length: 0.0005         # 最小线段长度(米) - 过短的线段可能导致合并问题
```

---

## 六、单元测试

```cpp
// test_geometry_preprocessing.cpp
#include <gtest/gtest.h>
#include "geometry_preprocessor.hpp"
#include "collinear_merger.hpp"

namespace path_planner {
namespace test {

// 测试Polyline拆分
TEST(GeometryPreprocessorTest, SplitSimplePolyline) {
    Polyline poly;
    poly.id = 1;
    poly.vertices = {
        {0, 0, 0},
        {1, 0, 0},
        {1, 1, 0},
        {0, 1, 0}
    };
    poly.closed = false;
    
    CADData cad_data;
    cad_data.path_lines.push_back(std::make_unique<Polyline>(poly));
    
    auto result = GeometryPreprocessor::preprocess(cad_data);
    
    // 4个顶点应产生3条线段
    ASSERT_EQ(result.path_lines.size(), 3);
    
    for (const auto& line : result.path_lines) {
        EXPECT_EQ(line->type, GeometryType::LINE);
    }
}

TEST(GeometryPreprocessorTest, SplitClosedPolyline) {
    Polyline poly;
    poly.id = 2;
    poly.vertices = {
        {0, 0, 0},
        {1, 0, 0},
        {1, 1, 0}
    };
    poly.closed = true;
    
    CADData cad_data;
    cad_data.path_lines.push_back(std::make_unique<Polyline>(poly));
    
    auto result = GeometryPreprocessor::preprocess(cad_data);
    
    // 闭合三角形应产生3条线段
    ASSERT_EQ(result.path_lines.size(), 3);
}

// 测试共线合并
TEST(CollinearMergerTest, MergeTwoCollinearLines) {
    Line line1, line2;
    line1.id = 1;
    line1.type = GeometryType::LINE;
    line1.start = {0, 0, 0};
    line1.end = {1, 0, 0};
    
    line2.id = 2;
    line2.type = GeometryType::LINE;
    line2.start = {1, 0, 0};
    line2.end = {2, 0, 0};
    
    std::vector<const Line*> lines = {&line1, &line2};
    
    CollinearMerger merger;
    auto groups = merger.merge(lines);
    
    // 两条共线相连的线段应合并为一组
    ASSERT_EQ(groups.size(), 1);
    EXPECT_EQ(groups[0].line_ids.size(), 2);
    
    // 合并后的起点终点
    EXPECT_NEAR(groups[0].merged_start.x, 0, 0.001);
    EXPECT_NEAR(groups[0].merged_end.x, 2, 0.001);
}

TEST(CollinearMergerTest, NoMergeForNonCollinear) {
    Line line1, line2;
    line1.id = 1;
    line1.type = GeometryType::LINE;
    line1.start = {0, 0, 0};
    line1.end = {1, 0, 0};
    
    line2.id = 2;
    line2.type = GeometryType::LINE;
    line2.start = {1, 0, 0};
    line2.end = {1, 1, 0};  // 垂直于line1
    
    std::vector<const Line*> lines = {&line1, &line2};
    
    CollinearMerger merger;
    auto groups = merger.merge(lines);
    
    // 垂直的两条线段不应合并
    ASSERT_EQ(groups.size(), 2);
}

TEST(CollinearMergerTest, MergeWithGap) {
    Line line1, line2;
    line1.id = 1;
    line1.type = GeometryType::LINE;
    line1.start = {0, 0, 0};
    line1.end = {1, 0, 0};
    
    line2.id = 2;
    line2.type = GeometryType::LINE;
    line2.start = {1.5, 0, 0};  // 有0.5m的间隙
    line2.end = {2, 0, 0};
    
    std::vector<const Line*> lines = {&line1, &line2};
    
    CollinearMerger merger(0.001);  // 1mm容差
    auto groups = merger.merge(lines);
    
    // 有间隙的共线线段不应合并
    ASSERT_EQ(groups.size(), 2);
}

} // namespace test
} // namespace path_planner
```

---

## 七、性能优化建议

### 7.1 共线检测优化

对于大量线段的场景，O(n²)的全对比较可能较慢。可采用以下优化策略：

```cpp
// 使用空间索引加速
class SpatialIndex {
public:
    void insert(int32_t line_id, const BoundingBox& bbox);
    std::vector<int32_t> query(const BoundingBox& bbox);
    
private:
    // R-tree或Grid-based索引
};

// 使用方向哈希预过滤
class DirectionBucket {
public:
    // 将方向向量离散化到若干个bucket
    int getBucket(const Point3D& direction);
    
    // 只比较同一bucket或相邻bucket的线段
    std::vector<int32_t> getPotentialMatches(int bucket);
};
```

### 7.2 并行处理

```cpp
// 使用OpenMP并行化共线检测
#pragma omp parallel for schedule(dynamic)
for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
        if (areCollinear(*lines[i], *lines[j])) {
            #pragma omp critical
            {
                // 记录共线关系
            }
        }
    }
}
```

---

## 八、边界情况处理

| 边界情况 | 处理策略 |
|---------|---------|
| 空Polyline | 跳过，不产生任何线段 |
| 单点Polyline | 跳过，无法形成线段 |
| 两点Polyline | 产生一条线段 |
| 零长度线段 | 过滤掉，不参与合并 |
| 三条及以上共线线段 | 递归合并为一条路径 |
| 分叉的共线结构 | 各分支分别合并 |
| 浮点精度问题 | 使用容差比较，避免严格相等判断 |

---

## 九、实施计划

| 阶段 | 任务 | 预计工时 |
|-----|------|---------|
| 1 | 数据结构扩展（LineSegment等） | 2h |
| 2 | GeometryPreprocessor实现 | 4h |
| 3 | CollinearMerger实现 | 6h |
| 4 | 集成到主流程 | 2h |
| 5 | 单元测试编写 | 4h |
| 6 | 集成测试与调试 | 4h |
| 7 | 性能优化（如需要） | 4h |

**总计：约26工时**

---

## 十、总结

本方案通过两个核心模块实现了用户需求：

1. **GeometryPreprocessor**：在CAD解析后、路径规划前，将Polyline拆分为独立Line，同时保留原始关联信息，便于后续追溯和输出。

2. **CollinearMerger**：在路径规划阶段，使用并查集算法高效地识别和合并共线且相连的直线线段，生成优化后的连续路径。

两个模块相互独立，可以单独启用或禁用，通过配置文件灵活控制。实现遵循现有代码风格，对原有架构影响最小。
