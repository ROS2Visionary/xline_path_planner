# xline_path_planner Text 支持实现

## 概述

本次修改为 `xline_path_planner` 包添加了对 `text` 类型几何元素的路径规划支持。文字（text）只能使用左边或右边的喷码机，具体使用哪边需要根据机器人运动方向来动态决定。

## 修改的文件

### 1. `include/xline_path_planner/common_types.hpp`

**新增内容：**

- **`GeometryType::TEXT = 8`**：新增文字几何类型枚举值
- **`TextAlign` 结构体**：用于存储文字对齐方式
  ```cpp
  struct TextAlign {
    std::string horizontal = "left";    // left, center, right
    std::string vertical = "baseline";  // top, middle, bottom, baseline
  };
  ```
- **`Text` 结构体**：继承自 `Line`，用于表示文字元素
  ```cpp
  struct Text : public Line {
    Point3D position;           // 文字位置（锚点）
    std::string content;        // 文字内容
    double height;              // 文字高度
    double rotation;            // 旋转角度（度）
    std::string style;          // 字体样式
    double width_factor;        // 宽度因子
    double oblique;             // 倾斜角度
    TextAlign align;            // 对齐方式
    std::string subtype;        // 子类型
    
    double estimateWidth() const;  // 估算文字宽度
    void calculateDrawingBounds(Point3D& out_start, Point3D& out_end) const;  // 计算绘制边界
  };
  ```
- **`RouteSegment` 扩展**：添加了文字相关字段
  ```cpp
  std::string text_content;     // 文字内容
  double text_height = 0.0;     // 文字高度（米）
  double text_rotation = 0.0;   // 文字旋转角度（度）
  ```

### 2. `include/xline_path_planner/cad_parser.hpp`

**新增内容：**

- **`parse_text()` 方法声明**：用于解析 JSON 中的 text 类型元素

### 3. `src/cad_parser.cpp`

**新增内容：**

- **`parse_text()` 方法实现**：
  - 解析文字位置（position）并进行单位转换（毫米→米）
  - 解析文字内容（content）
  - 解析高度（height）并进行单位转换
  - 解析旋转角度（rotation）
  - 解析对齐方式（align）
  - 计算文字的绘制边界（起点和终点）

- **`parse_from_json_obj()` 修改**：
  - 添加对 `type == "text"` 的处理分支
  - 统计解析的文字数量

### 4. `src/path_planner.cpp`

**核心新增内容：**

- **`determineTextPrinter()` 函数**：根据运动方向决定使用哪个打印机
  ```cpp
  PrinterType determineTextPrinter(
    const Point3D& current_position,  // 机器人当前位置
    const Point3D& text_position,     // 文字位置
    double text_rotation              // 文字旋转角度
  );
  ```
  
  **选择策略：**
  1. 计算从当前位置到文字位置的运动方向向量
  2. 计算文字的书写方向（基于 rotation 角度）
  3. 计算运动方向与文字方向的相对角度
  4. 如果相对角度在 [-π/2, π/2] 范围内（即运动方向与文字方向大致一致），使用 **右侧喷码机**
  5. 否则使用 **左侧喷码机**

- **`determineTextPrinterByPosition()` 函数**：备用方案
  - 当没有有效的运动方向时（例如第一个元素是文字）
  - 根据文字相对于画布中心的位置决定打印机
  - 文字在左半部分 → 左侧喷码机
  - 文字在右半部分 → 右侧喷码机

- **`planGeometryPath()` 修改**：
  - 添加对 `GeometryType::TEXT` 的处理
  - 设置 `ink_mode = InkMode::TEXT`
  - 保存文字信息到 RouteSegment
  - 文字路径不应用偏移（保持精确位置）

- **`processGeometryGroup()` 修改**：
  - 添加对文字类型的分类统计
  - 在规划文字路径后，根据运动方向确定打印机类型

### 5. `src/output_formatter.cpp`

**新增内容：**

- **文字类型 JSON 导出**：
  ```json
  {
    "id": 9,
    "type": "text",
    "order": 5,
    "work": true,
    "printed": false,
    "printer_type": "LEFT_PRINTER",
    "ink": {
      "enabled": true,
      "mode": "text",
      "printer": "left"
    },
    "position": {"x": 500.0, "y": 500.0},
    "content": "中心点",
    "height": 50.0,
    "rotation": 0.0,
    "style": "Standard",
    "width_factor": 1.0,
    "oblique": 0.0,
    "subtype": "text",
    "align": {
      "horizontal": "center",
      "vertical": "middle"
    },
    "start": {"x": 475.0, "y": 500.0},
    "end": {"x": 525.0, "y": 500.0}
  }
  ```

## 使用示例

对于 `square_1m.json` 中的文字元素：

```json
{
  "id": 9,
  "type": "text",
  "position": {"x": 500.0, "y": 500.0},
  "height": 50.0,
  "rotation": 0.0,
  "content": "中心点",
  "align": {"horizontal": "center", "vertical": "middle"}
}
```

路径规划器会：
1. 解析文字位置 (500mm, 500mm) → (0.5m, 0.5m)
2. 解析文字高度 50mm → 0.05m
3. 计算文字绘制范围（根据内容长度和对齐方式）
4. 根据机器人到达该文字时的运动方向，选择使用左侧或右侧喷码机
5. 生成路径段，设置 `ink_mode = TEXT`，`printer_type = LEFT_PRINTER 或 RIGHT_PRINTER`

## 打印机选择逻辑详解

```
运动方向 →
文字方向 →

情况1: 运动方向与文字方向大致一致 (相对角度在 -90° 到 90° 之间)
       机器人从左向右经过文字
       → 使用右侧喷码机 (RIGHT_PRINTER)

情况2: 运动方向与文字方向相反 (相对角度超出 -90° 到 90° 范围)
       机器人从右向左经过文字
       → 使用左侧喷码机 (LEFT_PRINTER)
```

## 注意事项

1. 文字路径**不应用偏移**，因为文字位置应该精确
2. 文字**只能使用左侧或右侧喷码机**，不能使用中间喷码机
3. 打印机的选择是动态的，取决于规划时的运动方向
4. 如果文字是第一个规划的元素（没有前一个位置），则根据文字在画布上的位置决定
