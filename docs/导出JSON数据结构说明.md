# 导出 JSON 数据结构说明

本文档说明 CADFlow Pro 导出 JSON 的数据结构、字段含义与示例，并标注与旧版导出格式的兼容差异。

参考实现与样例：
- 新版导出（标准格式）：`src/services/file_service.py`
- 旧版导出（兼容格式）：`src/ui/main_window.py`
- 新版实际样例：`other/cad.json`
- 旧版示例：`sample_export.json`

## 顶层结构（新版标准格式）

导出文件为一个对象，包含以下部分：`metadata`、`linetypes`、`layers`、`lines`、`base_points`（始终存在，可能为空）。

```json
{
  "metadata": {
    "source_file": "string",
    "export_timestamp": "YYYY-MM-DD HH:MM:SS",
    "total_layers": 0,
    "total_lines": 0,
    "total_linetypes": 0,
    "app_version": "x.y.z",
    "export_filter": "default_layers_only",
    "base_point_count": 0
  },
  "linetypes": [
    // 线型定义对象...
  ],
  "layers": [
    {
      "name": "string",
      "color": "#RRGGBB" | {"hex": "#RRGGBB", "rgb": {"r": 0, "g": 0, "b": 0}},
      "visible": true,
      "locked": false,
      "line_count": 0,
      "description": "string"
    }
  ],
  "lines": [
    {
      "id": 1,
      "type": "line|circle|arc|polyline|spline|ellipse|text",
      "line_type": "continuous|dashed|...",
      "thickness": 1.0,
      "hidden": false,
      "layer": "string",
      "color": "#RRGGBB",

      "start": {"x": 0.0, "y": 0.0},        
      "end":   {"x": 0.0, "y": 0.0}         
    }
  ]
  ,
  "base_points": [               // 基准点数组（用于标记参考点；无则为空数组）
    { "x": 0.0, "y": 0.0 },
    { "x": 100.0, "y": 50.0 }
  ]
}
```

说明：
- `metadata.source_file`：原始文件名，若未知则为 `"unknown"`。
- `metadata.export_timestamp`：导出时间，格式 `YYYY-MM-DD HH:MM:SS`。
- `metadata.total_layers` / `total_lines` / `total_linetypes`：统计信息（仅统计导出的数据）。
- `metadata.export_filter`：导出过滤器，当前为 `"default_layers_only"`（仅导出“默认图层”，详见“导出过滤器”）。
- `metadata.base_point_count`：导出的基准点数量（与 `base_points` 数组长度一致）。
- `linetypes`：线型定义列表（数组）。仅导出“被实体实际引用”的线型定义；包含简单与复杂两类（见下文）。
- `layers`：图层列表（数组）。每个元素描述一个图层。**注意：只包含可见图层，且忽略线条数为0的空图层**。
- `lines`：几何实体列表（数组）。每个元素是一条线/圆/弧/多段线/样条/椭圆等。**注意：只包含可见图层的实体**。
- `base_points`：基准点坐标列表（数组，可能为空）。用于将参考点随数据一起保存与回读，通常为1~3个点。
  - 导出时始终包含此字段；如当前没有基准点，则为 `[]`；
  - 导入时，应用会在图纸上绘制编号圆点标记；
  - 兼容旧字段：若存在 `base_point: {x,y}` 也会被识别为1个基准点。

## 线型定义对象（linetypes[]）

线型定义（包含简单和复杂线型）。复杂线型支持包含文字和形状的字体型线型。

- `name`: 线型名称（string，必填，如"WATER_LINE"）
- `description`: 线型描述（string，可为空）
- `pattern_length`: 模式总长度（float）
- `is_complex`: 是否为复杂线型（bool，包含文字或形状时为true）
- `elements`: 线型元素数组（array）

说明：
- 简单线型（如 `Continuous`、`ByBlock`、`DASHED`）通常 `is_complex=false`，其 `elements` 可能为空数组（见 `other/cad.json`）。
- 仅当线型为复杂线型时，`elements` 才会包含文字或形状等元素定义。

### 线型元素对象（elements[]）

每个线型元素定义线型模式的一部分：

- `type`: 元素类型（"dash" | "dot" | "space" | "text" | "shape"）
- `length`: 元素长度（float，space和dash类型使用）

文字和形状元素的额外属性：
- `text`: 文字内容（string，text类型使用）
- `shape_name`: 形状名称（string，shape类型使用）
- `shape_file`: SHX字体文件名（string，shape类型使用）
- `scale`: 缩放比例（float，默认1.0）
- `rotation`: 旋转角度（float，度数，默认0.0）
- `x_offset`: X方向偏移（float，默认0.0）
- `y_offset`: Y方向偏移（float，默认0.0）

导出来源：`src/services/file_service.py`

## 图层对象（layers[]）

- `name`: 图层名称（string，必填）
- `color`: 颜色。导出时统一序列化为 `"#RRGGBB"` 字符串；导入时兼容对象 `{ hex, rgb }`（详见“颜色表示”）
- `visible`: 是否可见（bool，默认 true）
- `locked`: 是否锁定（bool，默认 false）
- `line_count`: 该图层的实体数量（int）
- `description`: 描述（string，可为空字符串）

导出来源：`src/services/file_service.py`

## 几何实体对象（lines[]）

通用字段：
- `id`: 递增的实体ID（int，从1开始）
- `type`: 几何类型（小写）
  - 取值：`line` | `circle` | `arc` | `polyline` | `spline` | `ellipse` | `text`
- `line_type`: 线型（小写存储；导入时大小写不敏感）
- `has_complex_linetype`: 是否使用复杂线型（bool，可选；仅当关联的线型定义为复杂线型时出现）
- `linetype_name`: 复杂线型名称（string，可选；与上条同条件）
- `thickness`: 线宽（float）
- `hidden`: 是否隐藏（bool）
- `layer`: 所属图层名（string）
- `color`: 十六进制颜色，如 `"#FFFFFF"`。导出时总是为字符串；导入时若缺失，默认 `#FFFFFF`。

按类型的附加字段：

1) `line` 直线
- `start`: `{ x: float, y: float }`
- `end`: `{ x: float, y: float }`

2) `circle` 圆
- `center`: `{ x: float, y: float }`
- `radius`: `float`

3) `arc` 弧
- `center`: `{ x: float, y: float }`
- `radius`: `float`
- `start_angle`: `float`（度）
- `end_angle`: `float`（度）

4) `polyline` 多段线
- `vertices`: `[{ x: float, y: float }, ...]`
- `closed`: `bool`（是否闭合）
- `bulges`: `[float, ...]`（可选；与边对应的弧度）

5) `spline` 样条
- `degree`: `int`（默认 3）
- `control_points`: `[{ x: float, y: float }, ...]`
- `knots`: `[float, ...]`（可选）
- `weights`: `[float, ...]`（可选）
- `periodic`: `bool`（可选；是否周期/闭合样条）

6) `ellipse` 椭圆
- `center`: `{ x: float, y: float }`
- `major_axis`: `{ x: float, y: float }`（主轴方向向量分量）
- `ratio`: `float`（次轴/主轴比）
- `start_angle`: `float`（度）
- `end_angle`: `float`（度）
- `rotation`: `float`（主轴相对 X 轴旋转角，度）

导出来源：`src/services/file_service.py`

7) `text` 文字（TEXT/MTEXT）
- 通用字段：
  - `subtype`: `"text" | "mtext"`（原始DXF实体类型）
  - `position`: `{ x: float, y: float }`（插入点）
  - `height`: `float`（TEXT: `height`；MTEXT: `char_height`）
  - `rotation`: `float`（度）
  - `style`: `string`（文字样式名）
  - `content`: `string`（纯文本，去除格式标记；支持中文）
  - `raw`: `string`（可选；原始内容，保留格式标记）
- TEXT 专有（可选）：
  - `width_factor`: `float`（宽度比例，默认 `1.0`）
  - `oblique`: `float`（倾斜角，度）
  - `align`: `{ horizontal?: "left|center|right|aligned|middle|fit", vertical?: "baseline|bottom|middle|top" }`
- MTEXT 专有（可选）：
  - `box_width`: `float`（文本框宽度）
  - `attachment`: `"top_left|top_center|top_right|middle_left|middle_center|middle_right|bottom_left|bottom_center|bottom_right"`
  - `direction`: `"left_to_right|top_to_bottom|by_style"`

导出来源：`src/services/file_service.py`

## 取值与单位约定

- 角度：单位为“度”（degree）。
- 坐标：与原始CAD一致，不做单位转换（保留导入/解析时的单位）。
- `line_type`：常见取值 `continuous`、`dashed`、`dotted` 等；导入时大小写不敏感，并会转换为大写存储在模型中；导出为小写字符串。
- 颜色表示：
  - 导出统一为字符串 `#RRGGBB`；
  - 导入兼容对象 `{ "hex": "#RRGGBB", "rgb": { "r": 255, "g": 255, "b": 255 } }`；
  - 导入兼容旧字段组合 `{ r, g, b }` 或 `{ red, green, blue }`；
  - 颜色解析与容错见 `src/services/file_service.py`。

## 旧版导出格式（兼容）

旧版导出使用图层字典结构，将实体嵌入到每个图层下：

```json
{
  "version": "1.0",
  "created_by": "CADFlow Pro",
  "export_time": "YYYY-MM-DD HH:MM:SS",
  "total_layers": 3,
  "total_lines": 4,
  "layers": {
    "建筑轮廓": {
      "name": "建筑轮廓",
      "color": { "hex": "#dc143c", "rgb": {"r":220,"g":20,"b":60} },
      "visible": true,
      "locked": false,
      "description": "...",
      "line_count": 2,
      "lines": [ /* 实体数组（结构与上文一致，但无 layer/color 字段） */ ]
    }
  }
}
```

差异点：
- `layers` 为“字典（以图层名为 key）”，而不是“数组”；
- 实体数据嵌套在对应图层内；实体通常不包含 `layer` 和 `color` 字段；
- 元数据键为 `version/created_by/export_time` 等，而非 `metadata` 对象；
- 示例见：`sample_export.json:1`；旧版实现：`src/ui/main_window.py:1791`。

导入兼容性：
- 导入流程同时支持新版与旧版两种结构（见 `src/services/file_service.py`）。

## 极简示例（新版标准格式）

```json
{
  "metadata": {
    "source_file": "example.dxf",
    "export_timestamp": "2025-09-07 12:00:00",
    "total_layers": 1,
    "total_lines": 1,
    "total_linetypes": 0,
    "app_version": "1.0.0",
    "export_filter": "default_layers_only",
    "base_point_count": 0
  },
  "layers": [
    {
      "name": "空洞",
      "color": "#6c757d",
      "visible": true,
      "locked": false,
      "line_count": 1,
      "description": "用于标注孔洞或留空区域"
    }
  ],
  "lines": [
    {
      "id": 1,
      "type": "line",
      "line_type": "continuous",
      "thickness": 1.0,
      "hidden": false,
      "layer": "空洞",
      "color": "#FFFFFF",
      "start": {"x": 0.0, "y": 0.0},
      "end":   {"x": 100.0, "y": 0.0}
    }
  ]
  ,
  "base_points": []
}
```

## 实体示例片段（不同类型）

1) 圆（circle）
```json
{
  "id": 2,
  "type": "circle",
  "line_type": "continuous",
  "thickness": 1.0,
  "hidden": false,
  "layer": "空洞",
  "color": "#00FF00",
  "center": {"x": 50.0, "y": 50.0},
  "radius": 25.0
}
```

2) 弧（arc）
```json
{
  "id": 3,
  "type": "arc",
  "line_type": "continuous",
  "thickness": 1.0,
  "hidden": false,
  "layer": "空洞",
  "color": "#FF0000",
  "center": {"x": 0.0, "y": 0.0},
  "radius": 10.0,
  "start_angle": 0.0,
  "end_angle": 90.0
}
```

3) 多段线（polyline）
```json
{
  "id": 4,
  "type": "polyline",
  "line_type": "continuous",
  "thickness": 1.0,
  "hidden": false,
  "layer": "空洞",
  "color": "#0000FF",
  "vertices": [{"x":0,"y":0},{"x":10,"y":0},{"x":10,"y":10}],
  "closed": false,
  "bulges": [0.0, 0.5]
}
```

4) 样条（spline）
```json
{
  "id": 5,
  "type": "spline",
  "line_type": "continuous",
  "thickness": 1.0,
  "hidden": false,
  "layer": "曲线",
  "color": "#333333",
  "degree": 3,
  "control_points": [{"x":0,"y":0},{"x":5,"y":10},{"x":10,"y":0}],
  "knots": [0,0,0,1,1,1],
  "weights": [1,1,1],
  "periodic": false
}
```

5) 椭圆（ellipse）
```json
{
  "id": 6,
  "type": "ellipse",
  "line_type": "continuous",
  "thickness": 1.0,
  "hidden": false,
  "layer": "辅助",
  "color": "#888888",
  "center": {"x": 0, "y": 0},
  "major_axis": {"x": 10, "y": 0},
  "ratio": 0.5,
  "start_angle": 0.0,
  "end_angle": 360.0,
  "rotation": 0.0
}
```

6) 文字（text - TEXT）
```json
{
  "id": 7,
  "type": "text",
  "line_type": "continuous",
  "thickness": 1.0,
  "hidden": false,
  "layer": "空洞",
  "color": "#FFFFFF",
  "subtype": "text",
  "position": {"x": 120.0, "y": 45.0},
  "height": 5.0,
  "rotation": 0.0,
  "style": "仿宋0.7",
  "content": "跑道号码标志",
  "width_factor": 0.7,
  "oblique": 0.0,
  "align": {"horizontal": "left", "vertical": "baseline"}
}
```

7) 文字（text - MTEXT）
```json
{
  "id": 8,
  "type": "text",
  "line_type": "continuous",
  "thickness": 1.0,
  "hidden": false,
  "layer": "空洞",
  "color": "#FFFFFF",
  "subtype": "mtext",
  "position": {"x": 5788.13, "y": 1602.20},
  "height": 4.0,
  "rotation": 0.0,
  "style": "仿宋0.7",
  "content": "(单位:m)",
  "box_width": 45.912,
  "attachment": "top_left",
  "direction": "left_to_right"
}
```

## 导入行为（回读）

- 导入支持新版与旧版两种结构；
- 颜色解析兼容字符串 `#RRGGBB`、`{hex, rgb}`、`{r,g,b}`/`{red,green,blue}`；
- 缺失的实体字段按默认值处理：
  - `color` 默认 `#FFFFFF`；
  - `layer` 默认 `"Default"`；
  - `line_type` 默认 `continuous`，导入时转大写存储；
- 详见：`src/services/file_service.py`。

当前样例包含的实体类型：`line`、`text(text/mtext)`、`polyline`、`arc`、`circle`（见 `other/cad.json`）。

## 复杂线型示例

### 1. 给水管道线型定义
```json
{
  "name": "WATER_LINE",
  "description": "给水管道线型",
  "pattern_length": 40.0,
  "is_complex": true,
  "elements": [
    {
      "type": "dash",
      "length": 15.0
    },
    {
      "type": "text",
      "length": 0.0,
      "text": "水",
      "scale": 1.2,
      "rotation": 0.0,
      "x_offset": 0.0,
      "y_offset": -2.0
    },
    {
      "type": "dash", 
      "length": 15.0
    },
    {
      "type": "space",
      "length": 10.0
    }
  ]
}
```

### 2. 使用复杂线型的线条实体
```json
{
  "id": 1,
  "type": "line",
  "line_type": "water_line",
  "has_complex_linetype": true,
  "linetype_name": "WATER_LINE",
  "thickness": 1.5,
  "hidden": false,
  "layer": "给水系统",
  "color": "#0066CC",
  "start": {"x": 10.0, "y": 50.0},
  "end": {"x": 200.0, "y": 50.0}
}
```

### 3. 包含形状的电缆线型
```json
{
  "name": "CABLE_LINE",
  "description": "电缆线型",
  "pattern_length": 30.0,
  "is_complex": true,
  "elements": [
    {
      "type": "dash",
      "length": 10.0
    },
    {
      "type": "shape",
      "length": 0.0,
      "shape_name": "LIGHTNING",
      "shape_file": "electrical.shx",
      "scale": 0.8,
      "rotation": 0.0,
      "x_offset": 0.0,
      "y_offset": 0.0
    },
    {
      "type": "dash",
      "length": 10.0
    },
    {
      "type": "space",
      "length": 10.0
    }
  ]
}
```

## 导出过滤器

从 v1.0.0 开始，导出功能默认应用过滤器以优化导出数据：

### default_layers_only

- 行为：仅导出“默认图层”（且图层必须可见、且非空）及其实体。
- 默认图层判定：
  - 优先识别图层对象的 `is_default: true` 标记；
  - 若未标记，则按约定名称回退：`"空洞"`、`"障碍物"`、`"路径"`、`"Default"`。
- 过滤细则：
  - 仅保留可见的默认图层；
  - 仅导出这些图层上的实体；
  - 忽略线条数为 0 的空图层；
  - `metadata.total_layers`/`total_lines` 为过滤后的统计值；
  - `metadata.export_filter` 固定为 `"default_layers_only"`。

示例（概念化）：
```json
{
  "metadata": {
    "total_layers": 1,
    "total_lines": 3,
    "export_filter": "default_layers_only"
  },
  "layers": [
    {"name": "空洞", "visible": true, "line_count": 3}
  ],
  "lines": [ /* 仅来自“空洞”等默认图层的实体 */ ]
}
```

## 建议与最佳实践

- 建议统一采用新版标准格式（`metadata + linetypes[] + layers[] + lines[]`）。
- `color` 优先使用 `#RRGGBB` 字符串表示，简化解析。
- 角度统一以"度"为单位；坐标单位保持与输入CAD一致。
- 若面向外部系统消费数据，请固定 `line_type` 枚举集并约定大小写。
- 复杂线型的`pattern_length`应与所有元素长度总和匹配，以获得最佳显示效果。
- 文字元素建议使用适当的`y_offset`避免与线条重叠。
- **导出前检查图层可见性**：确保需要的图层已设为可见状态，隐藏不需要的图层以精简导出数据。
