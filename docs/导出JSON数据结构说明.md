导出 JSON 数据结构说明（规划结果）
=================================

本文档描述 xline_path_planner 输出的“CAD 风格 JSON”结构，适用于由 `xline_path_planner/src/output_formatter.cpp` 生成的规划结果文件（例如 `path_visualizations/path_from_cad.json`）。该结构用于前端可视化与后续处理，不等同于原始 CAD 解析输入文件。

一、顶层结构
-------------

- 根对象仅包含一个键：`lines`
- `lines` 为几何实体数组，数组顺序与规划的段序一致，可用每个元素内的 `order` 字段进行确认

示例：
```json
{
  "lines": [ { /* entity #0 */ }, { /* entity #1 */ } ]
}
```

二、坐标与单位
---------------

- 坐标单位：毫米（mm）。系统内部计算使用米（m），在导出阶段统一将所有坐标与半径从 m 转换为 mm（×1000）。
- 角度单位：度（degree）。弧线/椭圆等角度字段导出为度。
- 颜色：字符串，十六进制 `#RRGGBB`。
- 不透明度：`opacity`，0.0～1.0 的浮点数（可选字段）。

三、通用字段（所有实体可能包含）
--------------------------------

- `id`: int，实体 ID。
- `type`: string，几何类型，小写：`line` | `polyline` | `circle` | `arc`。
- `line_type`: string，线型名称（例如 `"continuous"` 或空字符串）。
- `thickness`: float，线宽。
- `hidden`: bool，是否隐藏。
- `layer`: string，图层名称。
- `layer_id`: int，可选，图层 ID（若源数据可用）。
- `color`: string，颜色，形如 `#FFFFFF`。
- `opacity`: float，可选，不透明度（0~1）。当前仅对转场线统一输出 `0.5`。
- `selected`: bool，可选，是否被选中（若源数据包含）。
- `order`: int，导出顺序（同规划段索引）。
- `work`: bool，是否为“工作段”（绘图）
  - `true` 表示绘图路径（DRAWING_PATH）
  - `false` 表示转场路径（TRANSITION_PATH）
- `start`: `{ x: float, y: float }`，本段几何的起点（详见类型说明）
- `end`: `{ x: float, y: float }`，本段几何的终点（详见类型说明）

说明：
- 圆与圆弧同样包含 `start/end`，用以明确该段在实际执行路径上的进入/离开点；与纯几何定义保持兼容同时兼顾路径消费端的简化解析。
- `selected=false` 的实体在 CAD 解析阶段会被跳过；导出结果中该字段仅用于记录源元数据，消费端不要以此作为唯一过滤依据。

四、按类型的附加字段
----------------------

1) 直线（`type: "line"`）
- 必选：`start`, `end`
- 示例：
```json
{
  "id": 1,
  "type": "line",
  "line_type": "continuous",
  "thickness": 1.0,
  "hidden": false,
  "layer": "路径",
  "color": "#FFFFFF",
  "order": 0,
  "work": true,
  "start": {"x": 0.0, "y": 0.0},
  "end":   {"x": 100.0, "y": 0.0}
}
```

2) 多段线（`type: "polyline"`）
- 必选：`vertices: [{x,y}, ...]`
- 同时补充 `start/end` 为首尾顶点，便于仅解析起止点的消费端兼容。
- 示例：
```json
{
  "id": 2,
  "type": "polyline",
  "line_type": "continuous",
  "thickness": 1.0,
  "hidden": false,
  "layer": "路径",
  "color": "#FFFFFF",
  "order": 1,
  "work": true,
  "vertices": [{"x":0,"y":0},{"x":10,"y":0},{"x":10,"y":10}],
  "start": {"x":0,"y":0},
  "end":   {"x":10,"y":10}
}
```

3) 圆（`type: "circle"`）
- 必选：`center: {x,y}`, `radius: float`（单位：mm）
- 额外提供：`start/end`（规划实际的进入/离开点）。
- 示例：
```json
{
  "id": 3,
  "type": "circle",
  "line_type": "continuous",
  "thickness": 1.0,
  "hidden": false,
  "layer": "路径",
  "color": "#00FF00",
  "order": 2,
  "work": true,
  "center": {"x": 50.0, "y": 50.0},
  "radius": 25.0,
  "start": {"x": 75.0, "y": 50.0},
  "end":   {"x": 75.0, "y": 50.0}
}
```

4) 圆弧（`type: "arc"`）
- 必选：`center: {x,y}`, `radius: float`（单位：mm）, `start_angle: float`, `end_angle: float`（单位：度）
- 额外提供：`start/end`（基于角度与半径计算的端点），便于消费端无需三角运算即可获取端点。
- 示例：
```json
{
  "id": 4,
  "type": "arc",
  "line_type": "continuous",
  "thickness": 1.0,
  "hidden": false,
  "layer": "路径",
  "color": "#FF0000",
  "order": 3,
  "work": true,
  "center": {"x": 0.0, "y": 0.0},
  "radius": 10.0,
  "start_angle": 0.0,
  "end_angle": 90.0,
  "start": {"x": 10.0, "y": 0.0},
  "end":   {"x": 0.0,  "y": 10.0}
}
```

五、转场路径约定（TRANSITION_PATH）
----------------------------------

规划中的“转场段”以直线实体输出，字段统一约定如下：

- `id`: `1000000`
- `type`: `"line"`
- `line_type`: `""`（空字符串）
- `thickness`: `1.0`
- `hidden`: `false`
- `selected`: `false`
- `layer`: `"TRANSITION"`
- `layer_id`: `1000000`
- `color`: `"#D3D3D3"`
- `opacity`: `0.5`
- `work`: `false`
- `order`: 段索引
- `start`/`end`: 该转场段的起止点

示例：
```json
{
  "id": 1000000,
  "type": "line",
  "line_type": "",
  "thickness": 1.0,
  "hidden": false,
  "selected": false,
  "layer": "TRANSITION",
  "layer_id": 1000000,
  "color": "#D3D3D3",
  "opacity": 0.5,
  "order": 10,
  "work": false,
  "start": {"x": -1.0, "y": 0.0},
  "end":   {"x": -2.0, "y": 0.0}
}
```

六、与旧版差异
--------------

- 顶层不再包含 `metadata` 等信息，仅保留 `lines`。
- 新增了 `order` 与 `work` 字段以区分段顺序与是否绘图路径。
- 圆（circle）与圆弧（arc）新增 `start/end` 字段，便于消费端快速获取端点。
- 转场路径（TRANSITION）以固定样式导出，统一颜色和不透明度。

七、消费端建议
--------------

- 按 `order` 字段进行绘制顺序控制；`lines` 的数组顺序与 `order` 一致，可直接使用其一。
- 若仅需“绘图路径”，可筛选 `work=true`；若需展示“转场路径”，使用 `work=false` 项，并可利用 `opacity` 进行半透明显示。
- 对于弧线，使用 `center/radius/start_angle/end_angle` 可进行精确绘制；亦可直接使用 `start/end` 进行快速端点标注。

版本
----

- 本说明与实现同步于：`xline_path_planner/src/output_formatter.cpp` 当前版本。
