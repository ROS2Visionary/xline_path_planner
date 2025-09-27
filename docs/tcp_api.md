# TCP 接入指南（xline_path_planner）

本文档面向“非 ROS 客户端”的外部调用者，介绍如何通过内置 TCP 服务触发路径规划任务。协议极简：一行请求，一行 JSON 响应。

---

## 总览
- 进程：`ros2 run xline_path_planner planner_node`
- 监听地址与端口：由配置项 `tcp_server.host` 与 `tcp_server.port` 指定
- 请求内容：仅发送 CAD 文件名（不含路径），以换行 `\n` 结束
- 响应内容：单行 JSON，形如 `{"success":true, "error":"", "message":"..."}`，随后服务器主动断开连接

相关配置：
- `cad_parser.cad_file_dir`：CAD 文件所在目录，服务器用它与文件名拼出完整路径
- `visualization.output_dir`：图像输出目录（栅格图、路径图）
- `visualization.planned_output_dir`：规划结果 JSON 输出目录
- `tcp_server.*`：TCP 服务开关与监听参数

参考：xline_path_planner/config/planner.yaml:12、xline_path_planner/config/planner.yaml:36

---

## 配置与启动
1) 编辑配置文件 `planner.yaml`（安装后位于 share/xline_path_planner/config）：
```yaml
cad_parser:
  cad_file_dir: "/root/xline_path_planner/cad_data"   # 服务器端 CAD 文件目录

tcp_server:
  enabled: true
  host: "0.0.0.0"
  port: 52088

visualization:
  output_dir: "/root/xline_path_planner/path_visualizations"
```

2) 启动节点
```bash
ros2 run xline_path_planner planner_node
```
日志中可见监听提示：`TCP server listening on 0.0.0.0:52088`。

---

## 协议定义
- 连接：TCP，短连接，1 请求/连接。
- 请求：客户端发送一行 UTF-8 文本，仅包含 `file_name`，末尾必须有 `\n`。
  - 示例：`cad_2_transformed.json\n`
- 服务端行为：
  - 若 `file_name` 为相对路径，则与 `cad_parser.cad_file_dir` 拼接为完整路径
  - 读取并解析 CAD → 栅格化 → 规划 → 可选可视化与保存结果
- 响应：发送一行 JSON，随后主动关闭连接。
  - 字段：
    - `success`：true/false
    - `error`：错误信息（失败时）
    - `message`：成功消息，包含输出目录等提示

成功示例：
```json
{"success": true, "error": "", "message": "规划完成。JSON 目录: /root/xline_path_planner/planned_results；可视化目录: /root/xline_path_planner/path_visualizations"}
```
失败示例：
```json
{"success": false, "error": "CAD 文件不存在: /root/xline_path_planner/cad_data/missing.json", "message": ""}
```

---

## 快速调用示例
- netcat（nc）
```bash
printf "cad_2_transformed.json\n" | nc 127.0.0.1 52088
```

- Python（标准库 socket）
```python
import socket, json
HOST, PORT = '127.0.0.1', 52088
fname = 'cad_2_transformed.json'  # 仅文件名
with socket.create_connection((HOST, PORT), timeout=60) as s:
    s.sendall((fname + '\\n').encode('utf-8'))
    data = s.recv(8192)
print(json.loads(data.decode('utf-8')))
```

- Node.js
```js
const net = require('net');
const client = net.createConnection({ host: '127.0.0.1', port: 52088 }, () => {
  client.write('cad_2_transformed.json\n');
});
client.on('data', (data) => {
  console.log(data.toString());
  client.end();
});
```

---

## 输出产物
当配置 `visualization.save_path_visualization: true` 时，将在 `visualization.output_dir` 目录生成：
- `grid_map.<image_format>`：栅格图
- `planned_paths.<image_format>`：路径可视化图
- `planned_<文件名stem>.json`：规划结果 JSON

例如：`planned_cad_2_transformed.json` 位于 `visualization.planned_output_dir` 指定的目录；图像位于 `visualization.output_dir`。

---

## 超时与并发
- 规划为 CPU 密集任务，请为客户端设置较大的读取超时（建议 ≥60s，视 CAD 复杂度而定）。
- 服务器对规划过程加互斥锁，任务顺序执行；多客户端会排队，吞吐由单次规划耗时决定。

---

## 安全与约束
- 建议仅传“纯文件名”，不要包含路径分隔符；服务端会与配置的 `cad_file_dir` 拼接。
- 请将服务监听在内网，或用防火墙限制来源地址。
- 不建议对公网开放；协议无认证/加密与流量控制。

---

## 故障排查
- 报“CAD 文件不存在”：确认文件已放在 `cad_parser.cad_file_dir` 指定的目录，且文件名拼写正确。
- 报“解析失败/栅格失败/路径规划失败”：检查 CAD JSON 格式是否满足解析要求，或调整 `grid_map`、`offsets`、`path_planner` 等参数。
- 无响应或被动断开：查看节点日志（终端输出），确认 TCP 服务已启用并监听到正确地址与端口。

---

## 版本说明
- 响应 `message` 内包含输出目录与 JSON 文件路径，当前未包含详细指标与告警。如需返回更多信息，请改用 ROS Service 或扩展 TCP 协议。
