# README_1：py-xiaozhi 语音导航与讲解功能扩展说明

## 1. 功能概述

本文档说明在 `py-xiaozhi` 项目中新增的机器人语音导航控制功能。该功能用于连接小智 AI 与 ROS2 导航系统，使用户可以通过自然语言语音指令控制机器人前往指定导航点，并在机器人到达并停稳后，由小智自动播报对应讲解内容。

实现后的完整链路如下：

```text
用户语音指令
    ↓
py-xiaozhi 识别用户意图
    ↓
调用 MCP 工具 self.robot_nav.go_to_waypoint
    ↓
发布 ROS2 Topic：/waypoint_task_cmd
    ↓
My_nav 中的 waypoint_task_executor.py 接收任务并调用 Nav2
    ↓
机器人导航到目标点并停稳
    ↓
My_nav 发布 ROS2 Topic：/xiaozhi_explain_text
    ↓
py-xiaozhi 的 RobotExplainPlugin 监听讲解文本
    ↓
调用小智自身 TTS 能力进行语音讲解
```

该功能实现了“语音指令 - 自主导航 - 到点讲解”的闭环。

---

## 2. 新增文件结构

在 `py-xiaozhi` 项目中新增或修改的主要文件如下：

```text
py-xiaozhi-main/
├── src/
│   ├── mcp/
│   │   ├── mcp_server.py                    # 修改：注册机器人导航 MCP 工具
│   │   └── tools/
│   │       └── robot_nav/
│   │           ├── __init__.py              # 新增：导出机器人导航工具管理器
│   │           ├── manager.py               # 新增：注册 MCP 工具
│   │           ├── tools.py                 # 新增：实现导航点查询与导航指令发布
│   │           └── robot_nav_config.json    # 新增：导航点文件、ROS 环境与别名配置
│   ├── plugins/
│   │   └── robot_explain.py                 # 新增：监听 ROS 讲解文本并调用小智 TTS
│   ├── application.py                       # 修改：注册 RobotExplainPlugin
│   └── plugins/
│       └── manager.py                       # 可选修改：调试阶段打印插件 setup/start 错误
```

---

## 3. MCP 导航工具说明

### 3.1 工具一：`self.robot_nav.list_waypoints`

作用：查询机器人当前可前往的导航点。

适用语音示例：

```text
你能去哪？
有哪些展区？
当前有哪些导航点？
```

返回内容示例：

```text
当前机器人可以前往以下导航点：
- home，可称呼为：起点、出发点
- exhibit_1，可称呼为：一号展区、第一展区
- charge_area，可称呼为：充电区、充电站
```

---

### 3.2 工具二：`self.robot_nav.go_to_waypoint`

作用：控制机器人前往指定导航点。

适用语音示例：

```text
带我去一号展区
导航到充电区
我要去起点
去第二展区看看
```

该工具不会直接控制机器人底盘，也不会直接调用 Nav2。它只负责将目标点名称发布到 ROS2 任务入口：

```text
/waypoint_task_cmd
```

由 My_nav 项目中的 `waypoint_task_executor.py` 负责后续导航、停稳判断与讲解触发。

---

## 4. robot_nav_config.json 配置说明

配置文件路径：

```text
src/mcp/tools/robot_nav/robot_nav_config.json
```

示例：

```json
{
  "ros_setup": "/opt/ros/humble/setup.bash",
  "workspace_setup": "/home/jie/Desktop/My_nav/install/setup.bash",
  "waypoint_file": "/home/jie/Desktop/My_nav/src/rm_nav_bringup/config/waypoints/RMUL2026H.yaml",
  "command_topic": "/waypoint_task_cmd",
  "aliases": {
    "起点": "home",
    "出发点": "home",
    "一号展区": "exhibit_1",
    "1号展区": "exhibit_1",
    "第一展区": "exhibit_1",
    "二号展区": "exhibit_2",
    "2号展区": "exhibit_2",
    "第二展区": "exhibit_2",
    "充电区": "charge_area",
    "充电站": "charge_area"
  }
}
```

字段说明：

| 字段 | 说明 |
|---|---|
| `ros_setup` | ROS2 Humble 环境脚本路径 |
| `workspace_setup` | My_nav 工作空间 install 环境脚本路径 |
| `waypoint_file` | My_nav 中保存导航点的 YAML 文件 |
| `command_topic` | 小智向 ROS2 发布导航任务的 Topic |
| `aliases` | 中文语音名称与实际导航点名称的映射 |

注意：`aliases` 右侧的值必须和 waypoint YAML 文件中 `points:` 下的点名一致。

---

## 5. mcp_server.py 修改说明

在 `src/mcp/mcp_server.py` 的 `add_common_tools()` 中注册机器人导航工具。

推荐添加位置：在已有工具注册之后、`self.tools.extend(original_tools)` 之前。

添加内容：

```python
# 添加机器人导航工具
from src.mcp.tools.robot_nav import get_robot_nav_manager

robot_nav_manager = get_robot_nav_manager()
robot_nav_manager.init_tools(self.add_tool, PropertyList, Property, PropertyType)
```

原因：`add_common_tools()` 会先清空工具列表，再注册通用工具，最后恢复原有工具。因此新工具必须在 `self.tools.extend(original_tools)` 之前注册。

---

## 6. RobotExplainPlugin 功能说明

新增文件：

```text
src/plugins/robot_explain.py
```

功能：监听 ROS2 Topic：

```text
/xiaozhi_explain_text
```

当 My_nav 中的任务节点发布讲解文本后，该插件调用小智应用层的：

```python
app._send_text_tts(text)
```

使小智使用自身 TTS 能力播报讲解内容。

该设计避免了 MCP 工具长时间等待机器人导航结果造成的超时、取消或 WebSocket 连接异常问题。

---

## 7. application.py 修改说明

在 `application.py` 中导入插件：

```python
from src.plugins.robot_explain import RobotExplainPlugin
```

然后在插件注册位置加入：

```python
RobotExplainPlugin()
```

示例：

```python
self.plugins.register(
    McpPlugin(),
    IoTPlugin(),
    AudioPlugin(),
    WakeWordPlugin(),
    CalendarPlugin(),
    RobotExplainPlugin(),
    UIPlugin(mode=mode),
    ShortcutsPlugin(),
)
```

如果项目插件列表不同，只需要保证 `RobotExplainPlugin()` 被注册，并且注册发生在：

```python
await self.plugins.setup_all(self)
await self.plugins.start_all()
```

之前。

---

## 8. 推荐运行流程

### 8.1 启动 ROS2 仿真导航

在 My_nav 工作空间中启动仿真导航：

```bash
cd ~/Desktop/My_nav
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rm_nav_bringup bringup_sim.launch.py \
    world:=RMUL2026H \
    mode:=nav \
    lio:=fastlio \
    localization:=slam_toolbox \
    lio_rviz:=False \
    nav_rviz:=True
```

如使用 AMCL 定位，可改为：

```bash
localization:=amcl
```

---

### 8.2 启动 My_nav 任务执行节点

```bash
cd ~/Desktop/My_nav
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/rm_nav_bringup/scripts/waypoint_task_executor.py \
    --file ~/Desktop/My_nav/src/rm_nav_bringup/config/waypoints/RMUL2026H.yaml \
    --odom-topic /Odometry
```

如果实际里程计话题为 `/odom`，则改为：

```bash
--odom-topic /odom
```

---

### 8.3 启动 py-xiaozhi

```bash
cd ~/Desktop/py-xiaozhi-main
python main.py --mode cli
```

或启动 GUI：

```bash
python main.py
```

---

## 9. 使用示例

用户说：

```text
带我去一号展区。
```

预期流程：

```text
小智调用 self.robot_nav.go_to_waypoint
    ↓
向 /waypoint_task_cmd 发布 exhibit_1
    ↓
My_nav 执行导航
    ↓
机器人到达并停稳
    ↓
发布 /xiaozhi_explain_text
    ↓
RobotExplainPlugin 调用小智 TTS 播报讲解
```

---

## 10. 调试方法

### 10.1 测试 MCP 工具是否注册成功

```bash
cd ~/Desktop/py-xiaozhi-main

python3 - <<'PY'
import asyncio
import json

from src.mcp.mcp_server import McpServer

async def fake_send(payload):
    print(json.dumps(json.loads(payload), ensure_ascii=False, indent=2))

async def main():
    server = McpServer.get_instance()
    server.set_send_callback(fake_send)
    server.add_common_tools()

    await server.parse_message({
        "jsonrpc": "2.0",
        "method": "tools/list",
        "params": {},
        "id": 1
    })

asyncio.run(main())
PY
```

输出中应包含：

```text
self.robot_nav.go_to_waypoint
self.robot_nav.list_waypoints
```

---

### 10.2 测试导航指令发布

```bash
python3 - <<'PY'
import asyncio
from src.mcp.tools.robot_nav.tools import go_to_waypoint

async def main():
    print(await go_to_waypoint({"target": "一号展区"}))

asyncio.run(main())
PY
```

同时在 ROS 终端监听：

```bash
ros2 topic echo /waypoint_task_cmd
```

应看到对应导航点名称。

---

### 10.3 测试讲解桥接

小智启动后，在 ROS 终端发布：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/My_nav/install/setup.bash

ros2 topic pub --once /xiaozhi_explain_text std_msgs/msg/String \
"{data: '测试讲解。现在小智应该开始播报这句话。'}"
```

若小智能播报，说明 `RobotExplainPlugin` 工作正常。

---

## 11. 常见问题

### 问题 1：小智能导航，但到达后不讲解

检查：

```bash
ros2 topic echo /xiaozhi_explain_text
```

如果没有消息，说明 My_nav 的任务节点没有发布讲解文本。  
如果有消息但小智不播报，检查 `RobotExplainPlugin` 是否注册成功。

---

### 问题 2：小智调用导航工具后卡住

原因通常是 MCP 工具等待导航结果时间过长。  
正确做法：`go_to_waypoint` 只发布 `/waypoint_task_cmd` 后立即返回，不在 MCP 工具中等待机器人到达。

---

### 问题 3：小智识别了语音但不调用工具

检查角色提示词中是否明确写明：

```text
当用户说“带我去……”、“导航到……”时，必须调用 self.robot_nav.go_to_waypoint。
```

---

### 问题 4：导航点名称不匹配

检查：

```text
src/mcp/tools/robot_nav/robot_nav_config.json
```

中的 `aliases` 右侧值是否存在于 waypoint YAML 的 `points:` 中。

---

## 12. 设计原则

本功能采用“短 MCP 调用 + ROS 异步事件回传”的方式：

```text
MCP 工具只负责发送任务
ROS 负责导航、停稳检测和任务触发
py-xiaozhi 插件负责接收讲解文本并播报
```

这种设计避免了长时间阻塞 MCP 工具调用，同时保证导航任务和语音讲解解耦，便于后续扩展为多展点导览、动态讲解、问答互动等功能。
