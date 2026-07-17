# 航点生成、可视化与任务调度

## 创建航点
```text
在 RViz 里用 Publish Point 放置导航点
→ 脚本把点保存成带名字的 YAML
→ 之后用名字调用 Nav2 的 NavigateToPose action
→ 小车移动到对应导航点
```

### 1. 启动已知地图导航模式

导航点要保存在 `map` 坐标系下，所以建议在 `mode:=nav` 下使用，而不是建图模式。

例如你地图叫 `test_map`：

```bash
cd ~/Desktop/My_nav
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=test_map \
    mode:=nav \
    lio:=fastlio \
    localization:=slam_toolbox \
    lio_rviz:=False \
    nav_rviz:=True
```

确认 Nav2 action 存在：

```bash
ros2 action list | grep navigate_to_pose
```

赋予执行权限：

```bash
chmod +x src/rm_nav_bringup/scripts/waypoint_tool.py
```

### 2. 在 RViz 里放置并保存导航点

确保 RViz：

```text
Fixed Frame = map
```

然后使用 RViz 工具栏里的：

```text
Publish Point
```

然后运行记录命令，例如保存一个点叫 `home`：

```bash
cd ~/Desktop/My_nav
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/rm_nav_bringup/scripts/waypoint_tool.py record home
```

终端会等待你在 RViz 里点一个位置。你点击后，它会保存到：

```text
src/rm_nav_bringup/config/waypoints/test_map.yaml
```

再保存一个点，比如 `supply_area`：

```bash
python3 src/rm_nav_bringup/scripts/waypoint_tool.py record supply_area
```

如果想指定到达后的朝向，用 `--yaw`：

```bash
python3 src/rm_nav_bringup/scripts/waypoint_tool.py record supply_area --yaw 1.5708
```

这里 yaw 单位是弧度：

```text
0       面向 map 的 x 正方向
1.5708  左转 90°
3.1416  转 180°
-1.5708 右转 90°
```

### 3. 查看已保存的导航点

```bash
python3 src/rm_nav_bringup/scripts/waypoint_tool.py list
```

会看到类似：

```text
Waypoint file: /home/jie/Desktop/My_nav/src/rm_nav_bringup/config/waypoints/test_map.yaml
- home: frame=map, x=1.234, y=0.567
- supply_area: frame=map, x=3.210, y=-1.450
```

YAML 文件内容类似：

```yaml
points:
  home:
    frame_id: map
    position:
      x: 1.234
      y: 0.567
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
```

### 4. 指定小车去某个导航点

例如去 `home`：

```bash
python3 src/rm_nav_bringup/scripts/waypoint_tool.py goto home
```

去 `supply_area`：

```bash
python3 src/rm_nav_bringup/scripts/waypoint_tool.py goto supply_area
```

这个脚本会调用 Nav2 的：

```text
/navigate_to_pose
```

本质上和你在 RViz 里点导航目标一样，只是目标点来自你保存的 YAML。

### 5. 推荐你的使用流程

完整流程如下：

```bash
# 1. 启动已知地图导航
ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=test_map \
    mode:=nav \
    lio:=fastlio \
    localization:=slam_toolbox \
    lio_rviz:=False \
    nav_rviz:=True
```

然后保存点：

```bash
python3 src/rm_nav_bringup/scripts/waypoint_tool.py record home
python3 src/rm_nav_bringup/scripts/waypoint_tool.py record supply_area
python3 src/rm_nav_bringup/scripts/waypoint_tool.py list
```

之后指定去某个点：

```bash
python3 src/rm_nav_bringup/scripts/waypoint_tool.py goto home
```

### 6. 注意事项

导航点必须保存在 `map` 坐标系下，所以 RViz 的 `Fixed Frame` 应该设为：

```text
map
```

保存点时建议用 `Publish Point`，不要用 `Nav2 Goal`，否则小车会立即开始导航。

如果你后续换了地图，例如 `world:=room_map`，建议换一个 YAML 文件：

```bash
python3 src/rm_nav_bringup/scripts/waypoint_tool.py \
    --file ~/Desktop/My_nav/src/rm_nav_bringup/config/waypoints/room_map.yaml \
    record home
```

对应导航时也指定同一个文件：

```bash
python3 src/rm_nav_bringup/scripts/waypoint_tool.py \
    --file ~/Desktop/My_nav/src/rm_nav_bringup/config/waypoints/room_map.yaml \
    goto home
```

这样每张地图都有自己的一组命名导航点。

## rviz显示航点
```text
已有导航点 YAML
→ 在 RViz 地图上显示成圆点 + 名字
→ 用 RViz 的 Publish Point 点击某个导航点附近
→ 自动发送 NavigateToPose，让小车去这个点
```

### 1. 运行导航模式

先用你已有地图启动导航，不要用 mapping 模式：

```bash
cd ~/Desktop/My_nav
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=test_map \
    mode:=nav \
    lio:=fastlio \
    localization:=slam_toolbox \
    lio_rviz:=False \
    nav_rviz:=True
```

赋权：

```bash
chmod +x src/rm_nav_bringup/scripts/waypoint_ui_node.py
```

### 2. 启动导航点显示节点

假设你的导航点文件是之前保存的：

```text
src/rm_nav_bringup/config/waypoints/test_map.yaml
```

运行：

```bash
cd ~/Desktop/My_nav
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/rm_nav_bringup/scripts/waypoint_ui_node.py \
    --file ~/Desktop/My_nav/src/rm_nav_bringup/config/waypoints/test_map.yaml \
    --click-to-go \
    --threshold 0.6
```

这个节点会做两件事：

```text
1. 发布 /waypoint_markers，用于 RViz 显示导航点
2. 监听 /clicked_point，点击某个导航点附近后发送导航目标
```

### 3. 在 RViz 里显示导航点

在 RViz 中：

```text
Add
→ By topic
→ /waypoint_markers
→ MarkerArray
```

然后你应该能在地图上看到：

```text
绿色圆点：导航点位置
白色文字：导航点名字
```

RViz 的 Fixed Frame 建议设置为：

```text
map
```

### 4. 在 RViz 里点击导航点并让小车移动

使用 RViz 工具栏里的：

```text
Publish Point
```

然后点击某个绿色导航点附近。

如果点击位置距离某个导航点小于 `--threshold 0.6` 米，脚本会自动发送：

```text
/navigate_to_pose
```

小车就会导航到这个点。

终端会显示类似：

```text
Selected waypoint 'home' by click, distance=0.18 m.
Sending NavigateToPose goal: home, x=1.234, y=0.567
Navigation goal accepted.
```

### 5. 一套完整使用流程

#### 启动导航

```bash
ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=test_map \
    mode:=nav \
    lio:=fastlio \
    localization:=slam_toolbox \
    lio_rviz:=False \
    nav_rviz:=True
```

#### 启动导航点 UI 层

```bash
python3 src/rm_nav_bringup/scripts/waypoint_ui_node.py \
    --file ~/Desktop/My_nav/src/rm_nav_bringup/config/waypoints/test_map.yaml \
    --click-to-go \
    --threshold 0.6
```

#### RViz 中添加显示

```text
Add → /waypoint_markers → MarkerArray
```

#### RViz 中点击导航

```text
使用 Publish Point 点击绿色导航点附近
```

### 6. 这个方案的能力边界

这个方案可以实现：

```text
显示已有导航点
显示导航点名字
点击导航点附近让小车过去
修改 YAML 后自动刷新显示
```

暂时不包含：

```text
右键菜单
拖拽移动导航点
在 RViz 里重命名 / 删除导航点
```

真正的“右键菜单、拖拽点位”需要用 ROS2 的 `interactive_markers` 做交互式标记，复杂度更高。当前这个方案更适合你现在的项目阶段：**稳定、简单、能直接用已有 Nav2 导航。**

## 任务调度
```text
收到明确指令：“去 A 点”
→ 节点向 Nav2 发送 A 点目标
→ 等待 NavigateToPose action 返回成功
→ 再确认小车速度稳定为 0 附近
→ 开始执行讲解任务
```

### 一、这个功能和最终目标的关系

这个基础功能可以直接作为你最终系统的核心调度层。

后续你接入外部语音 AI 时，不需要让 AI 直接操作 Nav2。AI 只需要发布一句类似：

```text
home
```

或者：

```text
exhibition_area
```

到一个 ROS2 topic，例如：

```text
/waypoint_task_cmd
```

然后这个节点负责：

```text
查找导航点
发送导航目标
等待到达
确认停稳
执行讲解
```

后面你把“终端打印讲解消息”替换成：

```text
调用 TTS
播放音频
调用大模型生成讲解词
控制屏幕显示
```

就可以逐步演进成完整功能。

### 二、先实现一个任务调度节点

这个节点使用你之前保存的导航点 YAML，例如：

```text
src/rm_nav_bringup/config/waypoints/test_map.yaml
```

里面应该有类似：

```yaml
points:
  home:
    frame_id: map
    position:
      x: 1.23
      y: 0.45
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
```

赋权：

```bash
chmod +x src/rm_nav_bringup/scripts/waypoint_task_executor.py
```

### 三、可选：给导航点配置讲解内容

打开你的导航点文件：

```bash
nano src/rm_nav_bringup/config/waypoints/test_map.yaml
```

原来可能只有：

```yaml
points:
  home:
    frame_id: map
    position:
      x: 1.23
      y: 0.45
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
```

你可以在后面加一个 `tasks` 字段：

```yaml
tasks:
  home:
    message: |
      大家好，我们现在到达的是起始展示点。
      这里可以介绍机器人系统的整体功能。
      当前演示的是到点后自动触发讲解任务。
```

完整结构类似：

```yaml
points:
  home:
    frame_id: map
    position:
      x: 1.23
      y: 0.45
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0

tasks:
  home:
    message: |
      大家好，我们现在到达的是起始展示点。
      这里可以介绍机器人系统的整体功能。
      当前演示的是到点后自动触发讲解任务。
```

如果没有配置 `tasks`，节点也会打印默认消息。

### 四、运行方式

#### 1. 启动导航系统

例如：

```bash
cd ~/Desktop/My_nav
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=test_map \
    mode:=nav \
    lio:=fastlio \
    localization:=slam_toolbox \
    lio_rviz:=False \
    nav_rviz:=True
```

确认 Nav2 action 存在：

```bash
ros2 action list | grep navigate_to_pose
```

应该看到：

```text
/navigate_to_pose
```

---

#### 2. 启动任务调度节点

新开终端：

```bash
cd ~/Desktop/My_nav
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/rm_nav_bringup/scripts/waypoint_task_executor.py \
    --file ~/Desktop/My_nav/src/rm_nav_bringup/config/waypoints/test_map.yaml
```

#### 3. 发送“去某个导航点”的指令

比如去 `home`：

```bash
ros2 topic pub --once /waypoint_task_cmd std_msgs/msg/String "{data: 'home'}"
```

节点会执行：

```text
收到 home 指令
→ 发送 NavigateToPose
→ 小车开始导航
→ Nav2 返回成功
→ 检查 /Odometry 速度是否稳定
→ 终端打印讲解消息
```

### 五、为什么这个不会“路过就触发”？

因为这个节点不是一直监测“小车是否靠近某个点”。

它的触发条件是：

```text
必须先收到 /waypoint_task_cmd 指令
并且这个节点亲自发送了 NavigateToPose 目标
并且 Nav2 返回 STATUS_SUCCEEDED
并且小车速度稳定低于阈值一段时间
```

所以小车路过某个导航点不会触发讲解。

### 六、和你之前功能的关系

你之前的功能可以继续保留：

```text
waypoint_tool.py        负责保存导航点
waypoint_ui_node.py     负责显示导航点 / 点击导航点
waypoint_task_executor.py 负责“到点后执行任务”
```

但要注意：

如果你直接用 `waypoint_ui_node.py --click-to-go` 发送导航目标，`waypoint_task_executor.py` 不知道这次目标是它发的，所以不会触发讲解。

为了让“点击导航点”也能触发讲解，后面可以把 `waypoint_ui_node.py` 改成：

```text
点击导航点后，不直接调用 NavigateToPose
而是发布 /waypoint_task_cmd
```

也就是让所有入口统一变成：

```text
语音命令 / 图形点击 / 命令行
→ /waypoint_task_cmd
→ waypoint_task_executor.py
→ 导航 + 到点任务
```

这就是后续最推荐的架构。当前先用命令行发布 `/waypoint_task_cmd` 测试基础功能最稳。
