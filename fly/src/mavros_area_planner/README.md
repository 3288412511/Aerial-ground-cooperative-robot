# MAVROS 面状航线规划上位机（增强版）

这个包包含：

1. `area_planner_gui.py`：上位机 GUI，用来绘制区域、生成面状航线并导出 CSV。
2. `mavros_csv_area_mission.cpp`：ROS 2 / MAVROS 飞行节点，读取 CSV 后通过 `/mavros/setpoint_position/local` 发布位置 setpoint。

坐标系使用 MAVROS 本地 ENU：`x` 为本地 East，`y` 为本地 North，`z` 向上，单位为米。

## 新增功能

### 1. 取消/删除对应点

- 点击“撤销最后一个点”：删除最后添加的区域顶点。
- 在“删除点编号 P”输入点编号，例如 `2`，点击“删除”：删除 P2。
- 在画布上右键点击某个顶点附近：删除最近的顶点。

### 2. 可确定飞行起始点

选择“设置起始参考点：点击画布”，在画布上点击希望靠近起飞入口的位置。

生成航线时，软件会比较航线首端和尾端哪个离这个参考点更近，并自动决定是否反转航点顺序，让第 0 个航点尽量靠近你指定的位置。

注意：这个版本的“起始参考点”不是强行插入一个飞行点，而是用于选择航线从哪一端开始。导出的 CSV 中 `index=0` 仍然是实际航线的第一个扫描端点。

### 3. 长方形/斜矩形区域规划

选择“矩形模式：点击两个点确定长边”。

操作流程：

1. 在画布点击第一个点，作为矩形长边起点 P0。
2. 点击第二个点，作为矩形长边终点 P1。
3. 输入 `rect_width`，作为矩形宽度。正值表示在 P0→P1 的左侧生成矩形，负值表示在右侧生成矩形。
4. 点击“用矩形参数生成区域”。
5. 软件会自动生成 P0、P1、P2、P3 四个矩形顶点，并自动计算长边角度，把它写入 `扫描角度 angle`。

这样斜着的长方形不需要你手动估计扫描角度，软件会根据 P0→P1 自动计算。

## 编译

把 `src/mavros_area_planner` 放入 ROS 2 工作空间后：

```bash
cd ~/ros2_ws
colcon build --packages-select mavros_area_planner
source install/setup.bash
```

如果运行 GUI 提示缺少 tkinter：

```bash
sudo apt update
sudo apt install python3-tk
```

## 运行 GUI

```bash
ros2 run mavros_area_planner area_planner_gui.py
```

导出 CSV 后，例如保存为：

```bash
/home/你的用户名/area_waypoints.csv
```

## 运行 MAVROS 飞行节点

先启动 MAVROS，并确认 `/mavros/state` 里 `connected: true`。

然后运行：

```bash
ros2 run mavros_area_planner mavros_csv_area_mission \
  --ros-args \
  -p waypoint_csv:=/home/你的用户名/area_waypoints.csv \
  -p reach_threshold:=0.25 \
  -p default_speed:=0.3
```

节点逻辑：

- 未进入 OFFBOARD：持续发布第一个航点，方便 PX4 允许切入 OFFBOARD。
- 进入 OFFBOARD 后：读取 CSV 航点并按顺序飞行。
- 两个 CSV 航点之间：节点根据 CSV 中的 `speed` 运行时生成虚拟 setpoint，使 `/mavros/setpoint_position/local` 平滑移动。
- 所有航点完成后：悬停在最后一个航点。

## 室内测试建议

第一次建议不装桨，只验证：

```bash
ros2 topic echo /mavros/state
ros2 topic echo /mavros/local_position/pose
ros2 topic echo /mavros/setpoint_position/local
```

带桨测试时建议先用小区域：

- 区域：0.5 m × 0.5 m
- 高度：0.5 ~ 0.8 m
- 速度：0.1 ~ 0.2 m/s
- 扫描间距：0.2 m
- 到点阈值：0.15 ~ 0.25 m
