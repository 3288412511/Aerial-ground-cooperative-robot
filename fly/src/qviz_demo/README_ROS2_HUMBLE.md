# qviz_demo ROS2 Humble 版本

该包已从 ROS1 Noetic/catkin 迁移为 ROS2 Humble/ament_cmake。

## 依赖

建议环境为 Ubuntu 22.04 + ROS2 Humble：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-rviz2 \
  ros-humble-rviz-common \
  ros-humble-rviz-default-plugins \
  ros-humble-nav2-map-server \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  qtbase5-dev qtwebengine5-dev
```

## 编译

把 `qviz_demo` 放在 ROS2 工作空间的 `src` 下：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select qviz_demo
source install/setup.bash
```

## 运行

```bash
ros2 run qviz_demo qviz_demo
```

静态 TF：

```bash
ros2 launch qviz_demo static_tf.launch.py
```

地图服务在程序中会使用 ROS2 的 `nav2_map_server` 启动，等价命令形式为：

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/path/to/map.yaml
```
