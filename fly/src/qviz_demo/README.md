# qviz_demo

Qt/RViz2 based visualization application for ROS2 Humble.

## Environment

- Ubuntu 22.04
- ROS2 Humble
- Qt5
- RViz2 / rviz_common
- Nav2 map server

## Dependencies

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

## Build

Put this package under a ROS2 workspace `src` directory:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select qviz_demo
source install/setup.bash
```

## Run

```bash
source install/setup.bash
```

Static TF launch:

```bash
ros2 launch qviz_demo static_tf.launch.py
```
