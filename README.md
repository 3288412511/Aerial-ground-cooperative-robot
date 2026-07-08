# D-Robotics RDK-Based Air-Ground Collaborative Fusion Perception and Autonomous Explosive Ordnance Disposal System

## Introduce

This project designs an air-ground collaborative fusion perception and autonomous explosive ordnance disposal system based on the D-Robotics RDK. The system adopts a collaborative mode of “wide-area reconnaissance by UAV, close-range verification by ground robot, and unified dispatching by the command-and-control terminal,” integrating orthophoto generation, 3D point cloud mapping, target recognition, autonomous navigation, laser strike, and result verification into a complete closed-loop workflow.

## Environment

- ubuntu 22.04
- ROS2 Humble
- Qt5
- RViz2 / rviz_common

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

## build

Put **livox_ros_driver2** **mavros_area_planner** **qviz_demo** **FAST_LIO_ROS2** package under a ROS2 workspace `src` directory:

```bash
    source /opt/ros/humble/setup.bash
    cd ~/your_workspace #your_workspace/src/pkg
    colcon build 
    source install/setup.bash
```
## 3. Directly run

### Start the Livox MID360 driver
```bash
    #Terminal 1: Start the Livox MID360 driver
    
    source ~/your_workspace/install/setup.bash
    ros2 launch livox_ros_driver2 msg_MID360_launch.py
```
- If you needs to use RVIZ Run the follow cmd instead
```bash
    ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```
### Start FAST-LIO2
```bash
    # Terminal 2: Start FAST-LIO2
    source ~/your_workspace/install/setup.bash
    ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```
### Run the host computer GUI to draw a planar (area) route.
```bash
    ros2 run mavros_area_planner area_planner_gui.py
```
- If prompted that tkinter is missing when running the GUI:
```bash
    sudo apt update
    sudo apt install python3-tk
```
After generating the coordinate points, save the file.

### Run the ROS 2 flight node.

First start MAVROS and confirm the connection.
```bash
    ros2 topic echo /mavros/state
```
Confirm that there is a local pose.
```bash
    ros2 topic echo /mavros/local_position/pose
```
Then run the waypoint flight node.
```bash
    ros2 run mavros_area_planner mavros_csv_area_mission \
    --ros-args \
    -p waypoint_csv:=/home/your_path/area_waypoints.csv \
     -p reach_threshold:=0.25 \
    -p default_speed:=0.3
```
- **reach_threshold** & **default_speed** You can change it to the parameters you want.


