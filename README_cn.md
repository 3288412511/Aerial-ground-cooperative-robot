# 基于地瓜 RDK 的空地协同融合感知与自主排爆系统

## 项目简介

本项目设计了一套基于地瓜 RDK 的空地协同融合感知与自主排爆系统。系统采用“无人机广域侦察、地面机器人抵近复核、指控端统一调度”的协同工作模式，将正射影像生成、三维点云建图、目标识别、自主导航、激光打击和结果复核等功能整合为完整闭环流程。

## 运行环境

- Ubuntu 22.04
- ROS 2 Humble
- Qt5
- RViz2 / rviz_common

## 依赖安装

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

## 编译方法

将 **livox_ros_driver2**、**mavros_area_planner**、**qviz_demo**、**FAST_LIO_ROS2** 等功能包放入 ROS 2 工作空间的 `src` 目录下，例如：

```bash
source /opt/ros/humble/setup.bash
cd ~/your_workspace
colcon build
source install/setup.bash
```

> 其中 `~/your_workspace` 为你的 ROS 2 工作空间路径。

## 运行方法

### 1. 启动 Livox MID360 驱动

打开第一个终端，执行以下命令启动 Livox MID360 雷达驱动：

```bash
source ~/your_workspace/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

如果需要同时使用 RViz 进行可视化显示，可以改用以下命令启动：

```bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

### 2. 启动 FAST-LIO2 建图节点

打开第二个终端，启动 FAST-LIO2 建图程序：

```bash
source ~/your_workspace/install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

### 3. 运行上位机航线规划界面

运行上位机 GUI，用于绘制面状航线并生成航点坐标：

```bash
ros2 run mavros_area_planner area_planner_gui.py
```

如果运行 GUI 时提示缺少 `tkinter`，可以执行以下命令安装：

```bash
sudo apt update
sudo apt install python3-tk
```

航线坐标点生成完成后，将其保存为 CSV 文件，用于后续 ROS 2 飞行节点读取。

### 4. 运行 ROS 2 航点飞行节点

在运行航点任务前，先启动 MAVROS，并确认飞控连接状态正常：

```bash
ros2 topic echo /mavros/state
```

然后确认 MAVROS 已经能够输出本地位姿信息：

```bash
ros2 topic echo /mavros/local_position/pose
```

确认状态正常后，运行航点飞行节点：

```bash
ros2 run mavros_area_planner mavros_csv_area_mission \
    --ros-args \
    -p waypoint_csv:=/home/your_path/area_waypoints.csv \
    -p reach_threshold:=0.25 \
    -p default_speed:=0.3
```

参数说明：

- `waypoint_csv`：生成的航点 CSV 文件路径。
- `reach_threshold`：航点到达判定阈值，单位为米。
- `default_speed`：默认飞行速度，可根据实际需求进行调整。

## 说明

在实际运行前，请确保：

1. Livox MID360 雷达能够正常发布点云数据；
2. FAST-LIO2 能够正常完成点云建图；
3. MAVROS 已与 PX4 飞控建立连接；
4. `/mavros/local_position/pose` 能够正常输出本地位姿；
5. 上位机生成的 CSV 航点文件路径正确。

完成以上检查后，即可通过 ROS 2 航点飞行节点读取面状航线坐标，并执行自动飞行任务。

## 正射影像图拼接
### 安装并运行好docker后，可以使用终端拉取ODM容器

```bash
docker pull opendronemap/odm
```

### 将图片放入名为“images”（例如/home/youruser/datasets/project/images）的文件夹中，运行 ODM：
docker run -ti --rm -v /home/youruser/datasets:/datasets opendronemap/odm --project-path /datasets project

### 可以通过在命令中附加参数来传递：
docker run -ti --rm -v /datasets:/datasets opendronemap/odm --project-path /datasets project [--additional --parameters --here]

### 例如，要生成DSM并提高正射照片分辨率：
docker run -ti --rm -v /datasets:/datasets opendronemap/odm --project-path /datasets project --dsm --orthophoto-resolution 2

### 拼图结果按以下方式整理：
|-- images/
    |-- img-1234.jpg
    |-- ...
|-- opensfm/
    |-- see mapillary/opensfm repository for more info
|-- odm_meshing/
    |-- odm_mesh.ply                   
|-- odm_texturing/
    |-- odm_textured_model.obj         
    |-- odm_textured_model_geo.obj     
|-- odm_georeferencing/
    |-- odm_georeferenced_model.laz    
|-- odm_orthophoto/
    |-- odm_orthophoto.tif          

### 使用以下软件打开ODM生成的文件：
 * .tif (GeoTIFF): [QGIS](http://www.qgis.org/)
 * .laz (Compressed LAS): [CloudCompare](https://www.cloudcompare.org/)
 * .obj (Wavefront OBJ), .ply (Stanford Triangle Format): [MeshLab](http://www.meshlab.net/)

