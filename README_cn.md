# 基于地瓜RDK的空地协同融合感知与自主排爆系统

## 项目介绍

本作品设计了一套基于地瓜 RDK 的空地协同融合感知与自主排爆系统。系统采用“无人机广域侦察、地面机器人抵近复核、指控端统一调度”的协同模式，将正射影像生成、三维点云建图、目标识别、自主导航、激光打击和结果复核整合为完整闭环。

## 环境要求

- ubuntu 22.04
- ROS2 Humble
- Qt5
- RViz2 / rviz_common

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

