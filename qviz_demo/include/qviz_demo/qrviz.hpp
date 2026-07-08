#ifndef QRVIZ_HPP
#define QRVIZ_HPP

#include <QObject>
#include <QVBoxLayout>
#include <OgreColourValue.h>

//包含ROS2及RViz2组件库头文件
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace class1_ros_qt_demo {

/*****************************************************************************
** Class
*****************************************************************************/
class qrviz
{
public:
  //构造函数通过参数传入界面对象指针从而通过该指针添加widget
  qrviz(QVBoxLayout* layout);


  /******************************************
  ** RViz全局设置方法
  *******************************************/
  void SetFixedFrame(QString FrameName);
  void SetPanelBackgroundColor(QColor color);


  /******************************************
  ** 创建Display对象方法
  *******************************************/
  rviz_common::Display* CreateDisplay(const QString& class_lookup_name, const QString& name);


  /******************************************
  ** 修改Display对象属性方法
  *******************************************/
  void ModifyGridAttribute(rviz_common::Display* Grid, int CellCount, QColor GridColor, bool Enabled);
  void ModifyTFAttribute(rviz_common::Display* TF, bool Enabled);
  void ModifyLaserScanAttribute(rviz_common::Display* LaserScan, QString LaserTopic, QString LaserRenderStyle, double LaserPointSize, bool Enabled);
  void ModifyPointCloud2Attribute(rviz_common::Display* PointCloud2, QString PointCloud2Topic, QString PointCloud2RenderStyle, double PointCloud2PointSize, bool Enabled);
  void ModifyRobotModelAttribute(rviz_common::Display* RobotModel, bool Enabled);
  void ModifyMapAttribute(rviz_common::Display* Map, QString MapTopic, QString ColorScheme, bool Enabled);
  void ModifyPathAttribute(rviz_common::Display* Path, QString PathTopic, QColor PathColor, bool Enabled);
  void ModifyPointStampedAttribute(rviz_common::Display* PointStamped, QString PointStampedTopic, QColor PointStampedColor, bool Enabled);
  void ModifyMarkerAttribute(rviz_common::Display* Marker, QString MarkerTopic, bool Enabled);
  void ModifyMarkerArrayAttribute(rviz_common::Display* MarkerArray, QString MarkerArrayTopic, bool Enabled);


  /******************************************
  ** 删除Display对象方法
  *******************************************/
  void DeleteDisplayLayer(rviz_common::Display* layer);


  /******************************************
  ** 使用RViz工具方法
  *******************************************/
  void SetInitialPose();
  void SetGoalPose();
  void PublishPoint();
  QWidget* panelWidget() const;
private:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> rviz_ros_node;
  rviz_common::RenderPanel* render_panel;      //声明3D视图面板微件
  rviz_common::VisualizationManager* manager;  //声明RViz中央管理器类对象
  rviz_common::ToolManager* toolManager;       //声明RViz工具管理器类对象
};

}  // namespace class1_ros_qt_demo

#endif // QRVIZ_HPP
