#include "../include/qviz_demo/qrviz.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace class1_ros_qt_demo {

qrviz::qrviz(QVBoxLayout* layout)
{
  //创建rviz panel
  render_panel = new rviz_common::RenderPanel();

  //向layout添加自己的rviz组件
  layout->addWidget(render_panel);

  //创建rviz控制对象。通过manager对象管理整个rviz
  rviz_ros_node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("qviz_demo_rviz");
  manager = new rviz_common::VisualizationManager(
      render_panel,
      rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr(rviz_ros_node),
      nullptr,
      rviz_ros_node->get_raw_node()->get_clock());
  /* 通过manager实例化一个rviz工具控制对象。通过toolManager对象可以去调用rviz工具栏中的其他工具
   * (通过类名添加，包含rviz/MoveCamera、rviz/Interact、rviz/Select、rviz/SetInitialPose、rviz/SetGoal、rviz/PublishPoint等)
   */
  toolManager = manager->getToolManager();

  //初始化render_panel，实现放大缩小等操作（注意：初始化render_panel需要在manager调用startUpdate()方法前进行，startUpdate()表示开始更新显示）
  render_panel->initialize(manager);   //为此widget设置Ogre::Camera
  //设置我们应该把所有固定数据转换到的固定坐标系
  manager->setFixedFrame("map");    //初始设置为“map”坐标系
  //panel初始颜色设置
  render_panel->setAutoFillBackground(true);
  render_panel->setStyleSheet("background-color: rgb(238, 238, 236);");

  //初始化rviz控制对象(该函数会初始化在VisualizationManager构造函数中没有初始化的对象。比如初始化工具管理器、视图管理器、选择管理器)
  manager->initialize();
  //启动计时器，创建并启动“更新”和“空闲”计时器，两者均设置为30Hz(= 1/30 = 33ms)
  manager->startUpdate();
  //移除并删除所有的displays
  manager->removeAllDisplays();
}


/***********************************************************************************************
 *                                     RViz全局设置方法定义                                       *
 ***********************************************************************************************/
/**
 * @brief: 设置固定坐标系方法
 * @param: QString FrameName ———— 固定坐标系名称
 */
void qrviz::SetFixedFrame(QString FrameName)
{
  manager->setFixedFrame(FrameName);
}

/**
 * @brief 设置rviz面板背景色方法
 * @param QColor color  ———— QColor颜色对象
 */
void qrviz::SetPanelBackgroundColor(QColor color)
{
  //Ogre::ColourValue(float red, float green, float blue, float alpha)四个参数范围要求是0.0～1.0，所以普通的0～255整形RGB值应先除以255再传参
  render_panel->setAutoFillBackground(true);
  render_panel->setStyleSheet(QString("background-color: rgba(%1, %2, %3, %4);")
                                .arg(color.red())
                                .arg(color.green())
                                .arg(color.blue())
                                .arg(color.alpha()));
}


/***********************************************************************************************
 *                                    创建Display对象方法定义                                     *
 ***********************************************************************************************/
/**
 * @brief 创建Display对象方法
 * @param const QString& class_lookup_name  ———— Display子类的“查找名称”。对于pluginlib，形式应当为"packagename/displaynameofclass"，比如"rviz_default_plugins/Image"。
 * @param const QString& name               ———— 显示在GUI上的display实例的名称，比如"Left arm camera"
 * @return Display对象指针
 */
rviz_common::Display* qrviz::CreateDisplay(const QString& class_lookup_name, const QString& name)
{
  rviz_common::Display* display = nullptr;
  //manager每调用一次creatDisplay方法就会在render panel上创建一个新的图层并返回一个Display对象
  display = manager->createDisplay(class_lookup_name, name, true); //参数中"class_lookup_name"为图层类名，比如：rviz_default_plugins/Grid、rviz_default_plugins/RobotModel、rviz_default_plugins/Map、rviz_default_plugins/LaserScan等
  Q_ASSERT(display != nullptr);
  return display;
}


/***********************************************************************************************
 *                                  修改Display对象属性方法定义                                   *
 ***********************************************************************************************/
/**
 * @brief 修改网格Display对象属性方法
 * @param rviz_common::Display* Grid ———— 网格Display对象
 * @param int CellCount       ———— 单元格数量
 * @param QColor GridColor    ———— 网格颜色
 * @param bool Enabled        ———— 是否启用Display对象
 */
void qrviz::ModifyGridAttribute(rviz_common::Display* Grid, int CellCount, QColor GridColor, bool Enabled)
{
  //设置图层是否可见
  Grid->setEnabled(Enabled);
  //通过获取到的对象，我们可以设置图层的属性，比如设置cell count
  Grid->subProp("Plane Cell Count")->setValue(CellCount);         //subProp()为设置图层属性方法
  //设置颜色
  Grid->subProp("Color")->setValue(GridColor);
}

/**
 * @brief 修改TFDisplay对象属性方法
 * @param rviz_common::Display* TF ———— TFDisplay对象
 * @param bool Enabled      ———— 是否启用Display对象
 */
void qrviz::ModifyTFAttribute(rviz_common::Display* TF, bool Enabled)
{
  //设置图层是否可见
  TF->setEnabled(Enabled);
}

/**
 * @brief 修改激光扫描Display对象属性方法
 * @param rviz_common::Display* LaserScan  ———— 激光扫描Display对象
 * @param QString LaserTopic        ———— 激光信息话题字符串
 * @param QString LaserRenderStyle  ———— 渲染模式字符串
 * @param double LaserPointSize     ———— 点的大小
 * @param bool Enabled              ———— 是否启用Display对象
 */
void qrviz::ModifyLaserScanAttribute(rviz_common::Display* LaserScan, QString LaserTopic, QString LaserRenderStyle, double LaserPointSize, bool Enabled)
{
  //设置图层是否可见
  LaserScan->setEnabled(Enabled);
  //设置图层属性
  LaserScan->subProp("Topic")->setValue(LaserTopic);
  LaserScan->subProp("Style")->setValue(LaserRenderStyle);
  if(LaserRenderStyle == "Points")
  {
    LaserScan->subProp("Size (Pixels)")->setValue(LaserPointSize);
  }
  else
  {
    LaserScan->subProp("Size (m)")->setValue(LaserPointSize);
  }
}

/**
 * @brief 修改点云2Display对象属性方法
 * @param rviz_common::Display* PointCloud2        ———— 点云2Display对象
 * @param QString PointCloud2Topic          ———— 点云2话题字符串
 * @param QString PointCloud2RenderStyle    ———— 渲染模式字符串
 * @param double PointCloud2PointSize       ———— 点云2点的大小
 * @param bool Enabled                      ———— 是否启用Display对象
 */
void qrviz::ModifyPointCloud2Attribute(rviz_common::Display* PointCloud2, QString PointCloud2Topic, QString PointCloud2RenderStyle, double PointCloud2PointSize, bool Enabled)
{
  //设置图层是否可见
  PointCloud2->setEnabled(Enabled);
  //设置图层属性
  PointCloud2->subProp("Topic")->setValue(PointCloud2Topic);
  PointCloud2->subProp("Style")->setValue(PointCloud2RenderStyle);
  if(PointCloud2RenderStyle == "Points")
  {
    PointCloud2->subProp("Size (Pixels)")->setValue(PointCloud2PointSize);
  }
  else
  {
    PointCloud2->subProp("Size (m)")->setValue(PointCloud2PointSize);
  }
}

/**
 * @brief 修改机器人模型Display对象属性方法
 * @param rviz_common::Display* RobotModel ———— 机器人模型Display对象
 * @param bool Enabled              ———— 是否启用Display对象
 */
void qrviz::ModifyRobotModelAttribute(rviz_common::Display* RobotModel, bool Enabled)
{
  //设置图层是否可见
  RobotModel->setEnabled(Enabled);
}

/**
 * @brief 修改地图Display对象属性方法
 * @param rviz_common::Display* Map  ———— 地图Display对象
 * @param QString MapTopic    ———— 地图信息话题字符串
 * @param QString ColorScheme ———— 配色方案字符串
 * @param bool Enabled        ———— 是否启用Display对象
 */
void qrviz::ModifyMapAttribute(rviz_common::Display* Map, QString MapTopic, QString ColorScheme, bool Enabled)
{
  //设置图层是否可见
  Map->setEnabled(Enabled);
  //设置图层属性
  Map->subProp("Topic")->setValue(MapTopic);
  Map->subProp("Color Scheme")->setValue(ColorScheme);
}

/**
 * @brief 修改路径Display对象属性方法
 * @param rviz_common::Display* Path ———— 路径Display对象
 * @param QString PathTopic   ———— 路径信息话题字符串
 * @param QColor PathColor    ———— 路径颜色
 * @param bool Enabled        ———— 是否启用Display对象
 */
void qrviz::ModifyPathAttribute(rviz_common::Display* Path, QString PathTopic, QColor PathColor, bool Enabled)
{
  //设置图层是否可见
  Path->setEnabled(Enabled);
  //设置图层属性
  Path->subProp("Topic")->setValue(PathTopic);
  Path->subProp("Color")->setValue(PathColor);
}

/**
 * @brief 修改带点戳Display对象属性方法
 * @param rviz_common::Display* PointStamped ———— 带点戳Display对象
 * @param QString PointStampedTopic   ———— 带点戳话题字符串
 * @param QColor PointStampedColor    ———— 带点戳颜色
 * @param bool Enabled                ———— 是否启用Display对象
 */
void qrviz::ModifyPointStampedAttribute(rviz_common::Display* PointStamped, QString PointStampedTopic, QColor PointStampedColor, bool Enabled)
{
  //设置图层是否可见
  PointStamped->setEnabled(Enabled);
  //设置图层属性
  PointStamped->subProp("Topic")->setValue(PointStampedTopic);
  PointStamped->subProp("Color")->setValue(PointStampedColor);
}

/**
 * @brief 修改标记Display对象属性方法
 * @param rviz_common::Display* Marker ———— 标记Display对象
 * @param QString MarkerTopic   ———— 标记话题字符串
 * @param bool Enabled          ———— 是否启用Display对象
 */
void qrviz::ModifyMarkerAttribute(rviz_common::Display* Marker, QString MarkerTopic, bool Enabled)
{
  //设置图层是否可见
  Marker->setEnabled(Enabled);
  //设置图层属性
  Marker->subProp("Marker Topic")->setValue(MarkerTopic);
}

/**
 * @brief 修改标记数组Display对象属性方法
 * @param rviz_common::Display* MarkerArray  ———— 标记数组Display对象
 * @param QString MarkerArrayTopic    ———— 标记数组话题字符串
 * @param bool Enabled                ———— 是否启用Display对象
 */
void qrviz::ModifyMarkerArrayAttribute(rviz_common::Display* MarkerArray, QString MarkerArrayTopic, bool Enabled)
{
  //设置图层是否可见
  MarkerArray->setEnabled(Enabled);
  //设置图层属性
  MarkerArray->subProp("Marker Topic")->setValue(MarkerArrayTopic);
}


/***********************************************************************************************
 *                                    删除Display对象方法定义                                     *
 ***********************************************************************************************/
/**
 * @brief 删除图层对象方法
 * @param rviz_common::Display* layer  ———— 图层对象
 */
void qrviz::DeleteDisplayLayer(rviz_common::Display* layer)
{
  delete layer;
}


/***********************************************************************************************
 *                                     使用RViz工具方法定义                                       *
 ***********************************************************************************************/
/**
 * @brief 使用设置导航初始点工具方法
 */
void qrviz::SetInitialPose()
{
  rviz_common::Tool* SIPtool = toolManager->addTool("rviz_default_plugins/SetInitialPose");
  //设置使用当前工具
  toolManager->setCurrentTool(SIPtool);
}

/**
 * @brief 使用设置导航目标点工具方法
 */
void qrviz::SetGoalPose()
{
  rviz_common::Tool* SGtool = toolManager->addTool("rviz_default_plugins/SetGoal");
  //获取属性容器
  rviz_common::properties::Property* SGtoolProp = SGtool->getPropertyContainer();
  //Nav2默认监听geometry_msgs/msg/PoseStamped类型的/goal_pose
  SGtoolProp->subProp("Topic")->setValue("/goal_pose");
  //设置使用当前工具
  toolManager->setCurrentTool(SGtool);
}

/**
 * @brief 使用发布点工具方法
 */
void qrviz::PublishPoint()
{
  rviz_common::Tool* PPtool = toolManager->addTool("rviz_default_plugins/PublishPoint");
  //设置使用当前工具
  toolManager->setCurrentTool(PPtool);
}

QWidget* qrviz::panelWidget() const
{
  return render_panel;
}

}  // namespace class1_ros_qt_demo
