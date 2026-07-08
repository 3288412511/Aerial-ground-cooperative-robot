/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/qviz_demo/qnode.hpp"
#include <cctype>
#include <cstdlib>
#include <exception>
#include <iostream>

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace class1_ros_qt_demo {

namespace {

std::string sanitizeNodeName(const std::string& node_name)
{
  std::string sanitized;
  sanitized.reserve(node_name.size());

  for (const char ch : node_name) {
    if (std::isalnum(static_cast<unsigned char>(ch)) || ch == '_') {
      sanitized.push_back(ch);
    } else {
      sanitized.push_back('_');
    }
  }

  if (sanitized.empty() || std::isdigit(static_cast<unsigned char>(sanitized.front()))) {
    sanitized.insert(sanitized.begin(), 'q');
  }

  return sanitized;
}

bool isValidRosDomainId(const std::string& ros_domain_id)
{
  if (ros_domain_id.empty()) {
    return false;
  }

  char* end = nullptr;
  const long value = std::strtol(ros_domain_id.c_str(), &end, 10);
  return end != ros_domain_id.c_str() && *end == '\0' && value >= 0 && value <= 232;
}

}  // namespace

/*****************************************************************************
** Implementation实现
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
  init_argv(argv),
  buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
  {}

//定义静态成员变量
double QNode::clicked_point[2]{0.0, 0.0};

QNode::~QNode() {
    if(rclcpp::ok()) {
      rclcpp::shutdown();
    }
	wait();
}

/**
 * @brief 使用当前ROS 2环境变量初始化节点
 * @retval 连接成功返回true，否则返回false
 */
bool QNode::init() {
  try {
    if (!rclcpp::ok()) {
      rclcpp::init(init_argc, init_argv);
    }
    node_ = rclcpp::Node::make_shared(node_name_);
    node_->declare_parameter<std::string>("yaml_file_path", "");
  } catch (const std::exception& ex) {
    std::cerr << "Failed to initialize ROS 2 node: " << ex.what() << std::endl;
    return false;
  }

  // Add your ros communications here.
  /************
  *   发布者   *
  ************/
  //创建点击点话题发布者
  clicked_point_publisher = node_->create_publisher<geometry_msgs::msg::PointStamped>("clicked_point", 1000);
  //创建标记话题发布者
  marker_publisher = node_->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1000);
  //创建标准点话题发布者
  standard_points_publisher = node_->create_publisher<visualization_msgs::msg::Marker>("standard_point", 1000);
  //创建路径话题发布者
  global_plan_publisher = node_->create_publisher<nav_msgs::msg::Path>("global_plan", 1000);
  //创建任务路径预览话题发布者
  task_path_preview_publisher = node_->create_publisher<nav_msgs::msg::Path>("/task_path_preview", 1000);
  //创建任务路径话题发布者
  task_path_publisher = node_->create_publisher<nav_msgs::msg::Path>("/task_path", 1000);

  /************
  *   订阅者   *
  ************/
  //创建标记点位话题订阅者
  clicked_point_sub = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 1000, std::bind(&QNode::clicked_point_callback, this, std::placeholders::_1));
  //创建sick_safetyscanners/scan1话题订阅者
  laserScan1Sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/sick_safetyscanners/scan1", 1, std::bind(&QNode::laser1Callback, this, std::placeholders::_1));  //本项目未用到
  //创建sick_safetyscanners/scan2话题订阅者
  laserScan2Sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/sick_safetyscanners/scan2", 1, std::bind(&QNode::laser2Callback, this, std::placeholders::_1));  //本项目未用到
  //创建map话题订阅者
  mapSub = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 1000, std::bind(&QNode::map_callback, this, std::placeholders::_1));

  //启动多线程，调用run()方法
	start();
	return true;
}

/**
 * @brief 使用界面提供的ROS 2 Domain ID和节点名初始化
 * @param const std::string &ros_domain_id  ：ROS_DOMAIN_ID，留空则使用当前环境
 * @param const std::string &node_name      ：ROS 2节点名
 * @retval 连接成功返回true，否则返回false
 */
bool QNode::init(const std::string &ros_domain_id, const std::string &node_name) {
  if (isValidRosDomainId(ros_domain_id)) {
    setenv("ROS_DOMAIN_ID", ros_domain_id.c_str(), 1);
  } else if (!ros_domain_id.empty()) {
    std::cerr << "Ignored invalid ROS_DOMAIN_ID: " << ros_domain_id << std::endl;
  }
  node_name_ = sanitizeNodeName(node_name.empty() ? "qviz_demo" : node_name);
  return init();
}

/**
 * @brief 重写覆盖父类(QThread类)的run()方法，在其中实现耗时操作以避免堵塞UI线程
 */
void QNode::run() {
  //循环频率
  rclcpp::Rate loop_rate(20);   //20Hz

  /****************
  *   TF监听容器   *
  ****************/
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(buffer);
  log(Info,std::string("成功启动 ROS 2 节点"));

	while ( rclcpp::ok() ) {
    get_curr_robot_pose(global_pose, buffer);
    rclcpp::spin_some(node_);
		loop_rate.sleep();
	}
	std::cout << "ROS 2 shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

/**
 * @brief 获取ROS各类输出流信息并打印
 * @param const LogLevel &level  ：打印信息类别枚举对象引用
 * @param const std::string &msg ：待打印的消息字符串常量
 */
void QNode::log( const LogLevel &level, const std::string &msg) {
  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;
  if(rclcpp::ok() && node_)
  {
    switch ( level ) {
      case(Debug) : {
          RCLCPP_DEBUG_STREAM(node_->get_logger(), msg);
          logging_model_msg << "[DEBUG] [" << node_->now().seconds() << "]: " << msg;
          break;
      }
      case(Info) : {
          RCLCPP_INFO_STREAM(node_->get_logger(), msg);
          logging_model_msg << "[INFO] [" << node_->now().seconds() << "]: " << msg;
          break;
      }
      case(Warn) : {
          RCLCPP_WARN_STREAM(node_->get_logger(), msg);
          logging_model_msg << "[WARN] [" << node_->now().seconds() << "]: " << msg;
          break;
      }
      case(Error) : {
          RCLCPP_ERROR_STREAM(node_->get_logger(), msg);
          logging_model_msg << "[ERROR] [" << node_->now().seconds() << "]: " << msg;
          break;
      }
      case(Fatal) : {
          RCLCPP_FATAL_STREAM(node_->get_logger(), msg);
          logging_model_msg << "[FATAL] [" << node_->now().seconds() << "]: " << msg;
          break;
      }
    }
  }
  else
  {
    logging_model_msg << "[INFO] 请先启动 ROS 2 节点";
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}



/**
 * @brief 发布clicked_point话题槽函数
 * @param double x  ———— X坐标
 * @param double y  ———— Y坐标
 */
void QNode::slot_publish_clicked_point(double x, double y)
{
  geometry_msgs::msg::PointStamped point;
  //设置frame
  point.header.frame_id = "map";
  //设置时间戳
  point.header.stamp = node_->now();
  point.point.x = x;
  point.point.y = y;
  point.point.z = 0;    //由于显示二维地图，所以设置z为0

  clicked_point_publisher->publish(point);
}

/**
 * @brief /clicked_point话题订阅者的回调函数，将收到的消息转发至/Visualization_marker话题
 */
void QNode::clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr point_with_stamp)
{
  //为静态成员变量赋值
  clicked_point[0] = point_with_stamp->point.x;
  clicked_point[1] = point_with_stamp->point.y;
  _isClicked = true;

  if(isGenerateTask)
  {
    //设置frame(marker)
    points.header.frame_id = "map";
    //设置时间戳
    points.header.stamp = node_->now();
    //设置动作
    points.action = visualization_msgs::msg::Marker::ADD;
    //设置类型
    points.type = visualization_msgs::msg::Marker::POINTS;
    //设置点的宽和高
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    //设置点的颜色
    points.color.g = 1.0f;
    points.color.a = 1.0;

    //设置frame(path)
    path.header.frame_id = "map";
    //设置时间戳
    path.header.stamp = node_->now();
    geometry_msgs::msg::PoseStamped pose_with_stamp;

    if(isUsingStandardPoint){
      StandardPoint nearestPoint = linear_nearest_neighbor_search(point_with_stamp->point.x, point_with_stamp->point.y);   //线性搜索求最近邻
      geometry_msgs::msg::Point temp;
      temp.x = nearestPoint.x();
      temp.y = nearestPoint.y();
      points.points.push_back(temp);                    //最近邻标准点入栈
      pose_with_stamp.pose.position = temp;             //最近邻标准点作为path的PoseStamped的position
    }
    else{
      points.points.push_back(point_with_stamp->point);  //clicked_point点入栈
      pose_with_stamp.pose.position = point_with_stamp->point;   //clicked_point点作为path的PoseStamped的position
    }
    path.poses.push_back(pose_with_stamp);

    emit pointClicked();
    marker_publisher->publish(points);     //发布marker
    global_plan_publisher->publish(path);  //发布path
  }
}

/**
 * @brief 发布"global_plan"话题槽函数
 */
nav_msgs::msg::Path QNode::slot_publish_global_plan()
{
  global_plan_publisher->publish(path);
  //返回path
  return path;
}



/**
 * @brief /map话题订阅者的回调函数
 */
void QNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr occupancyGrid)
{
  (void)occupancyGrid;
  emit mapOpened();
}

/**
 * @brief sick_safetyscanners/scan1话题订阅者的回调函数
 * @attention 在本项目中，该函数并未使用到
 */
void QNode::laser1Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  laserPoint1.header= msg->header;
  laserPoint1.angle_min = msg->angle_min;
  laserPoint1.angle_max = msg->angle_max;
  laserPoint1.angle_increment = msg->angle_increment;
  laserPoint1.time_increment = msg->time_increment;
  laserPoint1.scan_time = msg->scan_time;
  laserPoint1.ranges = msg->ranges;
}

/**
 * @brief sick_safetyscanners/scan2话题订阅者的回调函数
 * @attention 在本项目中，该函数并未使用到
 */
void QNode::laser2Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  laserPoint2.header= msg->header;
  laserPoint2.angle_min = msg->angle_min;
  laserPoint2.angle_max = msg->angle_max;
  laserPoint2.angle_increment = msg->angle_increment;
  laserPoint2.time_increment = msg->time_increment;
  laserPoint2.scan_time = msg->scan_time;
  laserPoint2.ranges = msg->ranges;
}

/**
 * @brief 获取当前机器人位置方法
 * @param geometry_msgs::msg::PoseStamped& global_pose ———— 全局位置变量
 * @param tf2_ros::Buffer& tf_                    ———— TF容器
 */
bool QNode::get_curr_robot_pose(geometry_msgs::msg::PoseStamped& global_pose, tf2_ros::Buffer& tf_)
{
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  geometry_msgs::msg::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
  robot_pose.header.frame_id = "base_link";
  robot_pose.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time current_time = node_->now();
  //获取机器人全局位姿
  try
  {
      // 尽可能使用当前时间（确保它不是将来的时间）
      if (tf_.canTransform("map", "base_link", current_time, tf2::durationFromSec(0.1)))
      {
          geometry_msgs::msg::TransformStamped transform = tf_.lookupTransform("map", "base_link", current_time);
          tf2::doTransform(robot_pose, global_pose, transform);
      }
      // 否则使用最近的时间
      else
      {
          tf_.transform(robot_pose, global_pose, "map");
      }
  }
  catch (tf2::LookupException& ex)
  {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "查找机器人姿态时出现错误(无可用转换): %s", ex.what());
      return false;
  }
  catch (tf2::ConnectivityException& ex)
  {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "查找机器人姿态时出现连接错误: %s", ex.what());
      return false;
  }
  catch (tf2::ExtrapolationException& ex)
  {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "查找机器人姿态时出现外推错误: %s", ex.what());
      return false;
  }
  // 检查 global_pose 超时
  if ((current_time - rclcpp::Time(global_pose.header.stamp)).seconds() > transform_tolerance)
  {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                      "Costmap2DROS 转换超时. 当前时间: %.4f, global_pose 时间戳: %.4f, 允许偏差: %.4f",
                      current_time.seconds(), rclcpp::Time(global_pose.header.stamp).seconds(), transform_tolerance);
      return false;
  }
  //发出实时位置信号
  emit position(global_pose.pose.position.x, global_pose.pose.position.y, global_pose.pose.position.z);

  return true;
}

/**
 * @brief 位姿数据转换(将double类型数据乘1000后转换为int16_t类型以发送给PLC使用)
 * @param 全局姿态global_pose
 * @param x坐标x
 * @param y坐标y
 * @param 偏航角curYaw
 * @attention 在本项目中，该函数并未使用到
 */
void QNode::pose_data_converse(geometry_msgs::msg::PoseStamped& global_pose, int16_t& x, int16_t& y, int16_t& curYaw)
{
  float xPose = global_pose.pose.position.x * 1000;
  float yPose = global_pose.pose.position.y * 1000;
  x = (int16_t)std::floor(xPose);
  y = (int16_t)std::floor(yPose);
  double yaw = tf2::getYaw(global_pose.pose.orientation);
  if (yaw < 0)
  {
      yaw += 2*M_PI;
  }
  curYaw = (int16_t)std::floor(yaw *1000);
}

/**
 * @brief 机器人避障方法(通过sick_safetyscanners/scan1话题数据判断避障)
 * @attention 在本项目中，该函数并未使用到
 */
std::vector<int16_t> QNode::scanObstacleDetectionToshortestDistanceAroundAGV()
{
    std::vector<int16_t> front_back_left_right;
    // 前面的激光雷达
    int length1 = laserPoint1.ranges.size();
    int length2 = laserPoint2.ranges.size();
    if ((length1 == 0) || (length2 == 0))
    {
        return {10000, 10000, 10000, 10000};
    }

    // std::cout<<"length1: "<<length1<<std::endl;
    std::vector<float> laser1_x;
    std::vector<float> laser1_yLeft;
    std::vector<float> laser1_yRight;
    std::vector<float> front_left_right_shortest_distance;

    for (int i = 0; i < length1; i++)
    {
        float angle = laserPoint1.angle_increment * i + laserPoint1.angle_min;
        float x = laserPoint1.ranges[i] * std::cos(angle);
        float y = laserPoint1.ranges[i] * std::sin(angle);
        if ((x > 0.15) && (!isinf(x)))
        {
            if (fabs(y) > 0.4)
            {
                float front_side_distance = hypotf32(x-0.15, fabs(y)-0.4);
                laser1_x.push_back(front_side_distance);
            }
            if (fabs(y) <= 0.4)
            {
                laser1_x.push_back(x-0.15);
            }
        }
        if ((x <= 0.15) && (y <= -0.4))
        {
            laser1_yLeft.push_back(y);
        }
        if ((x <= 0.15) && (y > 0.4))
        {
            laser1_yRight.push_back(y);
        }
    }

    if (laser1_x.size() == 0)
    {
        front_left_right_shortest_distance.push_back(10.0);
    }
    else if (laser1_x.size() > 0)
    {
        std::sort(laser1_x.begin(), laser1_x.end());
        front_left_right_shortest_distance.push_back(fabs(laser1_x.at(0)));
    }

    if (laser1_yLeft.size() == 0)
    {
        front_left_right_shortest_distance.push_back(10.0);
    }
    else if(laser1_yLeft.size() > 0)
    {
        std::sort(laser1_yLeft.rbegin(), laser1_yLeft.rend());
        front_left_right_shortest_distance.push_back(fabs(laser1_yLeft.at(0)));
    }

    if (laser1_yRight.size() == 0)
    {
        front_left_right_shortest_distance.push_back(10.0);
    }
    else if(laser1_yRight.size() > 0)
    {
        std::sort(laser1_yRight.begin(), laser1_yRight.end());
        front_left_right_shortest_distance.push_back(fabs(laser1_yRight.at(0)));
    }


    // 后面的激光雷达

    // std::cout<<"length2: "<<length2<<std::endl;
    std::vector<float> laser2_x;
    std::vector<float> laser2_yLeft;
    std::vector<float> laser2_yRight;
    std::vector<float> back_left_right_shortest_distance;

    for (int i = 0; i < length2; i++)
    {
        float angle = laserPoint2.angle_increment * i + laserPoint2.angle_min;
        float x = laserPoint2.ranges[i] * std::cos(angle);
        float y = laserPoint2.ranges[i] * std::sin(angle);
        if ((x > 0.15) && (!isinf(x)))
        {
            if (fabs(y) > 0.4)
            {
                float back_side_distance = hypotf32(x-0.15, fabs(y)-0.4);
                laser2_x.push_back(back_side_distance);
            }
            if (fabs(y) <= 0.4)
            {
                laser2_x.push_back(x-0.15);
            }
        }
        if ((x <= 0.15) && (y >= 0.4))
        {
            laser2_yLeft.push_back(y);
        }
        if ((x <= 0.15) && (y < -0.4))
        {
            laser2_yRight.push_back(y);
        }
    }

    if (laser2_x.size() == 0)
    {
        back_left_right_shortest_distance.push_back(10.0);
    }
    else if (laser2_x.size() > 0)
    {
        std::sort(laser2_x.begin(), laser2_x.end());
        back_left_right_shortest_distance.push_back(fabs(laser2_x.at(0)));
    }

    if (laser2_yLeft.size() == 0)
    {
        back_left_right_shortest_distance.push_back(10.0);
    }
    else if(laser2_yLeft.size() > 0)
    {
        std::sort(laser2_yLeft.begin(), laser2_yLeft.end());
        back_left_right_shortest_distance.push_back(fabs(laser2_yLeft.at(0)));
    }

    if (laser2_yRight.size() == 0)
    {
        back_left_right_shortest_distance.push_back(10.0);
    }
    else if(laser2_yRight.size() > 0)
    {
        std::sort(laser2_yRight.rbegin(), laser2_yRight.rend());
        back_left_right_shortest_distance.push_back(fabs(laser2_yRight.at(0)));
    }

    // std::vector<int16_t> front_back_left_right;
    int16_t front = (int16_t)(front_left_right_shortest_distance.at(0) * 1000);
    int16_t back = (int16_t)(back_left_right_shortest_distance.at(0) * 1000);
    front_back_left_right.push_back(front);
    front_back_left_right.push_back(back);

    // 左边最小距离
    if (front_left_right_shortest_distance.at(2) > back_left_right_shortest_distance.at(2))
    {
        int16_t back_right_modefy = (int16_t)((back_left_right_shortest_distance.at(2) - 0.4) * 1000);
        front_back_left_right.push_back(back_right_modefy);
    }
    if(front_left_right_shortest_distance.at(2) <= back_left_right_shortest_distance.at(2))
    {
        int16_t front_right_modefy = (int16_t)((front_left_right_shortest_distance.at(2) - 0.4) * 1000);
        front_back_left_right.push_back(front_right_modefy);
    }
    // 右边最小距离
    if (front_left_right_shortest_distance.at(1) > back_left_right_shortest_distance.at(1))
    {
        int16_t back_left_modefy = (int16_t)((back_left_right_shortest_distance.at(1) - 0.4) * 1000);
        front_back_left_right.push_back(back_left_modefy);
    }
    if (front_left_right_shortest_distance.at(1) <= back_left_right_shortest_distance.at(1))
    {
        int16_t front_left_modefy = (int16_t)((front_left_right_shortest_distance.at(1) - 0.4) * 1000);
        front_back_left_right.push_back(front_left_modefy);
    }

    return front_back_left_right;
}

/**
 * @brief 将地图(yaml)文件的路径信息传递给ROS参数服务器(参数名为"/yaml_file_path")
 * @attention 多次调用该函数则新的值会覆盖旧的值
 * @param filePath 文件路径字符串，比如"/home/user/map/map.yaml"
 */
void QNode::set_yaml_file_path(const QString& filePath)
{
  yaml_file_path_ = filePath.toStdString();
  if (node_) {
    node_->set_parameter(rclcpp::Parameter("yaml_file_path", yaml_file_path_));
  }
}

/**
 * @brief 获取ROS参数服务器(参数名为"/yaml_file_path")的地图(yaml)文件的路径信息
 * @attention 如果从未设置过该参数的值，则会返回一个空字符串
 * @retval 地图(yaml)文件路径字符串
 */
QString QNode::get_yaml_file_path()
{
  if (node_ && node_->has_parameter("yaml_file_path")) {
    yaml_file_path_ = node_->get_parameter("yaml_file_path").as_string();
  }
  return QString::fromStdString(yaml_file_path_);
}

/**
 * @brief 显示所有标准点方法(通过标记数组MarkerArray图层显示)
 * @param 是否显示(visible)
 */
void QNode::show_all_standard_points(bool visible)
{
  //创建Marker消息类型对象
  visualization_msgs::msg::Marker marker;
  //创建Point消息类型对象
  geometry_msgs::msg::Point point;
  //设置frame
  marker.header.frame_id = "map";
  //设置时间戳
  marker.header.stamp = node_->now();
  //设置动作
  marker.action = visualization_msgs::msg::Marker::ADD;
  //设置类型
  marker.type = visualization_msgs::msg::Marker::POINTS;
  //设置点的宽和高
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  //设置点的颜色
  marker.color.r = 1.0f;
  marker.color.a = 1.0;
  if(visible){
    QVector<StandardPoint>::Iterator p;
    for(p = standardPoints.begin(); p < standardPoints.end(); p++){
      point.x = p->x();
      point.y = p->y();
      point.z = 0.0;
      //点入栈
      marker.points.push_back(point);
    }
  }
  else{
    marker.points.clear();
  }
  //发布marker
  standard_points_publisher->publish(marker);
}

/**
 * @brief 使用线性搜索查找二维平面内距离目标坐标最近的标准点
 * @param 目标x坐标x
 * @param 目标y坐标y
 * @return 标准点(StandardPoint)类对象
 */
StandardPoint QNode::linear_nearest_neighbor_search(double x, double y)
{
  QVector<StandardPoint>::Iterator p;
  StandardPoint nearestPoint = standardPoints[0];           //使用StandardPoint对象保存当前最近的点
  double nearestDistance = pow((standardPoints[0].x() - x), 2) + pow((standardPoints[0].y() - y), 2); //使用一个double保存最近距离(二维欧式距离的平方)
  for(p = standardPoints.begin() + 1; p < standardPoints.end(); p++){
    //nearestPoint = ((pow((p->x() - x), 2) + pow((p->y() - y), 2)) < nearestDistance) ? *p : nearestPoint; //使用三目运算符判断最近邻
    double newDistance = pow((p->x() - x), 2) + pow((p->y() - y), 2);
    if(newDistance < nearestDistance){
      nearestPoint = *p;                //更新最近邻和最近距离
      nearestDistance = newDistance;
    }
  }

  return nearestPoint;
}

/**
 * @brief 如果points数组不为空的话，撤回一个点位
 */
void QNode::point_revocation()
{
  if(points.points.size() > 0){
    points.points.pop_back();
    path.poses.pop_back();
    emit pointRevoked();
    marker_publisher->publish(points);     //发布marker
    global_plan_publisher->publish(path);  //发布path
  }
}

/**
 * @brief 根据points成员变量的起点查找是否为标准点，如果是返回标准点名称，不是返回空字符串
 */
QString QNode::start_point_name_search()
{
  if(points.points.size() > 0){
    double coordinate_x = points.points[0].x;
    double coordinate_y = points.points[0].y;
    for(QVector<StandardPoint>::Iterator p = standardPoints.begin(); p < standardPoints.end(); p++){
      if(coordinate_x == (*p).x() && coordinate_y == (*p).y())
        return (*p).pointName();
    }
  }
  return "";
}

/**
 * @brief 根据points成员变量的终点查找是否为标准点，如果是返回标准点名称，不是返回空字符串
 */
QString QNode::end_point_name_search()
{
  if(points.points.size() > 0){
    double coordinate_x = points.points.back().x;
    double coordinate_y = points.points.back().y;
    for(QVector<StandardPoint>::Iterator p = standardPoints.begin(); p < standardPoints.end(); p++){
      if(coordinate_x == (*p).x() && coordinate_y == (*p).y())
        return (*p).pointName();
    }
  }
  return "";
}

}  // namespace class1_ros_qt_demo
