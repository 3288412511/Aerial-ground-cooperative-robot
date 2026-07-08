/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include "../include/qviz_demo/main_window.hpp"
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLineEdit>
#include <QCheckBox>
#include <QDir>
#include <QDirIterator>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QFrame>
#include <QHBoxLayout>
#include <QColorDialog>
#include <QFileInfo>
#include <QImage>
#include <QIntValidator>
#include <QPixmap>
#include <QUrl>
#include <QVBoxLayout>


/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace class1_ros_qt_demo {

using namespace Qt;

namespace {

const QString kDemoVideoRoot = QString::fromUtf8(u8"/mnt/hgfs/建图与导航/sucai");

QString findOrthoDetectionImage(const QString& rootPath)
{
  const QStringList filters = {"*.png", "*.jpg", "*.jpeg", "*.bmp", "*.tif", "*.tiff"};
  QDirIterator iterator(rootPath, filters, QDir::Files, QDirIterator::Subdirectories);
  QString fallbackPath;

  while (iterator.hasNext()) {
    const QString filePath = iterator.next();
    const QString fileName = QFileInfo(filePath).fileName();
    if (fallbackPath.isEmpty()) {
      fallbackPath = filePath;
    }
    if (fileName.contains(QString::fromUtf8(u8"正射影像图识别后")) ||
        (fileName.contains(QString::fromUtf8(u8"正射影像")) &&
         fileName.contains(QString::fromUtf8(u8"识别后")))) {
      return filePath;
    }
  }

  return fallbackPath;
}

QImage matToImage(const cv::Mat& frame)
{
  if (frame.empty()) {
    return QImage();
  }

  if (frame.type() == CV_8UC3) {
    cv::Mat rgbFrame;
    cv::cvtColor(frame, rgbFrame, cv::COLOR_BGR2RGB);
    return QImage(
      rgbFrame.data,
      rgbFrame.cols,
      rgbFrame.rows,
      static_cast<int>(rgbFrame.step),
      QImage::Format_RGB888
    ).copy();
  }

  if (frame.type() == CV_8UC1) {
    return QImage(
      frame.data,
      frame.cols,
      frame.rows,
      static_cast<int>(frame.step),
      QImage::Format_Grayscale8
    ).copy();
  }

  if (frame.type() == CV_8UC4) {
    return QImage(
      frame.data,
      frame.cols,
      frame.rows,
      static_cast<int>(frame.step),
      QImage::Format_ARGB32
    ).copy();
  }

  return QImage();
}

void updateVideoLabel(QLabel* label, const cv::Mat& frame, Qt::AspectRatioMode aspectRatioMode = Qt::KeepAspectRatio)
{
  if (label == nullptr) {
    return;
  }

  const QImage image = matToImage(frame);
  if (image.isNull()) {
    return;
  }

  const QPixmap pixmap = QPixmap::fromImage(image).scaled(
    label->size(),
    aspectRatioMode,
    Qt::SmoothTransformation
  );
  label->setPixmap(pixmap);
}

}  // namespace

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
  , yamlFilePath("/home")
  , gt_ptr(nullptr)
  , sp_ptr(nullptr)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  ui.line_edit_domain_id->setValidator(new QIntValidator(0, 232, this));
  ui.line_edit_domain_id->setPlaceholderText("0");
  ui.line_edit_node_name->setPlaceholderText("qviz_demo");
  ui.line_edit_topic->setReadOnly(true);

  ReadSettings();
  setWindowIcon(QIcon("://icon/robot.svg"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  registerZoomablePreview(ui.labelStitchingPreview);
  registerZoomablePreview(ui.labelUxodDetectionPreview);
  registerZoomablePreview(ui.labelStrikeMissionMosaic);

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));


  /*********************
  ** 连接实时位置信号与MainWindow更新位置显示槽函数
  **********************/
  connect(&qnode, &QNode::position, this, &MainWindow::slot_update_pos);
  connect(&qnode, &QNode::pointClicked, this, [=](){
    modeTypes.append(ui.comboBox_ModeType->currentText().toInt());
  });
  connect(&qnode, &QNode::pointRevoked, this, [=](){
    modeTypes.pop_back();
  });

  /*********************
  ** File Menu  菜单栏文件选项
  **********************/
  ui.action_OpenMap->setIcon(QIcon("://mapManager/OpenMap.svg"));

  /*********************
  ** mapServerProcess map_server进程相关设置
  **********************/
  connect(&qnode, &QNode::mapOpened, this, &MainWindow::slot_map_opened);

  connect(mapServerProcess, &QProcess::started, this, [=](){
    QString msg = QString("正加载地图 %1").arg(mapFileName);
    qnode.log(QNode::Info, msg.toStdString());
    QSystemTrayIcon trayIcon;
    trayIcon.setIcon(QIcon("://mapManager/OpenMap.svg"));
    trayIcon.show();
    trayIcon.showMessage("提示", msg, QSystemTrayIcon::Information, 2000);
  });

  connect(mapServerProcess, &QProcess::errorOccurred, this, &MainWindow::on_QProcessErrorOccurred);
  //读取标准输出
  connect(mapServerProcess, &QProcess::readyReadStandardOutput, this, [=](){
    qnode.log(QNode::Info, std::string(mapServerProcess->readAllStandardOutput().constData()));
  });
  //读取标准错误
  connect(mapServerProcess, &QProcess::readyReadStandardError, this, [=](){
    qnode.log(QNode::Info, std::string(mapServerProcess->readAllStandardError().constData()));
  });
  //处理进程结束(finished信号存在重载，需要使用函数指针)
  connect(mapServerProcess, QOverload<int,QProcess::ExitStatus>::of(&QProcess::finished), this, [=](int exitCode, QProcess::ExitStatus exitStatus){
    if (exitStatus == QProcess::NormalExit)
    {
      qDebug() << "The process exited normally with code" << exitCode;
      if(exitCode != 0)
      {
        QString msg = QString("打开地图 %1 失败").arg(mapFileName);
        qnode.log(QNode::Info, msg.toStdString());
        QSystemTrayIcon trayIcon;
        trayIcon.setIcon(QIcon("://mapManager/OpenMap.svg"));
        trayIcon.show();
        trayIcon.showMessage("提示", msg, QSystemTrayIcon::Critical, 2000);
      }
    }
    else
    {
      qDebug() << "The process crashed with code" << exitCode;
    }
  });

  connect(ui.action_OpenMap, &QAction::triggered, this, [=](){
    yamlFilePath = QFileDialog::getOpenFileName(this, "打开地图", yamlFilePath, "(*.yaml)");
    if(!yamlFilePath.isEmpty())
    {
      QFileInfo mapFileInfo(yamlFilePath);              //获取yaml文件信息
      mapFileName = mapFileInfo.completeBaseName();     //根据地图yaml文件路径获得地图名称
      //QProcessOpenMap();
      GnomeOpenMap();
    }
  });

  /*********************
  ** View Menu  菜单栏视图选项
  **********************/
  ui.action_Displays->setIcon(QIcon("://rViz/Displays.svg"));
  connect(ui.action_Displays, &QAction::triggered, this, [=](){
    if(ui.stackedWidget->currentWidget() == ui.Map)
    {
      ui.docktreeWidget->setHidden(false);
      ui.docktreeWidget->setFocus();
    }
  });

  ui.action_TaskManager->setIcon(QIcon("://taskManagement/task_manager.svg"));
  connect(ui.action_TaskManager, &QAction::triggered, this, [=](){
    if(ui.stackedWidget->currentWidget() == ui.Map)
    {
      ui.dockTaskManagerWidget->setHidden(false);
      ui.dockTaskManagerWidget->setFocus();
    }
  });

  /*********************
  ** External Program Menu  菜单栏外部程序选项
  **********************/
  ui.action_RViz->setIcon(QIcon("://rViz/package.png"));
  connect(ui.action_RViz, &QAction::triggered, this, [=](){
    system("gnome-terminal -- bash -c 'ros2 run rviz2 rviz2'&");
  });

  ui.action_ShellScript->setIcon(QIcon("://icon/SHELL.svg"));
  connect(ui.action_ShellScript, &QAction::triggered, this, [=](){
    shellSelect = new ShellSelect();
    shellSelect->setWindowTitle("脚本窗口");
    shellSelect->exec();
  });

  /*********************
  ** Initialization
  **********************/
  ui.stackedWidget->setCurrentIndex(0);
  ui.rViztoolBar->setHidden(true);
  ui.rViztoolBar->setEnabled(false);

  ui.docktreeWidget->setHidden(true);
  ui.dockWidgetContents->setEnabled(false);
  ui.docktreeWidget->adjustSize();

  ui.dockTaskManagerWidget->setHidden(true);
  ui.dockWidgetContents_2->setEnabled(false);
  ui.dockTaskManagerWidget->adjustSize();

  //使用Qt资源为 “: + 前缀名 + 文件名”
  QAction* ActionMap = ui.menu->addAction(QIcon("://rViz/Map.png"), "地图");
  connect(ActionMap, &QAction::triggered, this, &MainWindow::showMainMapPage);
  ui.menu->addSeparator();

  QAction* actionDualMap = ui.menu->addAction(QIcon("://rViz/Views.svg"), "双建图监视");
  connect(actionDualMap, &QAction::triggered, this, &MainWindow::showDualMapPage);
  ui.menu->addSeparator();

  QAction* actionAerialPhoto = ui.menu->addAction(QIcon("://rViz/Camera.png"), "航测拍照");
  connect(actionAerialPhoto, &QAction::triggered, this, [=](){
    ui.stackedWidget->setCurrentWidget(ui.AerialPhotoPage);
    ui.docktreeWidget->setHidden(true);
    ui.dockTaskManagerWidget->setHidden(true);
    ui.rViztoolBar->setHidden(true);
  });
  ui.menu->addSeparator();

  QAction* actionImageAnalysis = ui.menu->addAction(QIcon("://rViz/Image.png"), "图像处理");
  connect(actionImageAnalysis, &QAction::triggered, this, [=](){
    ui.stackedWidget->setCurrentWidget(ui.ImageAnalysisPage);
    ui.docktreeWidget->setHidden(true);
    ui.dockTaskManagerWidget->setHidden(true);
    ui.rViztoolBar->setHidden(true);
  });
  ui.menu->addSeparator();

  QAction* actionStrikeMission = ui.menu->addAction(QIcon("://taskManagement/AutoDrive.svg"), "协同打击");
  connect(actionStrikeMission, &QAction::triggered, this, [=](){
    ui.stackedWidget->setCurrentWidget(ui.StrikeMissionPage);
    ui.docktreeWidget->setHidden(true);
    ui.dockTaskManagerWidget->setHidden(true);
    ui.rViztoolBar->setHidden(true);
  });
  ui.menu->addSeparator();

  QAction* ActionLog = ui.menu->addAction(QIcon("://icon/Log.svg"), "日志");
  connect(ActionLog, &QAction::triggered, this, [=](){
    ui.stackedWidget->setCurrentWidget(ui.Log);
    ui.docktreeWidget->setHidden(true);
    ui.dockTaskManagerWidget->setHidden(true);
    ui.rViztoolBar->setHidden(true);
  });
  ui.menu->addSeparator();

  QAction* ActionMapManager = ui.menu->addAction(QIcon("://icon/MapManager.svg"), "地图管理");
  connect(ActionMapManager, &QAction::triggered, this, [=](){
    ui.stackedWidget->setCurrentWidget(ui.MapManager);
    ui.docktreeWidget->setHidden(true);
    ui.dockTaskManagerWidget->setHidden(true);
    ui.rViztoolBar->setHidden(true);
  });
  ui.menu->addSeparator();

  QAction* ActionTaskManager = ui.menu->addAction(QIcon("://icon/TaskManager.svg"), "任务管理");
  connect(ActionTaskManager, &QAction::triggered, this, [=](){
    ui.stackedWidget->setCurrentWidget(ui.TaskManager);
    ui.docktreeWidget->setHidden(true);
    ui.dockTaskManagerWidget->setHidden(true);
    ui.rViztoolBar->setHidden(true);
  });

  #pragma region rViz 树组件设计代码
  //设置树组件头
  ui.treeWidget_layer->setHeaderLabels(QStringList() << "key" << "value");
  //设置拉伸最后列属性为false
  ui.treeWidget_layer->header()->setStretchLastSection(false);
  //设置所有列自动拉伸
  ui.treeWidget_layer->header()->setSectionResizeMode(QHeaderView::Stretch);
  //连接
  connect(ui.treeWidget_layer, &QTreeWidget::currentItemChanged, this, &MainWindow::slot_display_layer_description);
  connect(ui.treeWidget_layer, &QTreeWidget::currentItemChanged, this, &MainWindow::slot_enable_remove_button);

  /***********************************************************************************************
   *                                     设计Global Options树                                     *
   ***********************************************************************************************/
  Global = new QTreeWidgetItem(QStringList() << "全局设置");
  //设置图标
  Global->setIcon(0,QIcon("://rViz/options.png"));
  //加载根节点
  ui.treeWidget_layer->addTopLevelItem(Global);
  //设置默认展开
  Global->setExpanded(true);

  /*
   * 定义子节点1 ———— Fixed Frame(FixedFrame是RViz中用来表示“世界”坐标系的参考系。FixedFrame应该是相对于世界不动的，否则会导致显示错误)
   * 所有的固定数据应该转换到固定坐标系下。比如一堵墙的坐标应该是个固定数据吧。但如果我们把FixedFrame设置为“base_link”(机器人本体坐标系)的话，
   * 那么该坐标系会随着机器人运动而运动，那墙的坐标在该坐标系下也是运动的，这是不应该的。所以一般也不会把FixedFrame设置为“base_link”。
   * 而“map”是一个虚拟的世界坐标系，它的z轴指向正上方(天空)，它就是一个固定的坐标系，不会随着机器人运动而变化。它通常是其他坐标系的父节点，例如
   * “odom”或“base_link”。它可以通过gmapping或amcl等定位模块计算并发布。
   * “odom”是一个局部参考系，它通常是“map”的子坐标系、“base_link”的父坐标系。odom是由运动源，例如轮子运动，视觉偏移等，来计算出来的。odom相
   * 对于机器人的位置是不变的，但是相对于世界的位置是随时间变化的。odom的准确性使它在局部参考系中很有用，但是不适合作为全局参考系，因为它会有累积
   * 误差。如果我们只关心机器人的短期运动状态，也可以把FixedFrame设置为“odom”。
   */
  QTreeWidgetItem* FixedFrame = new QTreeWidgetItem(QStringList() << "固定坐标系");
  //初始化一个combobox
  FFCBox = new QComboBox();
  FFCBox->addItems(QStringList() << "map" << "odom");                     //添加项
  FFCBox->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLength);  //设置尺寸调整策略为适应最小内容
  FFCBox->setEditable(true);                                              //设置可编辑
  //连接combobox当前文本更改信号与全局设置槽函数
  connect(FFCBox, &QComboBox::currentTextChanged, this, &MainWindow::slot_set_global_options);
  //添加子节点1
  Global->addChild(FixedFrame);
  //添加combobox于子节点1的第一列
  ui.treeWidget_layer->setItemWidget(FixedFrame, 1, FFCBox);

  /*
   * 定义子节点2 ———— Background Color
   */
  QTreeWidgetItem* BackgroundColor = new QTreeWidgetItem(QStringList() << "背景颜色");
  //添加子节点2
  Global->addChild(BackgroundColor);
  //初始化一个MyPalette对象
  BPalette = new MyPalette();
  //设置初始颜色
  BPalette->setColor(238, 238, 236);
  //添加mypalette于子节点2的第一列
  ui.treeWidget_layer->setItemWidget(BackgroundColor, 1, BPalette);
  //连接mypalette颜色改变信号与全局设置槽函数
  connect(BPalette, &MyPalette::colorchanged, this, &MainWindow::slot_set_global_options);

  #pragma endregion

  #pragma region rViz dock widget其他控件相关代码
  /* Qt的元对象系统会通过我们设置的槽函数的函数名自动匹配对应控件的信号，如果没有找到对应的信号，则会报出如下错误：
   * QMetaObject::connectSlotsByName: No matching signal for on_button_add_clicked()
   * 对于上面这个错误，因为我们实际上没有名为“button_add”的控件，所以找不到对应的"点击"信号
   * 而下方代码注释掉的原因也就是因为自动匹配的关系，无需再手动connect了
   */
  //connect(ui.button_add_layer, &QPushButton::clicked, this, &MainWindow::on_button_add_layer_clicked);
  //connect(ui.button_remove_layer, &QPushButton::clicked, this, &MainWindow::on_button_remove_layer_clicked);

  #pragma endregion

  #pragma region rViztoolBar组件设计代码
  //向rviz工具栏添加设置导航初始点动作
  QAction* action_2DPoseEstimate = ui.rViztoolBar->addAction(QIcon("://rViz/SetInitialPose.png"), "2D 姿态估计");
  //连接qaction触发信号与设置导航初始点槽函数
  connect(action_2DPoseEstimate, &QAction::triggered, this, &MainWindow::slot_set_initial_pose);
  //向rviz工具栏添加设置导航目标点动作
  QAction* action_2DNavGoal = ui.rViztoolBar->addAction(QIcon("://rViz/SetGoal(2).png"), "2D 导航目标点");
  //连接qaction触发信号与设置导航目标点槽函数
  connect(action_2DNavGoal, &QAction::triggered, this, &MainWindow::slot_set_goal_pose);
  //向rviz工具栏添加发布点位动作
  QAction* action_PublishPoint = ui.rViztoolBar->addAction(QIcon("://rViz/PublishPoint.svg"), "发布点位");
  //连接qaction触发信号与发布点位槽函数
  connect(action_PublishPoint, &QAction::triggered, this, &MainWindow::slot_publish_point);

  #pragma endregion

  #pragma region 任务管理dockwidget设计代码
  //设置图标
  ui.btn_pubPoint->setIcon(QIcon("://rViz/PublishPoint.svg"));
  //连接点击按钮信号与发布点位槽函数
  connect(ui.btn_pubPoint, &QPushButton::clicked, this, [=](){
    qnode.slot_publish_clicked_point(ui.doubleSpinBox_X->value(), ui.doubleSpinBox_Y->value());
  });

  connect(ui.TaskChoose, &QComboBox::currentTextChanged, this, &MainWindow::slot_task_path_preview);
  //设置图标
  ui.btn_PublishTask->setIcon(QIcon("://taskManagement/send.svg"));
  //连接发布任务路径槽函数
  connect(ui.btn_PublishTask, &QToolButton::clicked, this, [=](){
    QTaskHandle th;     //创建任务处理句柄
    QTask* task = th.query_task_by_name(ui.TaskChoose->currentText());
    if(task){
      qnode.publish_task_path(task->path());
      QSystemTrayIcon trayIcon;
      trayIcon.setIcon(QIcon("://icon/prompt.svg"));
      trayIcon.show();
      QString msg("已发布任务(%1)路径(话题名称“/task_path”)");
      msg = msg.arg(ui.TaskChoose->currentText());
      trayIcon.showMessage("提示", msg, QSystemTrayIcon::Information, 1000);
    }
  });


  //设置图标
  ui.btn_GenerateTask->setIcon(QIcon("://taskManagement/generate_task.svg"));
  ui.btn_SaveTask->setIcon(QIcon("://taskManagement/SaveTask.svg"));
  ui.btn_PointRevocation->setIcon(QIcon("://taskManagement/revocation.svg"));
  //连接点击按钮信号与生成任务槽函数
  connect(ui.btn_GenerateTask, &QPushButton::toggled, this, &MainWindow::slot_generate_task);
  connect(ui.comboBox_ModeType, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index){
    switch(index){
      case 0:
        ui.label_ModeTypeDescription->setText("停止");
      break;
      case 1:
        ui.label_ModeTypeDescription->setText("前进");
      break;
      case 2:
        ui.label_ModeTypeDescription->setText("后退");
      break;
      case 3:
        ui.label_ModeTypeDescription->setText("左移");
      break;
      case 4:
        ui.label_ModeTypeDescription->setText("右移");
      break;
    };
  });
  connect(ui.btn_SaveTask, &QPushButton::clicked, this, &MainWindow::slot_save_task);
  connect(ui.checkBox_useStandardPoint, &QCheckBox::toggled, this, [=](){
    qnode.set_use_standard_point(ui.checkBox_useStandardPoint->isChecked());
  });
  connect(ui.btn_PointRevocation, &QPushButton::clicked, &qnode, &QNode::point_revocation);
  //设置widget默认不显示
  ui.widget_SaveTask->setVisible(false);

  //设置图标
  ui.btn_SavePoint->setIcon(QIcon("://taskManagement/PutAway.svg"));
  ui.btn_SaveByClickedPoint->setIcon(QIcon("://rViz/PublishPoint.svg"));
  ui.btn_SaveByRobotPose->setIcon(QIcon("://taskManagement/robot.svg"));
  //设置widget默认不显示
  ui.widget_SavePoint->setVisible(false);

  connect(ui.btn_SavePoint, &QPushButton::toggled, this, [=](){
    if(ui.btn_SavePoint->isChecked()){
      ui.btn_SavePoint->setIcon(QIcon("://taskManagement/expansion.svg"));
      ui.widget_SavePoint->setVisible(true);
    }
    else{
      ui.btn_SavePoint->setIcon(QIcon("://taskManagement/PutAway.svg"));
      ui.widget_SavePoint->setVisible(false);
    }
  });

  connect(ui.btn_SaveByClickedPoint, &QPushButton::clicked, this, [=](){
    if(qnode.isClicked())
    {
      ui.btn_SavePoint->setIcon(QIcon("://taskManagement/PutAway.svg"));
      ui.btn_SavePoint->setChecked(false);
      ui.widget_SavePoint->setVisible(false);
      sp_ptr = new SavePoint(qnode.get_yaml_file_path(), QNode::clicked_point[0], QNode::clicked_point[1]);
      connect(sp_ptr, &SavePoint::pointUpdated, this, &MainWindow::slot_on_point_updated, Qt::UniqueConnection);
      sp_ptr->exec();
      delete sp_ptr;
    }
    else
    {
      QSystemTrayIcon trayIcon;
      trayIcon.setIcon(QIcon("://icon/warning.svg"));
      trayIcon.show();
      trayIcon.showMessage("提示", "您需要先点一个点哦!", QSystemTrayIcon::Information, 2000);
    }
  });

  connect(ui.btn_SaveByRobotPose, &QPushButton::clicked, this, [=](){
    ui.btn_SavePoint->setIcon(QIcon("://taskManagement/PutAway.svg"));
    ui.btn_SavePoint->setChecked(false);
    ui.widget_SavePoint->setVisible(false);
    sp_ptr = new SavePoint(qnode.get_yaml_file_path(), QNode::clicked_point[0], QNode::clicked_point[1]);
    connect(sp_ptr, &SavePoint::pointUpdated, this, &MainWindow::slot_on_point_updated, Qt::UniqueConnection);
    sp_ptr->exec();
    delete sp_ptr;
  });

  connect(ui.checkBox_ShowAllStandardPoints, &QCheckBox::toggled, this, &MainWindow::slot_show_all_standard_points);
  connect(ui.buttonApplyCarMapView, &QPushButton::clicked, this, &MainWindow::updateDualMapViews);
  connect(ui.buttonApplyDroneMapView, &QPushButton::clicked, this, &MainWindow::updateDualMapViews);
  connect(ui.buttonApplyCarMapView, &QPushButton::clicked, this, [=](){
    playDualMapVideo(true);
  });
  connect(ui.buttonApplyDroneMapView, &QPushButton::clicked, this, [=](){
    playDualMapVideo(false);
  });
  connect(ui.buttonAddCarDisplay, &QPushButton::clicked, this, [=](){
    addDisplayToDualMapView(carMapViz, ui.listWidgetCarDisplays, "小车建图 - 添加显示");
  });
  connect(ui.buttonRemoveCarDisplay, &QPushButton::clicked, this, [=](){
    removeSelectedDualMapDisplay(carMapViz, ui.listWidgetCarDisplays);
  });
  connect(ui.buttonAddDroneDisplay, &QPushButton::clicked, this, [=](){
    addDisplayToDualMapView(droneMapViz, ui.listWidgetDroneDisplays, "无人机建图 - 添加显示");
  });
  connect(ui.buttonRemoveDroneDisplay, &QPushButton::clicked, this, [=](){
    removeSelectedDualMapDisplay(droneMapViz, ui.listWidgetDroneDisplays);
  });
  connect(ui.buttonToggleCarSidebar, &QToolButton::toggled, this, [=](bool checked){
    ui.widgetCarSidebarContent->setVisible(checked);
    ui.buttonToggleCarSidebar->setText(checked ? "收起左边栏" : "展开左边栏");
    ui.widgetCarSidebar->setMaximumWidth(checked ? 360 : 120);
    updateDualMapVideoOverlayGeometry();
  });
  connect(ui.buttonToggleDroneSidebar, &QToolButton::toggled, this, [=](bool checked){
    ui.widgetDroneSidebarContent->setVisible(checked);
    ui.buttonToggleDroneSidebar->setText(checked ? "收起左边栏" : "展开左边栏");
    ui.widgetDroneSidebar->setMaximumWidth(checked ? 360 : 120);
    updateDualMapVideoOverlayGeometry();
  });
  ui.listWidgetCarDisplays->header()->setSectionResizeMode(QHeaderView::Stretch);
  ui.listWidgetDroneDisplays->header()->setSectionResizeMode(QHeaderView::Stretch);
  connect(ui.listWidgetCarDisplays, &QTreeWidget::currentItemChanged, this, [=](QTreeWidgetItem*, QTreeWidgetItem*){
    updateDualMapDisplayDescription(ui.listWidgetCarDisplays, ui.textBrowserCarDisplayDescription);
  });
  connect(ui.listWidgetDroneDisplays, &QTreeWidget::currentItemChanged, this, [=](QTreeWidgetItem*, QTreeWidgetItem*){
    updateDualMapDisplayDescription(ui.listWidgetDroneDisplays, ui.textBrowserDroneDisplayDescription);
  });
  connect(ui.listWidgetCarDisplays, &QTreeWidget::itemChanged, this, [=](QTreeWidgetItem* item, int){
    rviz_common::Display* display = reinterpret_cast<rviz_common::Display*>(static_cast<quintptr>(item->data(0, Qt::UserRole).toULongLong()));
    if (display != nullptr) {
      display->setEnabled(item->checkState(0) == Qt::Checked);
      QVariantMap properties = item->data(0, Qt::UserRole + 4).toMap();
      properties.insert("enabled", item->checkState(0) == Qt::Checked);
      item->setData(0, Qt::UserRole + 4, properties);
      refreshDualMapDisplayItem(item);
    }
  });
  connect(ui.listWidgetDroneDisplays, &QTreeWidget::itemChanged, this, [=](QTreeWidgetItem* item, int){
    rviz_common::Display* display = reinterpret_cast<rviz_common::Display*>(static_cast<quintptr>(item->data(0, Qt::UserRole).toULongLong()));
    if (display != nullptr) {
      display->setEnabled(item->checkState(0) == Qt::Checked);
      QVariantMap properties = item->data(0, Qt::UserRole + 4).toMap();
      properties.insert("enabled", item->checkState(0) == Qt::Checked);
      item->setData(0, Qt::UserRole + 4, properties);
      refreshDualMapDisplayItem(item);
    }
  });
  connect(ui.listWidgetCarDisplays, &QTreeWidget::itemDoubleClicked, this, [=](QTreeWidgetItem* item, int){
    editDualMapDisplay(carMapViz, item);
  });
  connect(ui.listWidgetDroneDisplays, &QTreeWidget::itemDoubleClicked, this, [=](QTreeWidgetItem* item, int){
    editDualMapDisplay(droneMapViz, item);
  });
  ui.widgetCarSidebarContent->setVisible(ui.buttonToggleCarSidebar->isChecked());
  ui.widgetDroneSidebarContent->setVisible(ui.buttonToggleDroneSidebar->isChecked());

  initializeDualMapVideoOverlays();
  initializeAerialPhotoPage();

  const auto loadOrthoDetectionResult = [=]() -> QString {
    const QString imagePath = findOrthoDetectionImage(kDemoVideoRoot);
    if (imagePath.isEmpty()) {
      return QString();
    }

    setZoomablePreview(ui.labelStitchingPreview, QPixmap(imagePath));
    setZoomablePreview(ui.labelUxodDetectionPreview, QPixmap(imagePath));
    setZoomablePreview(ui.labelStrikeMissionMosaic, QPixmap(imagePath));
    return imagePath;
  };

  connect(ui.buttonImportImage, &QPushButton::clicked, this, [=](){
    const QString imagePath = QFileDialog::getOpenFileName(
      this,
      "导入图片",
      QDir::homePath(),
      "Images (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)"
    );
    if (imagePath.isEmpty()) {
      return;
    }

    QPixmap pixmap(imagePath);
    if (pixmap.isNull()) {
      ui.textBrowserImageAnalysisStatus->setText(QString("导入失败: %1").arg(imagePath));
      return;
    }

    const QPixmap scaledPixmap = pixmap.scaled(
      ui.labelImportedImagePreview->size(),
      Qt::KeepAspectRatio,
      Qt::SmoothTransformation
    );
    ui.labelImportedImagePreview->setPixmap(scaledPixmap);
    ui.labelImportedImagePreview->setText(QString());
    setZoomablePreview(ui.labelStitchingPreview, pixmap);
    setZoomablePreview(ui.labelUxodDetectionPreview, pixmap);

    ui.labelImageAnalysisSectionTitle->setText("图像导入预览");
    ui.stackedWidgetImageAnalysisResult->setCurrentWidget(ui.pageImportedImage);
    ui.textBrowserImageAnalysisStatus->setText(
      QString("已导入图片:\n%1\n\n可继续选择“拼图”或“未爆弹识别”查看对应效果页。").arg(imagePath)
    );
  });

  connect(ui.buttonShowStitching, &QPushButton::clicked, this, [=](){
    ui.labelImageAnalysisSectionTitle->setText("拼图效果");
    ui.stackedWidgetImageAnalysisResult->setCurrentWidget(ui.pageStitchingResult);
    const QString imagePath = loadOrthoDetectionResult();
    if (imagePath.isEmpty()) {
      ui.textBrowserImageAnalysisStatus->setText("未找到“正射影像图识别后”的结果图片。");
      return;
    }
    ui.textBrowserImageAnalysisStatus->setText("当前显示正射影像拼图结果。");
  });

  connect(ui.buttonShowUxodDetection, &QPushButton::clicked, this, [=](){
    ui.labelImageAnalysisSectionTitle->setText("未爆弹识别效果");
    ui.stackedWidgetImageAnalysisResult->setCurrentWidget(ui.pageUxodDetectionResult);
    const QString imagePath = loadOrthoDetectionResult();
    if (imagePath.isEmpty()) {
      ui.textBrowserImageAnalysisStatus->setText("未找到“正射影像图识别后”的结果图片。");
      return;
    }
    ui.textBrowserImageAnalysisStatus->setText("当前显示未爆弹识别结果。");
  });

  ui.tableWidgetStrikeTargets->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  ui.tableWidgetStrikeTargets->setSelectionBehavior(QAbstractItemView::SelectRows);
  ui.tableWidgetStrikeTargets->setSelectionMode(QAbstractItemView::SingleSelection);
  ui.tableWidgetStrikeTargets->setEditTriggers(QAbstractItemView::NoEditTriggers);
  ui.buttonSendTargetToAgv->setEnabled(false);
  ui.buttonStartAgvAttack->setEnabled(false);
  ui.buttonLaserStrike->setEnabled(false);
  ui.buttonDroneVerify->setEnabled(false);

  connect(ui.buttonLoadStrikeMosaic, &QPushButton::clicked, this, [=](){
    const QString imagePath = loadOrthoDetectionResult();
    if (imagePath.isEmpty()) {
      ui.textBrowserStrikeMissionStatus->setText("未找到“正射影像图识别后”的结果图片。");
      return;
    }
    ui.tableWidgetStrikeTargets->setRowCount(0);

    const struct {
      const char* id;
      double x;
      double y;
      const char* status;
    } sampleTargets[] = {
      {"UXO-01", 12.40, 8.70, "待下发"},
      {"UXO-02", 18.15, 5.20, "待下发"},
      {"UXO-03", 7.85, 14.35, "待下发"},
      {"UXO-04", 22.60, 11.10, "待下发"},
      {"UXO-05", 15.75, 19.45, "待下发"},
      {"UXO-06", 9.30, 24.80, "待下发"},
      {"UXO-07", 27.95, 13.60, "待下发"},
      {"UXO-08", 31.20, 7.45, "待下发"},
      {"UXO-09", 20.85, 27.10, "待下发"},
      {"UXO-10", 5.65, 18.25, "待下发"},
      {"UXO-11", 34.40, 22.75, "待下发"},
      {"UXO-12", 25.10, 30.90, "待下发"}
    };

    for (int i = 0; i < 12; ++i) {
      ui.tableWidgetStrikeTargets->insertRow(i);
      ui.tableWidgetStrikeTargets->setItem(i, 0, new QTableWidgetItem(sampleTargets[i].id));
      ui.tableWidgetStrikeTargets->setItem(i, 1, new QTableWidgetItem(QString::number(sampleTargets[i].x, 'f', 2)));
      ui.tableWidgetStrikeTargets->setItem(i, 2, new QTableWidgetItem(QString::number(sampleTargets[i].y, 'f', 2)));
      ui.tableWidgetStrikeTargets->setItem(i, 3, new QTableWidgetItem(sampleTargets[i].status));
    }

    ui.labelSelectedStrikeTarget->setText("当前目标：未选择");
    ui.buttonSendTargetToAgv->setEnabled(true);
    ui.buttonStartAgvAttack->setEnabled(false);
    ui.buttonLaserStrike->setEnabled(false);
    ui.buttonDroneVerify->setEnabled(false);
    ui.textBrowserStrikeMissionStatus->setText("已加载拼图与未爆弹识别结果。左侧已生成 12 个模拟未爆弹目标，请选择后下发给地面端小车。");
  });

  connect(ui.tableWidgetStrikeTargets, &QTableWidget::currentCellChanged, this, [=](int currentRow, int, int, int){
    if (currentRow < 0) {
      ui.labelSelectedStrikeTarget->setText("当前目标：未选择");
      return;
    }
    const QString targetId = ui.tableWidgetStrikeTargets->item(currentRow, 0)->text();
    const QString x = ui.tableWidgetStrikeTargets->item(currentRow, 1)->text();
    const QString y = ui.tableWidgetStrikeTargets->item(currentRow, 2)->text();
    ui.labelSelectedStrikeTarget->setText(QString("当前目标：%1  坐标(%2, %3)").arg(targetId, x, y));
  });

  connect(ui.buttonSendTargetToAgv, &QPushButton::clicked, this, [=](){
    if (ui.button_connect->isEnabled()) {
      ui.textBrowserStrikeMissionStatus->setText("请先启动 ROS 2 节点，再向地面端小车下发目标坐标。");
      return;
    }
    const int row = ui.tableWidgetStrikeTargets->currentRow();
    if (row < 0) {
      ui.textBrowserStrikeMissionStatus->setText("请先在表格中选择一个未爆弹目标。");
      return;
    }
    const QString targetId = ui.tableWidgetStrikeTargets->item(row, 0)->text();
    const double x = ui.tableWidgetStrikeTargets->item(row, 1)->text().toDouble();
    const double y = ui.tableWidgetStrikeTargets->item(row, 2)->text().toDouble();
    qnode.slot_publish_clicked_point(x, y);
    ui.tableWidgetStrikeTargets->item(row, 3)->setText("已下发");
    ui.buttonStartAgvAttack->setEnabled(true);
    ui.buttonLaserStrike->setEnabled(false);
    ui.buttonDroneVerify->setEnabled(false);
    ui.textBrowserStrikeMissionStatus->setText(
      QString("已将目标 %1 的坐标下发给地面端小车：(%2, %3)\n\n下一步可启动小车机动。")
      .arg(targetId)
      .arg(x, 0, 'f', 2)
      .arg(y, 0, 'f', 2)
    );
  });

  connect(ui.buttonStartAgvAttack, &QPushButton::clicked, this, [=](){
    const int row = ui.tableWidgetStrikeTargets->currentRow();
    if (row < 0) {
      ui.textBrowserStrikeMissionStatus->setText("请先选择已下发的未爆弹目标。");
      return;
    }
    ui.tableWidgetStrikeTargets->item(row, 3)->setText("小车机动中");
    ui.buttonLaserStrike->setEnabled(true);
    ui.textBrowserStrikeMissionStatus->setText("小车已开始导航机动。");
  });

  connect(ui.buttonLaserStrike, &QPushButton::clicked, this, [=](){
    const int row = ui.tableWidgetStrikeTargets->currentRow();
    if (row < 0) {
      ui.textBrowserStrikeMissionStatus->setText("请先选择当前打击目标。");
      return;
    }
    ui.tableWidgetStrikeTargets->item(row, 3)->setText("已激光打击");
    ui.buttonDroneVerify->setEnabled(true);
    ui.textBrowserStrikeMissionStatus->setText("已执行激光打击。\n下一步可调度无人机飞临目标上空复核。");
  });

  connect(ui.buttonDroneVerify, &QPushButton::clicked, this, [=](){
    const int row = ui.tableWidgetStrikeTargets->currentRow();
    if (row < 0) {
      ui.textBrowserStrikeMissionStatus->setText("请先选择当前复核目标。");
      return;
    }
    ui.tableWidgetStrikeTargets->item(row, 3)->setText("等待空中复核");
    ui.textBrowserStrikeMissionStatus->setText("无人机已飞临目标区域。");
  });

  #pragma endregion

  #pragma region MapManager 地图管理窗口设计代码
  ui.tableWidget_mapManager->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);   //将QTableWidget表头大小策略设置为"Stretch"

  connect(ui.btn_AddMapManager, &QPushButton::clicked, this, [=](){
    QStringList mapMsg = AddMapManager::getMapMessage(this);
    if(mapMsg.size() == 3)
    {
      add_TableWidgetRow(ui.tableWidget_mapManager, mapMsg[0], mapMsg[1], mapMsg[2]);
    }
  });
  #pragma endregion

  #pragma region TaskManager 任务管理窗口设计代码
  ui.tableWidget_taskManager->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);  //列表头大小策略为“适应内容”
  ui.tableWidget_taskManager->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);   //将QTableWidget行表头大小策略设置为"Stretch"

  #pragma endregion

  /*********************
  ** Auto Start
  **********************/
  if ( ui.checkbox_remember_settings->isChecked() ) {
    on_button_connect_clicked(true);
  }
}

MainWindow::~MainWindow()
{
  stopFpvStream();
  delete mapServerProcess;
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showRos2InitFailureMessage() {
	QMessageBox msgBox;
	msgBox.setText("ROS 2 节点启动失败，请检查 ROS 2 环境是否已正确 source。");
	msgBox.exec();
  close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
  (void)check;

  const bool connected = ui.checkbox_use_environment->isChecked()
    ? qnode.init()
    : qnode.init(ui.line_edit_domain_id->text().trimmed().toStdString(),
                 ui.line_edit_node_name->text().trimmed().toStdString());

  if (!connected) {
    showRos2InitFailureMessage();
    return;
  }

  ui.button_connect->setEnabled(false);
  ui.line_edit_domain_id->setReadOnly(true);
  ui.line_edit_node_name->setReadOnly(true);
  ui.line_edit_topic->setReadOnly(true);
  ui.rViztoolBar->setEnabled(true);
  ui.dockWidgetContents->setEnabled(true);
  ui.dockWidgetContents_2->setEnabled(true);
  myviz = new qrviz(ui.RVizLayout);
  ensureDualMapViewsInitialized();
  QString str("网格");
  add_TreeWidgetItem_Grid(ui.treeWidget_layer, str);
}



void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_domain_id->setEnabled(enabled);
	ui.line_edit_node_name->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

void MainWindow::ensureDualMapViewsInitialized()
{
  if (carMapViz == nullptr) {
    carMapViz = new qrviz(ui.CarRVizLayout);
    carGridDisplay = createDualMapDisplay(carMapViz, AddLayer::grid, defaultDualMapDisplayProperties(AddLayer::grid, "小车网格", QString()));
    carMapDisplay = createDualMapDisplay(carMapViz, AddLayer::map, defaultDualMapDisplayProperties(AddLayer::map, "小车地图", ui.lineEditCarMapTopic->text().trimmed()));
    appendDualMapDisplayItem(ui.listWidgetCarDisplays, carGridDisplay, AddLayer::grid, "小车网格", QString());
    appendDualMapDisplayItem(ui.listWidgetCarDisplays, carMapDisplay, AddLayer::map, "小车地图", ui.lineEditCarMapTopic->text().trimmed());
    if (ui.listWidgetCarDisplays->topLevelItemCount() > 0) {
      ui.listWidgetCarDisplays->setCurrentItem(ui.listWidgetCarDisplays->topLevelItem(0));
    }
  }

  if (droneMapViz == nullptr) {
    droneMapViz = new qrviz(ui.DroneRVizLayout);
    droneGridDisplay = createDualMapDisplay(droneMapViz, AddLayer::grid, defaultDualMapDisplayProperties(AddLayer::grid, "无人机网格", QString()));
    droneMapDisplay = createDualMapDisplay(droneMapViz, AddLayer::map, defaultDualMapDisplayProperties(AddLayer::map, "无人机地图", ui.lineEditDroneMapTopic->text().trimmed()));
    appendDualMapDisplayItem(ui.listWidgetDroneDisplays, droneGridDisplay, AddLayer::grid, "无人机网格", QString());
    appendDualMapDisplayItem(ui.listWidgetDroneDisplays, droneMapDisplay, AddLayer::map, "无人机地图", ui.lineEditDroneMapTopic->text().trimmed());
    if (ui.listWidgetDroneDisplays->topLevelItemCount() > 0) {
      ui.listWidgetDroneDisplays->setCurrentItem(ui.listWidgetDroneDisplays->topLevelItem(0));
    }
  }

  updateDualMapViews();
  updateDualMapVideoOverlayGeometry();
}

void MainWindow::initializeDualMapVideoOverlays()
{
  carMapVideoOverlay = new QLabel(ui.groupBox_carMap);
  carMapVideoOverlay->setAlignment(Qt::AlignCenter);
  carMapVideoOverlay->setStyleSheet(
    "background-color: #05070b;"
    "color: rgba(255,255,255,0.78);"
    "border: 1px solid rgba(255,255,255,0.10);"
  );
  carMapVideoOverlay->setText("小车建图画面");
  carMapVideoOverlay->hide();

  droneMapVideoOverlay = new QLabel(ui.groupBox_droneMap);
  droneMapVideoOverlay->setAlignment(Qt::AlignCenter);
  droneMapVideoOverlay->setStyleSheet(
    "background-color: #05070b;"
    "color: rgba(255,255,255,0.78);"
    "border: 1px solid rgba(255,255,255,0.10);"
  );
  droneMapVideoOverlay->setText("无人机建图画面");
  droneMapVideoOverlay->hide();

  carMapVideoTimer = new QTimer(this);
  connect(carMapVideoTimer, &QTimer::timeout, this, [=](){
    if (!carMapVideoCapture.isOpened()) {
      return;
    }
    cv::Mat frame;
    if (!carMapVideoCapture.read(frame) || frame.empty()) {
      carMapVideoCapture.set(cv::CAP_PROP_POS_FRAMES, 0);
      if (!carMapVideoCapture.read(frame) || frame.empty()) {
        stopDualMapVideo(true);
        if (carMapVideoOverlay != nullptr) {
          carMapVideoOverlay->setText("小车建图画面加载失败");
          carMapVideoOverlay->show();
        }
        return;
      }
    }
    updateVideoLabel(carMapVideoOverlay, frame, Qt::KeepAspectRatioByExpanding);
  });

  droneMapVideoTimer = new QTimer(this);
  connect(droneMapVideoTimer, &QTimer::timeout, this, [=](){
    if (!droneMapVideoCapture.isOpened()) {
      return;
    }
    cv::Mat frame;
    if (!droneMapVideoCapture.read(frame) || frame.empty()) {
      droneMapVideoCapture.set(cv::CAP_PROP_POS_FRAMES, 0);
      if (!droneMapVideoCapture.read(frame) || frame.empty()) {
        stopDualMapVideo(false);
        if (droneMapVideoOverlay != nullptr) {
          droneMapVideoOverlay->setText("无人机建图画面加载失败");
          droneMapVideoOverlay->show();
        }
        return;
      }
    }
    updateVideoLabel(droneMapVideoOverlay, frame, Qt::KeepAspectRatioByExpanding);
  });

  updateDualMapVideoOverlayGeometry();
}

void MainWindow::updateDualMapVideoOverlayGeometry()
{
  if (carMapVideoOverlay != nullptr) {
    QWidget* target = carMapViz == nullptr ? nullptr : carMapViz->panelWidget();
    if (target != nullptr) {
      carMapVideoOverlay->setGeometry(target->geometry());
    }
  }

  if (droneMapVideoOverlay != nullptr) {
    QWidget* target = droneMapViz == nullptr ? nullptr : droneMapViz->panelWidget();
    if (target != nullptr) {
      droneMapVideoOverlay->setGeometry(target->geometry());
    }
  }
}

QString MainWindow::resolveDemoVideoPath(const QString& stem) const
{
  const QStringList candidates = {
    QString("%1/%2.mp4").arg(kDemoVideoRoot, stem),
    QString("%1/%2.MP4").arg(kDemoVideoRoot, stem),
    QString("%1/%2/%2.mp4").arg(kDemoVideoRoot, stem),
    QString("%1/%2/%2.MP4").arg(kDemoVideoRoot, stem)
  };

  for (const QString& candidate : candidates) {
    if (QFileInfo(candidate).exists()) {
      return candidate;
    }
  }
  return candidates.first();
}

void MainWindow::playDualMapVideo(bool isCar)
{
  QLabel* overlay = isCar ? carMapVideoOverlay : droneMapVideoOverlay;
  QTimer* timer = isCar ? carMapVideoTimer : droneMapVideoTimer;
  cv::VideoCapture& capture = isCar ? carMapVideoCapture : droneMapVideoCapture;
  QString& currentPath = isCar ? carMapCurrentVideoPath : droneMapCurrentVideoPath;
  qrviz* viz = isCar ? carMapViz : droneMapViz;
  const QString videoPath = resolveDemoVideoPath(isCar ? "car" : "plane");

  if (overlay == nullptr) {
    return;
  }

  if (capture.isOpened()) {
    capture.release();
  }
  if (!capture.open(videoPath.toStdString())) {
    overlay->setPixmap(QPixmap());
    overlay->setText(isCar ? "小车建图画面加载失败" : "无人机建图画面加载失败");
    overlay->show();
    overlay->raise();
    return;
  }

  cv::Mat firstFrame;
  if (!capture.read(firstFrame) || firstFrame.empty()) {
    capture.release();
    overlay->setPixmap(QPixmap());
    overlay->setText(isCar ? "小车建图画面加载失败" : "无人机建图画面加载失败");
    overlay->show();
    overlay->raise();
    return;
  }

  currentPath = videoPath;
  if (viz != nullptr && viz->panelWidget() != nullptr) {
    viz->panelWidget()->hide();
  }
  updateDualMapVideoOverlayGeometry();
  overlay->setText(QString());
  updateVideoLabel(overlay, firstFrame, Qt::KeepAspectRatioByExpanding);
  overlay->show();
  overlay->raise();
  const double fps = capture.get(cv::CAP_PROP_FPS);
  const int intervalMs = fps > 1.0 ? qMax(15, qRound(1000.0 / fps)) : 33;
  if (timer != nullptr) {
    timer->start(intervalMs);
  }
}

void MainWindow::stopDualMapVideo(bool isCar)
{
  QLabel* overlay = isCar ? carMapVideoOverlay : droneMapVideoOverlay;
  QTimer* timer = isCar ? carMapVideoTimer : droneMapVideoTimer;
  cv::VideoCapture& capture = isCar ? carMapVideoCapture : droneMapVideoCapture;
  QString& currentPath = isCar ? carMapCurrentVideoPath : droneMapCurrentVideoPath;
  qrviz* viz = isCar ? carMapViz : droneMapViz;

  if (timer != nullptr) {
    timer->stop();
  }
  if (capture.isOpened()) {
    capture.release();
  }
  currentPath.clear();
  if (overlay != nullptr) {
    overlay->clear();
    overlay->hide();
  }
  if (viz != nullptr && viz->panelWidget() != nullptr) {
    viz->panelWidget()->show();
  }
}

void MainWindow::ensureStrikeMissionViewInitialized()
{
  return;
}

void MainWindow::updateDualMapViews()
{
  if (carMapViz == nullptr || droneMapViz == nullptr) {
    return;
  }

  const QString carFixedFrame = ui.lineEditCarFixedFrame->text().trimmed().isEmpty() ? "map" : ui.lineEditCarFixedFrame->text().trimmed();
  const QString droneFixedFrame = ui.lineEditDroneFixedFrame->text().trimmed().isEmpty() ? "map" : ui.lineEditDroneFixedFrame->text().trimmed();
  const QString carMapTopic = ui.lineEditCarMapTopic->text().trimmed().isEmpty() ? "/map" : ui.lineEditCarMapTopic->text().trimmed();
  const QString droneMapTopic = ui.lineEditDroneMapTopic->text().trimmed().isEmpty() ? "/map" : ui.lineEditDroneMapTopic->text().trimmed();

  carMapViz->SetFixedFrame(carFixedFrame);
  for (int i = 0; i < ui.listWidgetCarDisplays->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = ui.listWidgetCarDisplays->topLevelItem(i);
    rviz_common::Display* display = reinterpret_cast<rviz_common::Display*>(static_cast<quintptr>(item->data(0, Qt::UserRole).toULongLong()));
    QVariantMap properties = item->data(0, Qt::UserRole + 4).toMap();
    if (display == carGridDisplay) {
      applyDualMapDisplayProperties(carMapViz, carGridDisplay, AddLayer::grid, properties);
      refreshDualMapDisplayItem(item);
    } else if (display == carMapDisplay) {
      QVariantMap properties = item->data(0, Qt::UserRole + 4).toMap();
      properties.insert("topic", carMapTopic);
      item->setData(0, Qt::UserRole + 4, properties);
      item->setData(0, Qt::UserRole + 3, carMapTopic);
      applyDualMapDisplayProperties(carMapViz, carMapDisplay, AddLayer::map, properties);
      refreshDualMapDisplayItem(item);
      if (item == ui.listWidgetCarDisplays->currentItem()) {
        updateDualMapDisplayDescription(ui.listWidgetCarDisplays, ui.textBrowserCarDisplayDescription);
      }
    }
  }

  droneMapViz->SetFixedFrame(droneFixedFrame);
  for (int i = 0; i < ui.listWidgetDroneDisplays->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = ui.listWidgetDroneDisplays->topLevelItem(i);
    rviz_common::Display* display = reinterpret_cast<rviz_common::Display*>(static_cast<quintptr>(item->data(0, Qt::UserRole).toULongLong()));
    QVariantMap properties = item->data(0, Qt::UserRole + 4).toMap();
    if (display == droneGridDisplay) {
      applyDualMapDisplayProperties(droneMapViz, droneGridDisplay, AddLayer::grid, properties);
      refreshDualMapDisplayItem(item);
    } else if (display == droneMapDisplay) {
      QVariantMap properties = item->data(0, Qt::UserRole + 4).toMap();
      properties.insert("topic", droneMapTopic);
      item->setData(0, Qt::UserRole + 4, properties);
      item->setData(0, Qt::UserRole + 3, droneMapTopic);
      applyDualMapDisplayProperties(droneMapViz, droneMapDisplay, AddLayer::map, properties);
      refreshDualMapDisplayItem(item);
      if (item == ui.listWidgetDroneDisplays->currentItem()) {
        updateDualMapDisplayDescription(ui.listWidgetDroneDisplays, ui.textBrowserDroneDisplayDescription);
      }
    }
  }
}

QVariantMap MainWindow::defaultDualMapDisplayProperties(int layerCode, const QString& displayName, const QString& topicName) const
{
  QVariantMap properties;
  properties.insert("display_name", displayName);
  properties.insert("topic", topicName);
  properties.insert("enabled", true);
  properties.insert("cell_count", 30);
  properties.insert("color", QColor(160, 160, 164));
  properties.insert("path_color", QColor(25, 255, 0));
  properties.insert("point_color", QColor(255, 0, 0));
  properties.insert("style", "Points");
  properties.insert("point_size", 3.0);
  properties.insert("color_scheme", "map");
  if (layerCode == AddLayer::grid) {
    properties.insert("topic", QString());
  }
  return properties;
}

rviz_common::Display* MainWindow::createDualMapDisplay(qrviz* viz, int layerCode, const QVariantMap& properties)
{
  if (viz == nullptr) {
    return nullptr;
  }

  rviz_common::Display* display = nullptr;
  const QString displayName = properties.value("display_name").toString();
  switch (layerCode) {
    case AddLayer::grid:
      display = viz->CreateDisplay("rviz_default_plugins/Grid", displayName);
    break;
    case AddLayer::tf:
      display = viz->CreateDisplay("rviz_default_plugins/TF", displayName);
    break;
    case AddLayer::laserscan:
      display = viz->CreateDisplay("rviz_default_plugins/LaserScan", displayName);
    break;
    case AddLayer::pointcloud2:
      display = viz->CreateDisplay("rviz_default_plugins/PointCloud2", displayName);
    break;
    case AddLayer::robotmodel:
      display = viz->CreateDisplay("rviz_default_plugins/RobotModel", displayName);
    break;
    case AddLayer::map:
      display = viz->CreateDisplay("rviz_default_plugins/Map", displayName);
    break;
    case AddLayer::path:
      display = viz->CreateDisplay("rviz_default_plugins/Path", displayName);
    break;
    case AddLayer::pointstamped:
      display = viz->CreateDisplay("rviz_default_plugins/PointStamped", displayName);
    break;
    case AddLayer::marker:
      display = viz->CreateDisplay("rviz_default_plugins/Marker", displayName);
    break;
    case AddLayer::markerarray:
      display = viz->CreateDisplay("rviz_default_plugins/MarkerArray", displayName);
    break;
    default:
    break;
  }

  if (display != nullptr) {
    applyDualMapDisplayProperties(viz, display, layerCode, properties);
  }

  return display;
}

void MainWindow::applyDualMapDisplayProperties(qrviz* viz, rviz_common::Display* display, int layerCode, const QVariantMap& properties)
{
  if (viz == nullptr || display == nullptr) {
    return;
  }

  const QString topic = properties.value("topic").toString();
  const bool enabled = properties.value("enabled", true).toBool();
  const QString style = properties.value("style", "Points").toString();
  const double pointSize = properties.value("point_size", 3.0).toDouble();
  const QColor color = properties.value("color", QColor(160, 160, 164)).value<QColor>();
  const QColor pathColor = properties.value("path_color", QColor(25, 255, 0)).value<QColor>();
  const QColor pointColor = properties.value("point_color", QColor(255, 0, 0)).value<QColor>();
  const QString colorScheme = properties.value("color_scheme", "map").toString();
  const int cellCount = properties.value("cell_count", 30).toInt();

  display->setName(properties.value("display_name").toString());
  switch (layerCode) {
    case AddLayer::grid:
      viz->ModifyGridAttribute(display, cellCount, color, enabled);
    break;
    case AddLayer::tf:
      viz->ModifyTFAttribute(display, enabled);
    break;
    case AddLayer::laserscan:
      viz->ModifyLaserScanAttribute(display, topic, style, pointSize, enabled);
    break;
    case AddLayer::pointcloud2:
      viz->ModifyPointCloud2Attribute(display, topic, style, pointSize, enabled);
    break;
    case AddLayer::robotmodel:
      viz->ModifyRobotModelAttribute(display, enabled);
    break;
    case AddLayer::map:
      viz->ModifyMapAttribute(display, topic.isEmpty() ? "/map" : topic, colorScheme, enabled);
    break;
    case AddLayer::path:
      viz->ModifyPathAttribute(display, topic, pathColor, enabled);
    break;
    case AddLayer::pointstamped:
      viz->ModifyPointStampedAttribute(display, topic, pointColor, enabled);
    break;
    case AddLayer::marker:
      viz->ModifyMarkerAttribute(display, topic, enabled);
    break;
    case AddLayer::markerarray:
      viz->ModifyMarkerArrayAttribute(display, topic, enabled);
    break;
    default:
    break;
  }
}

QTreeWidgetItem* MainWindow::appendDualMapDisplayItem(QTreeWidget* treeWidget, rviz_common::Display* display, int layerCode, const QString& displayName, const QString& topicName)
{
  if (treeWidget == nullptr || display == nullptr) {
    return nullptr;
  }

  QTreeWidgetItem* item = new QTreeWidgetItem();
  item->setFlags(item->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable | Qt::ItemIsEnabled);
  item->setCheckState(0, display->isEnabled() ? Qt::Checked : Qt::Unchecked);
  item->setData(0, Qt::UserRole, static_cast<qulonglong>(reinterpret_cast<quintptr>(display)));
  item->setData(0, Qt::UserRole + 1, layerCode);
  item->setData(0, Qt::UserRole + 2, displayName);
  item->setData(0, Qt::UserRole + 3, topicName);
  item->setData(0, Qt::UserRole + 4, defaultDualMapDisplayProperties(layerCode, displayName, topicName));
  treeWidget->addTopLevelItem(item);
  refreshDualMapDisplayItem(item);
  return item;
}

void MainWindow::addDisplayToDualMapView(qrviz* viz, QTreeWidget* treeWidget, const QString& dialogTitle)
{
  if (viz == nullptr || treeWidget == nullptr) {
    return;
  }

  int layerCode = AddLayer::getLayer(this, dialogTitle);
  if (layerCode == AddLayer::null) {
    return;
  }

  const QString displayName = AddLayer::getDisplayName();
  QString topicName = AddLayer::getTopicName();
  if (topicName.isEmpty()) {
    const QStringList topics = AddLayer::getTopicListByLayerCode(static_cast<AddLayer::LayerCode>(layerCode));
    if (!topics.isEmpty()) {
      topicName = topics.first();
    }
  }
  QVariantMap properties = defaultDualMapDisplayProperties(layerCode, displayName, topicName);
  rviz_common::Display* display = createDualMapDisplay(viz, layerCode, properties);
  if (display == nullptr) {
    return;
  }

  QTreeWidgetItem* item = appendDualMapDisplayItem(treeWidget, display, layerCode, displayName, topicName);
  if (item != nullptr) {
    item->setData(0, Qt::UserRole + 4, properties);
  }
}

void MainWindow::removeSelectedDualMapDisplay(qrviz* viz, QTreeWidget* treeWidget)
{
  if (viz == nullptr || treeWidget == nullptr || treeWidget->currentItem() == nullptr) {
    return;
  }

  QTreeWidgetItem* item = treeWidget->currentItem();
  rviz_common::Display* display = reinterpret_cast<rviz_common::Display*>(static_cast<quintptr>(item->data(0, Qt::UserRole).toULongLong()));

  if (display == carGridDisplay || display == carMapDisplay || display == droneGridDisplay || display == droneMapDisplay) {
    QSystemTrayIcon trayIcon;
    trayIcon.setIcon(QIcon("://icon/warning.svg"));
    trayIcon.show();
    trayIcon.showMessage("提示", "默认网格和地图显示不允许删除。", QSystemTrayIcon::Information, 1500);
    return;
  }

  viz->DeleteDisplayLayer(display);
  delete item;
}

void MainWindow::refreshDualMapDisplayItem(QTreeWidgetItem* item)
{
  if (item == nullptr) {
    return;
  }

  const QVariantMap properties = item->data(0, Qt::UserRole + 4).toMap();
  const QString displayName = properties.value("display_name", item->data(0, Qt::UserRole + 2).toString()).toString();
  const QString topicName = properties.value("topic", item->data(0, Qt::UserRole + 3).toString()).toString();
  const bool enabled = item->checkState(0) == Qt::Checked;
  QString statusText = enabled ? "OK" : "Disabled";

  const int layerCode = item->data(0, Qt::UserRole + 1).toInt();
  const bool requiresTopic = !(layerCode == AddLayer::grid || layerCode == AddLayer::tf || layerCode == AddLayer::robotmodel);
  if (enabled && requiresTopic && topicName.trimmed().isEmpty()) {
    statusText = "No Topic";
  }

  item->setText(0, displayName);
  item->setText(1, topicName);
  item->setText(2, statusText);
}

void MainWindow::updateDualMapDisplayDescription(QTreeWidget* treeWidget, QTextBrowser* descriptionBrowser)
{
  if (treeWidget == nullptr || descriptionBrowser == nullptr) {
    return;
  }

  QTreeWidgetItem* item = treeWidget->currentItem();
  if (item == nullptr) {
    descriptionBrowser->clear();
    return;
  }

  const int layerCode = item->data(0, Qt::UserRole + 1).toInt();
  const QVariantMap properties = item->data(0, Qt::UserRole + 4).toMap();
  const QString displayName = properties.value("display_name", item->data(0, Qt::UserRole + 2).toString()).toString();
  const QString topicName = properties.value("topic", item->data(0, Qt::UserRole + 3).toString()).toString();
  const QString enabledText = item->checkState(0) == Qt::Checked ? "启用" : "停用";

  QString typeName;
  QString description;
  switch (layerCode) {
    case AddLayer::grid:
      typeName = "Grid";
      description = "沿地平面显示参考坐标系网格。";
    break;
    case AddLayer::tf:
      typeName = "TF";
      description = "显示坐标变换层次结构。";
    break;
    case AddLayer::laserscan:
      typeName = "LaserScan";
      description = "显示 sensor_msgs/LaserScan 数据。";
    break;
    case AddLayer::pointcloud2:
      typeName = "PointCloud2";
      description = "显示 sensor_msgs/PointCloud2 数据。";
    break;
    case AddLayer::robotmodel:
      typeName = "RobotModel";
      description = "显示机器人模型。";
    break;
    case AddLayer::map:
      typeName = "Map";
      description = "显示 nav_msgs/OccupancyGrid 地图。";
    break;
    case AddLayer::path:
      typeName = "Path";
      description = "显示 nav_msgs/Path 路径。";
    break;
    case AddLayer::pointstamped:
      typeName = "PointStamped";
      description = "显示 geometry_msgs/PointStamped 点位。";
    break;
    case AddLayer::marker:
      typeName = "Marker";
      description = "显示 visualization_msgs/Marker。";
    break;
    case AddLayer::markerarray:
      typeName = "MarkerArray";
      description = "显示 visualization_msgs/MarkerArray。";
    break;
    default:
      typeName = "Unknown";
      description = "未知显示类型。";
    break;
  }

  QString text = QString("名称: %1\n类型: %2\n状态: %3").arg(displayName, typeName, enabledText);
  if (!topicName.isEmpty()) {
    text += QString("\n话题: %1").arg(topicName);
  }
  text += QString("\n\n%1").arg(description);
  descriptionBrowser->setText(text);
}

void MainWindow::editDualMapDisplay(qrviz* viz, QTreeWidgetItem* item)
{
  if (viz == nullptr || item == nullptr) {
    return;
  }

  rviz_common::Display* display = reinterpret_cast<rviz_common::Display*>(static_cast<quintptr>(item->data(0, Qt::UserRole).toULongLong()));
  if (display == nullptr) {
    return;
  }

  const int layerCode = item->data(0, Qt::UserRole + 1).toInt();
  QVariantMap properties = item->data(0, Qt::UserRole + 4).toMap();

  QDialog dialog(this);
  dialog.setWindowTitle("编辑显示属性");
  QVBoxLayout* layout = new QVBoxLayout(&dialog);
  QFormLayout* form = new QFormLayout();
  layout->addLayout(form);

  QLineEdit* nameEdit = new QLineEdit(properties.value("display_name").toString(), &dialog);
  form->addRow("名称", nameEdit);

  QCheckBox* enabledCheck = new QCheckBox(&dialog);
  enabledCheck->setChecked(properties.value("enabled", true).toBool());
  form->addRow("启用", enabledCheck);

  QLineEdit* topicEdit = nullptr;
  if (layerCode != AddLayer::grid && layerCode != AddLayer::tf && layerCode != AddLayer::robotmodel) {
    topicEdit = new QLineEdit(properties.value("topic").toString(), &dialog);
    form->addRow("Topic", topicEdit);
  }

  QSpinBox* cellCountSpin = nullptr;
  QPushButton* colorButton = nullptr;
  QColor selectedColor = properties.value("color", QColor(160, 160, 164)).value<QColor>();
  QColor selectedPathColor = properties.value("path_color", QColor(25, 255, 0)).value<QColor>();
  QColor selectedPointColor = properties.value("point_color", QColor(255, 0, 0)).value<QColor>();
  QComboBox* styleCombo = nullptr;
  QDoubleSpinBox* pointSizeSpin = nullptr;
  QComboBox* colorSchemeCombo = nullptr;

  auto bindColorButton = [&](const QString& label, QColor& targetColor) {
    QPushButton* button = new QPushButton(targetColor.name(), &dialog);
    QObject::connect(button, &QPushButton::clicked, &dialog, [&dialog, &targetColor, button](){
      QColor color = QColorDialog::getColor(targetColor, &dialog, "选择颜色");
      if (color.isValid()) {
        targetColor = color;
        button->setText(color.name());
      }
    });
    form->addRow(label, button);
    return button;
  };

  switch (layerCode) {
    case AddLayer::grid:
      cellCountSpin = new QSpinBox(&dialog);
      cellCountSpin->setRange(1, 500);
      cellCountSpin->setValue(properties.value("cell_count", 30).toInt());
      form->addRow("Cell Count", cellCountSpin);
      colorButton = bindColorButton("Color", selectedColor);
    break;
    case AddLayer::laserscan:
    case AddLayer::pointcloud2:
      styleCombo = new QComboBox(&dialog);
      styleCombo->addItems(QStringList() << "Points" << "Squares" << "Flat Squares" << "Spheres" << "Boxes" << "Tiles");
      styleCombo->setCurrentText(properties.value("style", "Points").toString());
      form->addRow("Style", styleCombo);
      pointSizeSpin = new QDoubleSpinBox(&dialog);
      pointSizeSpin->setRange(0.01, 100.0);
      pointSizeSpin->setDecimals(2);
      pointSizeSpin->setValue(properties.value("point_size", 3.0).toDouble());
      form->addRow("Point Size", pointSizeSpin);
    break;
    case AddLayer::map:
      colorSchemeCombo = new QComboBox(&dialog);
      colorSchemeCombo->addItems(QStringList() << "map" << "costmap" << "raw");
      colorSchemeCombo->setCurrentText(properties.value("color_scheme", "map").toString());
      form->addRow("Color Scheme", colorSchemeCombo);
    break;
    case AddLayer::path:
      colorButton = bindColorButton("Color", selectedPathColor);
    break;
    case AddLayer::pointstamped:
      colorButton = bindColorButton("Color", selectedPointColor);
    break;
    default:
    break;
  }

  QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
  layout->addWidget(buttonBox);
  QObject::connect(buttonBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  QObject::connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

  if (dialog.exec() != QDialog::Accepted) {
    return;
  }

  properties.insert("display_name", nameEdit->text().trimmed().isEmpty() ? properties.value("display_name").toString() : nameEdit->text().trimmed());
  properties.insert("enabled", enabledCheck->isChecked());
  if (topicEdit != nullptr) {
    properties.insert("topic", topicEdit->text().trimmed());
  }
  if (cellCountSpin != nullptr) {
    properties.insert("cell_count", cellCountSpin->value());
    properties.insert("color", selectedColor);
  }
  if (styleCombo != nullptr) {
    properties.insert("style", styleCombo->currentText());
  }
  if (pointSizeSpin != nullptr) {
    properties.insert("point_size", pointSizeSpin->value());
  }
  if (colorSchemeCombo != nullptr) {
    properties.insert("color_scheme", colorSchemeCombo->currentText());
  }
  if (layerCode == AddLayer::path) {
    properties.insert("path_color", selectedPathColor);
  }
  if (layerCode == AddLayer::pointstamped) {
    properties.insert("point_color", selectedPointColor);
  }

  item->setCheckState(0, properties.value("enabled", true).toBool() ? Qt::Checked : Qt::Unchecked);
  item->setData(0, Qt::UserRole + 2, properties.value("display_name").toString());
  item->setData(0, Qt::UserRole + 3, properties.value("topic").toString());
  item->setData(0, Qt::UserRole + 4, properties);
  applyDualMapDisplayProperties(viz, display, layerCode, properties);
  refreshDualMapDisplayItem(item);

  if (item->treeWidget() == ui.listWidgetCarDisplays) {
    if (display == carMapDisplay) {
      ui.lineEditCarMapTopic->setText(properties.value("topic").toString());
    }
    updateDualMapDisplayDescription(ui.listWidgetCarDisplays, ui.textBrowserCarDisplayDescription);
  } else if (item->treeWidget() == ui.listWidgetDroneDisplays) {
    if (display == droneMapDisplay) {
      ui.lineEditDroneMapTopic->setText(properties.value("topic").toString());
    }
    updateDualMapDisplayDescription(ui.listWidgetDroneDisplays, ui.textBrowserDroneDisplayDescription);
  }
}

void MainWindow::showMainMapPage()
{
  ui.stackedWidget->setCurrentWidget(ui.Map);
  ui.docktreeWidget->setHidden(false);
  ui.dockTaskManagerWidget->setHidden(false);
  ui.rViztoolBar->setHidden(false);
}

void MainWindow::showDualMapPage()
{
  ui.stackedWidget->setCurrentWidget(ui.DualMapView);
  ui.docktreeWidget->setHidden(true);
  ui.dockTaskManagerWidget->setHidden(true);
  ui.rViztoolBar->setHidden(true);
}

void MainWindow::initializeAerialPhotoPage()
{
  static const QString kAerialMissionVideoPath =
    QString::fromUtf8(u8"/mnt/hgfs/建图与导航/sucai/6月20日.mp4");

  aerialMapView = new QWebEngineView(this);
  ui.verticalLayout_aerialMap->replaceWidget(ui.graphicsViewAerialMap, aerialMapView);
  ui.graphicsViewAerialMap->hide();
  aerialMapView->setUrl(QUrl("qrc:/aerial_satellite_map.html"));

  fpvOverlayPanel = new QFrame(ui.frameAerialMap);
  fpvOverlayPanel->setObjectName("fpvOverlayPanel");
  fpvOverlayPanel->setStyleSheet(
    "#fpvOverlayPanel {"
    "background-color: rgba(17, 24, 39, 210);"
    "border: 1px solid rgba(255, 255, 255, 70);"
    "border-radius: 10px;"
    "}"
  );
  QVBoxLayout* overlayLayout = new QVBoxLayout(fpvOverlayPanel);
  overlayLayout->setContentsMargins(10, 10, 10, 10);
  overlayLayout->setSpacing(6);

  QHBoxLayout* fpvHeaderLayout = new QHBoxLayout();
  QLabel* fpvTitleLabel = new QLabel("FPV 实时画面", fpvOverlayPanel);
  fpvTitleLabel->setStyleSheet("color: white; font-weight: 600;");
  fpvHeaderLayout->addWidget(fpvTitleLabel);
  fpvHeaderLayout->addStretch();
  overlayLayout->addLayout(fpvHeaderLayout);

  if (fpvRtspUrlEdit != nullptr) {
    fpvRtspUrlEdit->hide();
  }

  fpvVideoStack = new QStackedWidget(fpvOverlayPanel);
  fpvVideoStack->setMinimumHeight(300);
  fpvVideoStack->setStyleSheet(
    "background-color: #05070b;"
    "border-radius: 8px;"
    "border: 1px solid rgba(255, 255, 255, 0.12);"
  );

  fpvVideoPlaceholder = new QLabel("FPV 预览区", fpvOverlayPanel);
  fpvVideoPlaceholder->setMinimumHeight(300);
  fpvVideoPlaceholder->setAlignment(Qt::AlignCenter);
  fpvVideoPlaceholder->setStyleSheet(
    "color: rgba(255, 255, 255, 0.72);"
    "background-color: transparent;"
    "border: none;"
  );
  fpvVideoStack->addWidget(fpvVideoPlaceholder);
  fpvVideoStack->setCurrentWidget(fpvVideoPlaceholder);
  overlayLayout->addWidget(fpvVideoStack, 1);
  fpvVideoTimer = new QTimer(fpvOverlayPanel);
  connect(fpvVideoTimer, &QTimer::timeout, this, [=](){
    if (!fpvVideoCapture.isOpened()) {
      return;
    }

    cv::Mat frame;
    if (!fpvVideoCapture.read(frame) || frame.empty()) {
      if (fpvVideoTimer != nullptr) {
        fpvVideoTimer->stop();
      }
      fpvVideoCapture.release();
      if (fpvStatusLabel != nullptr) {
        fpvStatusLabel->setText("已完成正射影像拍摄");
      }
      ui.textBrowserAerialStatus->setText("已完成正射影像拍摄");
      if (fpvOverlayPanel != nullptr) {
        fpvOverlayPanel->raise();
      }
      return;
    }

    updateVideoLabel(fpvVideoPlaceholder, frame);
  });

  fpvStatusLabel = new QLabel("FPV 未连接", fpvOverlayPanel);
  fpvStatusLabel->setStyleSheet("color: rgba(255, 255, 255, 0.88);");
  overlayLayout->addWidget(fpvStatusLabel);
  fpvOverlayPanel->hide();
  updateFpvOverlayGeometry();

  ui.textBrowserAerialStatus->setText("正在加载在线卫星地图...");

  connect(aerialMapView, &QWebEngineView::loadFinished, this, [=](bool ok){
    if (ok) {
      ui.textBrowserAerialStatus->setText("在线卫星地图已加载。\n\n1. 点击“选择拍照范围”并在地图上绘制区域。\n2. 点击“设置面状航线”生成面状航线。\n3. 如需重新规划，点击“重置并重画”。\n4. 调整拍照参数。\n5. 点击“开始拍照”生成拍照点。");
    } else {
      ui.textBrowserAerialStatus->setText("在线卫星地图加载失败。\n请检查 Qt WebEngine 组件、网络连接和外部地图服务可访问性。");
    }
  });

  connect(ui.buttonSelectPhotoArea, &QPushButton::toggled, this, [=](bool checked){
    if (!checked || aerialMapView == nullptr) {
      return;
    }
    ui.buttonSetPolygonRoute->setChecked(false);
    aerialMapView->page()->runJavaScript("startAreaSelection();", [=](const QVariant& result){
      ui.textBrowserAerialStatus->setText(result.toString());
    });
  });

  connect(ui.buttonResetAerialPlan, &QPushButton::clicked, this, [=](){
    ui.buttonSelectPhotoArea->setChecked(false);
    ui.buttonSetPolygonRoute->setChecked(false);
    if (aerialMapView == nullptr) {
      return;
    }
    aerialMapView->page()->runJavaScript("resetAerialPlanning();", [=](const QVariant& result){
      ui.textBrowserAerialStatus->setText(result.toString());
    });
  });

  connect(ui.buttonSetPolygonRoute, &QPushButton::toggled, this, [=](bool checked){
    if (!checked || aerialMapView == nullptr) {
      return;
    }
    ui.buttonSelectPhotoArea->setChecked(false);
    QString script = QString("setPolygonRoute(%1);")
                       .arg(ui.doubleSpinBoxRouteSpacing->value(), 0, 'f', 1);
    aerialMapView->page()->runJavaScript(script, [=](const QVariant& result){
      ui.textBrowserAerialStatus->setText(result.toString());
    });
  });

  connect(ui.buttonStartPhotoMission, &QPushButton::clicked, this, [=](){
    if (aerialMapView == nullptr) {
      return;
    }
    QString script = QString(
      "startPhotoMission(%1, %2, %3, %4);")
      .arg(ui.doubleSpinBoxPhotoInterval->value(), 0, 'f', 1)
      .arg(ui.doubleSpinBoxRouteSpacing->value(), 0, 'f', 1)
      .arg(ui.doubleSpinBoxFlightHeight->value(), 0, 'f', 1)
      .arg(ui.spinBoxOverlapRate->value());
    aerialMapView->page()->runJavaScript(script, [=](const QVariant& result){
      Q_UNUSED(result);
      ui.textBrowserAerialStatus->setText("正在进行正射影像拍摄");
      playLocalFpvVideo(kAerialMissionVideoPath);
    });
  });

  connect(ui.buttonToggleFpvOverlay, &QPushButton::toggled, this, [=](bool checked){
    if (fpvOverlayPanel == nullptr) {
      return;
    }
    fpvOverlayPanel->setVisible(checked);
    ui.buttonToggleFpvOverlay->setText(checked ? "关闭 FPV" : "开启 FPV");
    if (checked) {
      updateFpvOverlayGeometry();
      fpvOverlayPanel->raise();
    } else {
      stopFpvStream();
    }
  });
}

void MainWindow::updateFpvOverlayGeometry()
{
  if (fpvOverlayPanel == nullptr || ui.frameAerialMap == nullptr) {
    return;
  }

  const int overlayWidth = qMin(720, qMax(460, ui.frameAerialMap->width() / 2));
  const int overlayHeight = qMin(520, qMax(320, ui.frameAerialMap->height() / 2));
  const int margin = 16;
  fpvOverlayPanel->setGeometry(
    ui.frameAerialMap->width() - overlayWidth - margin,
    ui.frameAerialMap->height() - overlayHeight - margin,
    overlayWidth,
    overlayHeight
  );
}

void MainWindow::startFpvStream()
{
  if (!fpvCurrentVideoPath.isEmpty()) {
    playLocalFpvVideo(fpvCurrentVideoPath);
  }
}

void MainWindow::stopFpvStream()
{
  if (fpvVideoTimer != nullptr) {
    fpvVideoTimer->stop();
  }
  if (fpvVideoCapture.isOpened()) {
    fpvVideoCapture.release();
  }
  fpvCurrentVideoPath.clear();
  if (fpvVideoStack != nullptr && fpvVideoPlaceholder != nullptr) {
    fpvVideoPlaceholder->setText("FPV 预览区");
    fpvVideoPlaceholder->setPixmap(QPixmap());
    fpvVideoStack->setCurrentWidget(fpvVideoPlaceholder);
  }
  if (fpvStatusLabel != nullptr) {
    fpvStatusLabel->setText("FPV 已停止");
  }
}

void MainWindow::playLocalFpvVideo(const QString& filePath)
{
  if (fpvVideoStack == nullptr || fpvVideoPlaceholder == nullptr) {
    return;
  }

  QFileInfo videoInfo(filePath);
  if (!videoInfo.exists() || !videoInfo.isFile()) {
    if (fpvVideoStack != nullptr && fpvVideoPlaceholder != nullptr) {
      fpvVideoPlaceholder->setText("视频文件不存在");
      fpvVideoPlaceholder->setPixmap(QPixmap());
      fpvVideoStack->setCurrentWidget(fpvVideoPlaceholder);
    }
    if (fpvStatusLabel != nullptr) {
      fpvStatusLabel->setText("FPV 视频打开失败");
    }
    return;
  }

  if (fpvOverlayPanel != nullptr && !fpvOverlayPanel->isVisible()) {
    fpvOverlayPanel->show();
    updateFpvOverlayGeometry();
    fpvOverlayPanel->raise();
    ui.buttonToggleFpvOverlay->setChecked(true);
  }

  if (fpvVideoCapture.isOpened()) {
    fpvVideoCapture.release();
  }
  if (!fpvVideoCapture.open(videoInfo.absoluteFilePath().toStdString())) {
    fpvVideoPlaceholder->setText("无法打开视频");
    fpvVideoPlaceholder->setPixmap(QPixmap());
    fpvVideoStack->setCurrentWidget(fpvVideoPlaceholder);
    if (fpvStatusLabel != nullptr) {
      fpvStatusLabel->setText("FPV 视频打开失败");
    }
    return;
  }

  cv::Mat firstFrame;
  if (!fpvVideoCapture.read(firstFrame) || firstFrame.empty()) {
    fpvVideoCapture.release();
    fpvVideoPlaceholder->setText("视频首帧读取失败");
    fpvVideoPlaceholder->setPixmap(QPixmap());
    fpvVideoStack->setCurrentWidget(fpvVideoPlaceholder);
    if (fpvStatusLabel != nullptr) {
      fpvStatusLabel->setText("FPV 视频读取失败");
    }
    return;
  }

  fpvCurrentVideoPath = videoInfo.absoluteFilePath();
  fpvVideoPlaceholder->setText(QString());
  updateVideoLabel(fpvVideoPlaceholder, firstFrame);
  fpvVideoStack->setCurrentWidget(fpvVideoPlaceholder);
  const double fps = fpvVideoCapture.get(cv::CAP_PROP_FPS);
  const int intervalMs = fps > 1.0 ? qMax(15, qRound(1000.0 / fps)) : 33;
  if (fpvVideoTimer != nullptr) {
    fpvVideoTimer->start(intervalMs);
  }
  if (fpvStatusLabel != nullptr) {
    fpvStatusLabel->setText("正在进行正射影像拍摄");
  }
}



/**
 * @brief 显示图层描述槽函数
 */
void MainWindow::slot_display_layer_description()
{
  if(ui.treeWidget_layer->currentItem() == Global){
    ui.textBrowser_layerDescription->setText("全局设置");
  }
  else if(ui.treeWidget_layer->currentItem()->parent() == nullptr){
    switch(ui.treeWidget_layer->currentItem()->data(0, Qt::UserRole).value<int>())
    {
      case AddLayer::grid:
        ui.textBrowser_layerDescription->setText("网格\n\n\t沿地平面显示以目标参考系原点为中心的网格。");
      break;
      case AddLayer::tf:
        ui.textBrowser_layerDescription->setText("坐标变换\n\n\t显示坐标转换层次结构。");
      break;
      case AddLayer::laserscan:
        ui.textBrowser_layerDescription->setText("激光扫描\n\n\t将来自sensor_msgs::msg::LaserScan消息的数据显示为世界中的点，以点、公告板或立方体的形式绘制。");
      break;
      case AddLayer::pointcloud2:
        ui.textBrowser_layerDescription->setText("点云2\n\n\t将来自sensor_msgs::PointCloud2消息的数据显示为世界中的点，以点、公告板或立方体的形式绘制。");
      break;
      case AddLayer::robotmodel:
        ui.textBrowser_layerDescription->setText("机器人模型\n\n\t以正确的姿态(由当前坐标变换定义)显示机器人的视觉表示。");
      break;
      case AddLayer::map:
        ui.textBrowser_layerDescription->setText("地图\n\n\t将来自nav_msgs::OccupancyGrid消息的数据显示为一个地平面的占用网格。");
      break;
      case AddLayer::path:
        ui.textBrowser_layerDescription->setText("路径\n\n\t以线的形式显示nav_msgs::msg::Path消息中的数据。");
      break;
      case AddLayer::pointstamped:
        ui.textBrowser_layerDescription->setText("带点戳\n\n\t显示来自geometry_msgs/PointStamped消息的信息。");
      break;
      case AddLayer::marker:
        ui.textBrowser_layerDescription->setText("标记\n\n\t显示visualization_msgs::msg::Marker消息。");
      break;
      case AddLayer::markerarray:
        ui.textBrowser_layerDescription->setText("标记数组\n\n\t在不假定话题名称以“_array”结尾的情况下显示visualization_msgs::msg::MarkerArray消息。");
      break;
    }
  }
  else { ui.textBrowser_layerDescription->clear(); }
}



/**
 * @brief 添加图层按钮点击槽函数
 */
void MainWindow::on_button_add_layer_clicked()
{
  //打开新增图层对话框并获得其返回值
  int addLayerCode = AddLayer::getLayer(this);
  //获取图层显示名
  QString displayName = AddLayer::getDisplayName();
  //获取话题名
  QString topicName = AddLayer::getTopicName();
  switch(addLayerCode){
    case AddLayer::null:
    break;
    case AddLayer::grid:
      add_TreeWidgetItem_Grid(ui.treeWidget_layer, displayName);
    break;
    case AddLayer::tf:
      add_TreeWidgetItem_TF(ui.treeWidget_layer, displayName);
    break;
    case AddLayer::laserscan:
      add_TreeWidgetItem_LaserScan(ui.treeWidget_layer, displayName, topicName);
    break;
    case AddLayer::pointcloud2:
      add_TreeWidgetItem_PointCloud2(ui.treeWidget_layer, displayName, topicName);
    break;
    case AddLayer::robotmodel:
      add_TreeWidgetItem_RobotModel(ui.treeWidget_layer, displayName);
    break;
    case AddLayer::map:
      add_TreeWidgetItem_Map(ui.treeWidget_layer, displayName, topicName);
    break;
    case AddLayer::path:
      add_TreeWidgetItem_Path(ui.treeWidget_layer, displayName, topicName);
    break;
    case AddLayer::pointstamped:
      add_TreeWidgetItem_PointStamped(ui.treeWidget_layer, displayName, topicName);
    break;
    case AddLayer::marker:
      add_TreeWidgetItem_Marker(ui.treeWidget_layer, displayName, topicName);
    break;
    case AddLayer::markerarray:
      add_TreeWidgetItem_MarkerArray(ui.treeWidget_layer, displayName, topicName);
    break;
  }
}



/**
 * @brief 设置rviz树item激活槽函数
 */
void MainWindow::slot_enable_remove_button(QTreeWidgetItem* current)
{
  if(current->parent() == nullptr && current != Global)
    ui.button_remove_layer->setEnabled(true);
  else
    ui.button_remove_layer->setEnabled(false);
}



/**
 * @brief 移除图层按钮点击槽函数
 */
void MainWindow::on_button_remove_layer_clicked()
{
  //查询treewidgetitem对象存储的“用户数据”，以确定待移除的对象属于哪个图层
  switch(ui.treeWidget_layer->currentItem()->data(0, Qt::UserRole).value<int>()){
    case AddLayer::grid:
      remove_TreeWidgetItem_Grid(ui.treeWidget_layer, ui.treeWidget_layer->currentItem());
    break;
    case AddLayer::tf:
      remove_TreeWidgetItem_TF(ui.treeWidget_layer, ui.treeWidget_layer->currentItem());
    break;
    case AddLayer::laserscan:
      remove_TreeWidgetItem_LaserScan(ui.treeWidget_layer, ui.treeWidget_layer->currentItem());
    break;
    case AddLayer::pointcloud2:
      remove_TreeWidgetItem_PointCloud2(ui.treeWidget_layer, ui.treeWidget_layer->currentItem());
    break;
    case AddLayer::robotmodel:
      remove_TreeWidgetItem_RobotModel(ui.treeWidget_layer, ui.treeWidget_layer->currentItem());
    break;
    case AddLayer::map:
      remove_TreeWidgetItem_Map(ui.treeWidget_layer, ui.treeWidget_layer->currentItem());
    break;
    case AddLayer::path:
      remove_TreeWidgetItem_Path(ui.treeWidget_layer, ui.treeWidget_layer->currentItem());
    break;
    case AddLayer::pointstamped:
      remove_TreeWidgetItem_PointStamped(ui.treeWidget_layer, ui.treeWidget_layer->currentItem());
    break;
    case AddLayer::marker:
      remove_TreeWidgetItem_Marker(ui.treeWidget_layer, ui.treeWidget_layer->currentItem());
    break;
    case AddLayer::markerarray:
      remove_TreeWidgetItem_MarkerArray(ui.treeWidget_layer, ui.treeWidget_layer->currentItem());
    break;
  }
}



/**
 * @brief 设置全局设置槽函数
 */
void MainWindow::slot_set_global_options()
{
  myviz->SetFixedFrame(FFCBox->currentText());                //设置固定坐标系
  myviz->SetPanelBackgroundColor(BPalette->currentColor());   //设置背景颜色
}



/***********************************************************************************************
 *                                向UI界面添加新图层相关控件方法定义                                 *
 ***********************************************************************************************/
/**
 * @brief 向QTreeidget添加Grid顶层节点
 * @param QTreeWidget* treeWidget 待添加Grid顶层节点的QTreeWidget指针
 */
QTreeWidgetItem* MainWindow::add_TreeWidgetItem_Grid(QTreeWidget* treeWidget, QString& displayName)
{
  QTreeWidgetItem* Grid = new QTreeWidgetItem(QStringList() << displayName);
  //设置图标
  Grid->setIcon(0, QIcon("://rViz/Grid.png"));
  //初始化一个checkbox
  QCheckBox* checkbox = new QCheckBox();
  //设置复选框默认选中
  checkbox->setCheckState(Qt::Checked);
  //加载根节点
  treeWidget->addTopLevelItem(Grid);
  //设置Grid树默认展开(默认展开应写在加载根节点后)
  Grid->setExpanded(true);
  //添加checkbox于根节点第一列
  treeWidget->setItemWidget(Grid, 1, checkbox);
  //连接checkbox状态改变信号与修改网格图层属性槽函数
  connect(checkbox, &QCheckBox::stateChanged, this, &MainWindow::slot_modify_grid_attribute);
  //设置用户数据
  Grid->setData(0, Qt::UserRole, AddLayer::grid);

  /*
   * 定义子节点1 ———— plane cell count
   */
  QTreeWidgetItem* PlaneCellCount = new QTreeWidgetItem(QStringList() << "平面单元格数");
  //添加子节点1
  Grid->addChild(PlaneCellCount);
  //初始化一个spinbox
  QSpinBox* spinbox = new QSpinBox();
  spinbox->setMinimumWidth(150);                              //设置最小宽度
  spinbox->setRange(1,100);                                   //设置可调节范围
  spinbox->setValue(10);                                      //设置初始值
  //把spinbox添加到子节点1的第一列
  treeWidget->setItemWidget(PlaneCellCount, 1, spinbox);
  //定义函数指针(因为QSpinBox::valueChanged存在重载，不使用函数指针会产生调用不明确的错误)
  void (QSpinBox:: *signalpointer1)(int) = &QSpinBox::valueChanged;
  //连接spinbox的值改变信号与修改网格图层属性槽函数
  connect(spinbox, signalpointer1, this, &MainWindow::slot_modify_grid_attribute);

  /*
   * 定义子节点2 ———— color
   */
  QTreeWidgetItem* GridColor = new QTreeWidgetItem(QStringList() << "颜色");
  //添加子节点2
  Grid->addChild(GridColor);
  //初始化一个MyPalette对象
  MyPalette* palette = new MyPalette();
  //设置初始颜色
  palette->setColor(160, 160, 164);
  //把mypalette添加到子节点2的第一列
  treeWidget->setItemWidget(GridColor, 1, palette);
  //连接mypalette颜色改变信号与修改网格图层属性槽函数
  connect(palette, &MyPalette::colorchanged, this, &MainWindow::slot_modify_grid_attribute);

  //调用qrviz类的接口创建图层对象实体并取得其指针
  rviz_common::Display* display = myviz->CreateDisplay("rviz_default_plugins/Grid", "myGrid");
  //向Hash表中插入键值对
  m_LayerHash.insert(Grid, display);
  //默认调用一次修改图层属性函数
  myviz->ModifyGridAttribute(display, spinbox->value(), palette->currentColor(), checkbox->isChecked());

  return Grid;
}



/**
 * @brief 向QTreeidget添加TF顶层节点
 * @param QTreeWidget* treeWidget 待添加TF顶层节点的QTreeWidget指针
 */
QTreeWidgetItem* MainWindow::add_TreeWidgetItem_TF(QTreeWidget* treeWidget, QString& displayName)
{
  QTreeWidgetItem* TF = new QTreeWidgetItem(QStringList() << displayName);
  //设置图标
  TF->setIcon(0,QIcon("://rViz/TF.png"));
  //初始化一个checkbox
  QCheckBox* checkbox = new QCheckBox();
  //设置复选框默认选中
  checkbox->setCheckState(Qt::Checked);
  //连接checkbox状态改变信号与修改TF图层属性槽函数
  connect(checkbox, &QCheckBox::stateChanged, this, &MainWindow::slot_modify_TF_attribute);
  //加载根节点
  treeWidget->addTopLevelItem(TF);
  //添加checkbox于根节点第一列
  treeWidget->setItemWidget(TF, 1, checkbox);
  //设置用户数据
  TF->setData(0, Qt::UserRole, AddLayer::tf);

  //调用qrviz类的接口创建图层对象实体并取得其指针
  rviz_common::Display* display = myviz->CreateDisplay("rviz_default_plugins/TF", "myTF");
  //向Hash表中插入键值对
  m_LayerHash.insert(TF, display);
  //默认调用一次修改图层属性函数
  myviz->ModifyTFAttribute(display, checkbox->isChecked());
  return TF;
}



/**
 * @brief 向QTreeidget添加LaserScan顶层节点
 * @param QTreeWidget* treeWidget 待添加LaserScan顶层节点的QTreeWidget指针
 */
QTreeWidgetItem* MainWindow::add_TreeWidgetItem_LaserScan(QTreeWidget* treeWidget, QString& displayName, QString& topicName)
{
  QTreeWidgetItem* LaserScan = new QTreeWidgetItem(QStringList() << displayName);
  //设置图标
  LaserScan->setIcon(0, QIcon("://rViz/LaserScan.png"));
  //初始化一个checkbox
  QCheckBox* checkbox = new QCheckBox();
  //设置复选框默认选中状态
  checkbox->setCheckState(Qt::Checked);
  //连接checkbox状态改变信号与修改激光扫描图层属性槽函数
  connect(checkbox, &QCheckBox::stateChanged, this, &MainWindow::slot_modify_LaserScan_attribute);
  //加载根节点
  treeWidget->addTopLevelItem(LaserScan);
  //添加checkbox于根节点第一列
  treeWidget->setItemWidget(LaserScan, 1, checkbox);
  //设置默认展开(默认展开应写在加载根节点后)
  //LaserScan->setExpanded(true);
  //设置用户数据
  LaserScan->setData(0, Qt::UserRole, AddLayer::laserscan);

  /*
   * 定义子节点1 ———— topic
   */
  QTreeWidgetItem* LaserTopic = new QTreeWidgetItem(QStringList() << "话题");
  //添加子节点1
  LaserScan->addChild(LaserTopic);
  //初始化一个combobox
  QComboBox* combobox = new QComboBox();
  combobox->addItems(AddLayer::getTopicListByLayerCode(AddLayer::laserscan));                           //添加项
  combobox->setCurrentIndex(-1);                                                                        //设置默认不显示项
  combobox->setCurrentText(topicName);                                                                  //设置项(需要注意的是，如果topicName不匹配任何一个已经添加好的项，那么setCurrentText()方法将不起作用)
  combobox->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLength);                              //设置尺寸调整策略为适应最小内容
  combobox->setEditable(true);                                                                          //设置可编辑
  //添加combobox于子节点1的第一列
  treeWidget->setItemWidget(LaserTopic, 1, combobox);
  //连接combobox当前文本改变信号与修改激光扫描图层属性槽函数
  connect(combobox, &QComboBox::currentTextChanged, this, &MainWindow::slot_modify_LaserScan_attribute);

  /*
   * 定义子节点2 ———— style
   */
  QTreeWidgetItem* LaserRenderStyle = new QTreeWidgetItem(QStringList() << "渲染模式");
  //添加子节点2
  LaserScan->addChild(LaserRenderStyle);
  //初始化一个combobox
  QComboBox* StyleComboBox = new QComboBox();
  StyleComboBox->addItems(QStringList() << "Points" << "Squares" << "Flat Squares" << "Spheres" << "Boxes");  //添加项
  StyleComboBox->setEditable(false);                                                                          //设置不可编辑
  StyleComboBox->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLength);                               //设置尺寸调整策略为适应最小内容
  //添加combobox于子节点2的第一列
  treeWidget->setItemWidget(LaserRenderStyle, 1, StyleComboBox);
  //连接combobox当前文本改变信号与修改激光扫描图层属性槽函数
  connect(StyleComboBox, &QComboBox::currentTextChanged, this, &MainWindow::slot_modify_LaserScan_attribute);
  //连接combobox当前文本改变信号与激光点云图层渲染风格改变槽函数
  connect(StyleComboBox, &QComboBox::currentTextChanged, this, &MainWindow::slot_onStyleComboBox_text_changed);

  /*
   * 定义子节点3 ———— size
   */
  QTreeWidgetItem* LaserPointSize = new QTreeWidgetItem(QStringList() << "大小 (Pixels)");
  //添加子节点3
  LaserScan->addChild(LaserPointSize);
  //初始化一个lineedit
  QLineEdit* PointSizeLineEdit = new QLineEdit("3");
  //正则表达式限制只能输入正浮点数
  PointSizeLineEdit->setValidator(new QRegExpValidator(QRegExp("^(([0-9]+\\.[0-9]*[1-9][0-9]*)|([0-9]*[1-9][0-9]*\\.[0-9]+)|([0-9]*[1-9][0-9]*)|0)$")));
  //设置最小宽度
  PointSizeLineEdit->setMinimumWidth(150);
  //添加lineedit于子节点3第一列
  treeWidget->setItemWidget(LaserPointSize, 1, PointSizeLineEdit);
  //连接lineedit文本改变信号与修改激光扫描图层属性槽函数
  connect(PointSizeLineEdit, &QLineEdit::textChanged, this, &MainWindow::slot_modify_LaserScan_attribute);
  //连接lineedit编辑结束信号与数据处理槽函数
  connect(PointSizeLineEdit, &QLineEdit::editingFinished, this, &MainWindow::slot_onPointSizeLineEdit_edit_finished);

  //调用qrviz类的接口创建图层对象实体并取得其指针
  rviz_common::Display* display = myviz->CreateDisplay("rviz_default_plugins/LaserScan", "myLaserScan");
  //向Hash表中插入键值对
  m_LayerHash.insert(LaserScan, display);
  //默认调用一次修改图层属性函数
  myviz->ModifyLaserScanAttribute(display, combobox->currentText(), StyleComboBox->currentText(), PointSizeLineEdit->text().toDouble(), checkbox->isChecked());

  return LaserScan;
}


/**
 * @brief 向QTreeidget添加PointCloud2顶层节点
 * @param QTreeWidget* treeWidget 待添加PointCloud2顶层节点的QTreeWidget指针
 */
QTreeWidgetItem* MainWindow::add_TreeWidgetItem_PointCloud2(QTreeWidget* treeWidget, QString& displayName, QString& topicName)
{
  QTreeWidgetItem* PointCloud2 = new QTreeWidgetItem(QStringList() << displayName);
  //设置图标
  PointCloud2->setIcon(0, QIcon("://rViz/PointCloud2.png"));
  //初始化一个checkbox
  QCheckBox* checkBox = new QCheckBox();
  //设置复选框默认选中状态
  checkBox->setCheckState(Qt::Checked);
  //连接checkbox状态改变信号与修改点云2图层属性槽函数
  connect(checkBox, &QCheckBox::stateChanged, this, &MainWindow::slot_modify_PointCloud2_attribute);
  //加载根节点
  treeWidget->addTopLevelItem(PointCloud2);
  //添加checkbox于根节点第一列
  treeWidget->setItemWidget(PointCloud2, 1, checkBox);
  //设置默认展开(默认展开应写在加载根节点后)
  //PointCloud2->setExpanded(true);
  //设置用户数据
  PointCloud2->setData(0, Qt::UserRole, AddLayer::pointcloud2);

  /*
   * 定义子节点1 ———— topic
   */
  QTreeWidgetItem* PointCloud2Topic = new QTreeWidgetItem(QStringList() << "话题");
  //添加子节点1
  PointCloud2->addChild(PointCloud2Topic);
  //初始化一个combobox
  QComboBox* combobox = new QComboBox();
  combobox->addItems(AddLayer::getTopicListByLayerCode(AddLayer::pointcloud2)); //添加项
  combobox->setCurrentIndex(-1);                                              //设置默认不显示项
  combobox->setCurrentText(topicName);                                        //设置项(需要注意的是，如果topicName不匹配任何一个已经添加好的项，那么setCurrentText()方法将不起作用)
  combobox->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLength);    //设置尺寸调整策略为适应最小内容
  combobox->setEditable(true);                                                //设置可编辑
  //添加combobox于子节点1的第一列
  treeWidget->setItemWidget(PointCloud2Topic, 1, combobox);
  //连接combobox当前文本改变信号与修改点云2图层属性槽函数
  connect(combobox, &QComboBox::currentTextChanged, this, &MainWindow::slot_modify_PointCloud2_attribute);

  /*
   * 定义子节点2 ———— style
   */
  QTreeWidgetItem* PointCloud2RenderStyle = new QTreeWidgetItem(QStringList() << "渲染模式");
  //添加子节点2
  PointCloud2->addChild(PointCloud2RenderStyle);
  //初始化一个combobox
  QComboBox* StyleComboBox = new QComboBox();
  StyleComboBox->addItems(QStringList() << "Points" << "Squares" << "Flat Squares" << "Spheres" << "Boxes");   //添加项
  StyleComboBox->setEditable(false);                                                                           //设置不可编辑
  StyleComboBox->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLength);                                //设置尺寸调整策略为适应最小内容
  //添加combobox于子节点2的第一列
  treeWidget->setItemWidget(PointCloud2RenderStyle, 1, StyleComboBox);
  //连接combobox当前文本改变信号与修改点云2图层属性槽函数
  connect(StyleComboBox, &QComboBox::currentTextChanged, this, &MainWindow::slot_modify_PointCloud2_attribute);
  //连接combobox当前文本改变信号与点云2图层渲染风格改变槽函数
  connect(StyleComboBox, &QComboBox::currentTextChanged, this, &MainWindow::slot_onStyleComboBox_text_changed);

  /*
   * 定义子节点3 ———— size
   */
  QTreeWidgetItem* PointCloud2PointSize = new QTreeWidgetItem(QStringList() << "大小 (Pixels)");
  //添加子节点3
  PointCloud2->addChild(PointCloud2PointSize);
  //初始化一个lineedit
  QLineEdit* PointSizeLineEdit = new QLineEdit("3");
  //正则表达式限制只能输入正浮点数
  PointSizeLineEdit->setValidator(new QRegExpValidator(QRegExp("^(([0-9]+\\.[0-9]*[1-9][0-9]*)|([0-9]*[1-9][0-9]*\\.[0-9]+)|([0-9]*[1-9][0-9]*)|0)$")));
  //设置最小宽度
  PointSizeLineEdit->setMinimumWidth(150);
  //添加lineedit于子节点3第一列
  treeWidget->setItemWidget(PointCloud2PointSize, 1, PointSizeLineEdit);
  //连接lineedit文本改变信号与修改点云2图层属性槽函数
  connect(PointSizeLineEdit, &QLineEdit::textChanged, this, &MainWindow::slot_modify_PointCloud2_attribute);
  //连接lineedit编辑结束信号与数据处理槽函数
  connect(PointSizeLineEdit, &QLineEdit::editingFinished, this, &MainWindow::slot_onPointSizeLineEdit_edit_finished);

  //调用qrviz类的接口创建图层对象实体并取得其指针
  rviz_common::Display* display = myviz->CreateDisplay("rviz_default_plugins/PointCloud2", "myPointCloud2");
  //向Hash表中插入键值对
  m_LayerHash.insert(PointCloud2, display);
  //默认调用一次修改图层属性函数
  myviz->ModifyPointCloud2Attribute(display, combobox->currentText(), StyleComboBox->currentText(), PointSizeLineEdit->text().toDouble(), checkBox->isChecked());

  return PointCloud2;
}


/**
 * @brief 向QTreeidget添加RobotModel顶层节点
 * @param QTreeWidget* treeWidget 待添加RobotModel顶层节点的QTreeWidget指针
 */
QTreeWidgetItem* MainWindow::add_TreeWidgetItem_RobotModel(QTreeWidget *treeWidget, QString& displayName)
{
  QTreeWidgetItem* RobotModel = new QTreeWidgetItem(QStringList() << displayName);
  //设置图标
  RobotModel->setIcon(0, QIcon("://rViz/RobotModel.png"));
  //初始化一个checkbox
  QCheckBox* checkbox = new QCheckBox();
  //设置复选框默认选中
  checkbox->setCheckState(Qt::Checked);
  //连接checkbox状态改变信号与修改机器人模型图层属性槽函数
  connect(checkbox, &QCheckBox::stateChanged, this, &MainWindow::slot_modify_RobotModel_attribute);
  //加载根节点
  treeWidget->addTopLevelItem(RobotModel);
  //添加checkbox于根节点第一列
  treeWidget->setItemWidget(RobotModel, 1, checkbox);
  //设置用户数据
  RobotModel->setData(0, Qt::UserRole, AddLayer::robotmodel);

  //调用qrviz类的接口创建图层对象实体并取得其指针
  rviz_common::Display* display = myviz->CreateDisplay("rviz_default_plugins/RobotModel", "myRobotModel");
  //向Hash表中插入键值对
  m_LayerHash.insert(RobotModel, display);
  //默认调用一次修改图层属性函数
  myviz->ModifyRobotModelAttribute(display, checkbox->isChecked());
  return RobotModel;
}


/**
 * @brief 向QTreeidget添加Map顶层节点
 * @param QTreeWidget* treeWidget 待添加Map顶层节点的QTreeWidget指针
 */
QTreeWidgetItem* MainWindow::add_TreeWidgetItem_Map(QTreeWidget* treeWidget, QString& displayName, QString& topicName)
{
  QTreeWidgetItem* Map = new QTreeWidgetItem(QStringList() << displayName);
  //设置图标
  Map->setIcon(0, QIcon("://rViz/Map.png"));
//  Map->setFlags(Map->flags() | Qt::ItemIsUserCheckable);    //为顶层节点设置Qt::ItemIsUserCheckable标志同样可以添加复选框
//  Map->setCheckState(1, Qt::Checked);                       //但QTreeWidget没有提供一个专门的信号来检测QTreeWidgetItem的选中状态改变，不过我们可以使用itemChanged(QTreeWidgetItem * item, int column)这个信号
  //初始化一个checkbox
  QCheckBox* checkbox = new QCheckBox();
  //设置复选框默认选中
  checkbox->setChecked(Qt::Checked);
  //连接checkbox状态改变信号与修改地图图层属性槽函数
  connect(checkbox, &QCheckBox::stateChanged, this, &MainWindow::slot_modify_Map_attribute);
  //加载根节点
  treeWidget->addTopLevelItem(Map);
  //设置默认展开(默认展开应写在加载根节点后)
  //Map->setExpanded(true);
  //添加checkbox于根节点第一列
  treeWidget->setItemWidget(Map, 1, checkbox);
  //设置用户数据
  Map->setData(0, Qt::UserRole, AddLayer::map);

  /*
   * 定义子节点1 ———— topic
   */
  QTreeWidgetItem* MapTopic = new QTreeWidgetItem(QStringList() << "话题");
  //添加子节点1
  Map->addChild(MapTopic);
  //初始化一个combobox
  QComboBox* combobox1 = new QComboBox();
  combobox1->addItems(AddLayer::getTopicListByLayerCode(AddLayer::map));         //添加项
  combobox1->setCurrentIndex(-1);                                               //设置默认不显示项
  combobox1->setCurrentText(topicName);                                         //设置项(需要注意的是，如果topicName不匹配任何一个已经添加好的项，那么setCurrentText()方法将不起作用)
  combobox1->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLength);     //设置尺寸调整策略为适应最小内容
  combobox1->setEditable(true);                                                 //设置可编辑
  //添加combobox于子节点1的第一列
  treeWidget->setItemWidget(MapTopic, 1, combobox1);
  //连接combobox当前文本改变信号与修改地图图层属性槽函数
  connect(combobox1, &QComboBox::currentTextChanged, this, &MainWindow::slot_modify_Map_attribute);

  /*
   * 定义子节点2 ———— color scheme
   */
  QTreeWidgetItem* MapColorScheme = new QTreeWidgetItem(QStringList() << "配色方案");
  //添加子节点2
  Map->addChild(MapColorScheme);
  //初始化一个combobox
  QComboBox* combobox2 = new QComboBox();
  combobox2->addItems(QStringList() << "map" << "costmap" << "raw");          //添加项
  combobox2->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLength);   //设置尺寸调整策略为适应最小内容
  combobox2->setEditable(false);                                              //设置不可编辑
  //添加combobox于子节点2的第一列
  treeWidget->setItemWidget(MapColorScheme, 1, combobox2);
  //连接combobox当前文本改变信号与修改地图图层属性槽函数
  connect(combobox2, &QComboBox::currentTextChanged, this, &MainWindow::slot_modify_Map_attribute);

  //调用qrviz类的接口创建图层对象实体并取得其指针
  rviz_common::Display* display = myviz->CreateDisplay("rviz_default_plugins/Map", "myMap");
  //向Hash表中插入键值对
  m_LayerHash.insert(Map, display);
  //默认调用一次修改图层属性函数
  myviz->ModifyMapAttribute(display, combobox1->currentText(), combobox2->currentText(), checkbox->isChecked());
  //返回对象指针
  return Map;
}


/**
 * @brief 向QTreeidget添加Path顶层节点
 * @param QTreeWidget* treeWidget 待添加Path顶层节点的QTreeWidget指针
 */
QTreeWidgetItem* MainWindow::add_TreeWidgetItem_Path(QTreeWidget* treeWidget, QString& displayName, QString& topicName, QColor color)
{
  QTreeWidgetItem* Path = new QTreeWidgetItem(QStringList() << displayName);
  //设置图标
  Path->setIcon(0, QIcon("://rViz/Path.png"));
  //初始化一个checkbox
  QCheckBox* checkbox = new QCheckBox();
  //设置复选框默认选中
  checkbox->setCheckState(Qt::Checked);
  //连接checkbox状态改变信号与修改路径图层属性槽函数
  connect(checkbox, &QCheckBox::stateChanged, this, &MainWindow::slot_modify_Path_attribute);
  //加载根节点
  treeWidget->addTopLevelItem(Path);
  //设置默认展开(默认展开应写在加载根节点后)
  //Path->setExpanded(true);
  //添加checkbox于根节点第一列
  treeWidget->setItemWidget(Path, 1, checkbox);
  //设置用户数据
  Path->setData(0, Qt::UserRole, AddLayer::path);

  /*
   * 定义子节点1 ———— topic
   */
  QTreeWidgetItem* PathTopic = new QTreeWidgetItem(QStringList() << "话题");
  //添加子节点1
  Path->addChild(PathTopic);
  //初始化一个combobox
  QComboBox* combobox = new QComboBox();
  combobox->addItems(AddLayer::getTopicListByLayerCode(AddLayer::path));                                        //添加项
  combobox->setCurrentIndex(-1);                                                                                //设置默认不显示项
  combobox->setCurrentText(topicName);                                                                          //设置项(需要注意的是，如果topicName不匹配任何一个已经添加好的项，那么setCurrentText()方法将不起作用)
  combobox->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLength);                                      //设置尺寸调整策略为适应最小内容
  combobox->setEditable(true);                                                                                  //设置可编辑
  //添加combobox于子节点1的第一列
  treeWidget->setItemWidget(PathTopic, 1, combobox);
  //连接combobox当前文本改变信号与修改路径图层属性槽函数
  connect(combobox, &QComboBox::currentTextChanged, this, &MainWindow::slot_modify_Path_attribute);

  /*
   * 定义子节点2 ———— color
   */
  QTreeWidgetItem* PathColor = new QTreeWidgetItem(QStringList() << "路径颜色");
  //添加子节点2
  Path->addChild(PathColor);
  //初始化一个MyPalette对象
  MyPalette* palette = new MyPalette();
  //设置初始颜色
  palette->setColor(color);
  //添加mypalette于子节点2的第一列
  treeWidget->setItemWidget(PathColor, 1, palette);
  //连接palette颜色改变信号与修改路径图层属性槽函数
  connect(palette, &MyPalette::colorchanged, this, &MainWindow::slot_modify_Path_attribute);

  //调用qrviz类的接口创建图层对象实体并取得其指针
  rviz_common::Display* display = myviz->CreateDisplay("rviz_default_plugins/Path", "myPath");
  //向Hash表中插入键值对
  m_LayerHash.insert(Path, display);
  //默认调用一次修改图层属性函数
  myviz->ModifyPathAttribute(display, combobox->currentText(), palette->currentColor(), checkbox->isChecked());

  return Path;
}


/**
 * @brief 向QTreeidget添加PointStamped顶层节点
 * @param QTreeWidget* treeWidget 待添加PointStamped顶层节点的QTreeWidget指针
 */
QTreeWidgetItem* MainWindow::add_TreeWidgetItem_PointStamped(QTreeWidget* treeWidget, QString& displayName, QString& topicName)
{
  //PointStamped代表了一个带有参考坐标系和时间戳的点
  QTreeWidgetItem* PointStamped = new QTreeWidgetItem(QStringList() << displayName);
  //设置图标
  PointStamped->setIcon(0, QIcon("://rViz/PointStamped.png"));
  //初始化一个checkbox
  QCheckBox* checkbox = new QCheckBox();
  //设置复选框默认选中
  checkbox->setCheckState(Qt::Checked);
  //连接checkbox状态改变信号与修改带点戳图层属性槽函数
  connect(checkbox, &QCheckBox::stateChanged, this, &MainWindow::slot_modify_PointStamped_attribute);
  //加载根节点
  treeWidget->addTopLevelItem(PointStamped);
  //设置默认展开(默认展开应写在加载根节点后)
  //PointStamped->setExpanded(true);
  //添加checkbox于根节点第一列
  treeWidget->setItemWidget(PointStamped, 1, checkbox);
  //设置用户数据
  PointStamped->setData(0, Qt::UserRole, AddLayer::pointstamped);


  /*
   * 定义子节点1 ———— topic
   */
  QTreeWidgetItem* PointStampedTopic = new QTreeWidgetItem(QStringList() << "话题");
  //添加子节点1
  PointStamped->addChild(PointStampedTopic);
  //初始化一个combobox
  QComboBox* combobox = new QComboBox();
  combobox->addItems(AddLayer::getTopicListByLayerCode(AddLayer::pointstamped));    //添加项
  combobox->setCurrentIndex(-1);                                              //设置默认不显示项
  combobox->setCurrentText(topicName);                                        //设置项(需要注意的是，如果topicName不匹配任何一个已经添加好的项，那么setCurrentText()方法将不起作用)
  combobox->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLength);    //设置尺寸调整策略为适应最小内容
  combobox->setEditable(true);                                                //设置可编辑
  //添加combobox于子节点1的第一列
  treeWidget->setItemWidget(PointStampedTopic, 1, combobox);
  //连接combobox当前文本改变信号与修改带点戳图层属性槽函数
  connect(combobox, &QComboBox::currentTextChanged, this, &MainWindow::slot_modify_PointStamped_attribute);

  /*
   * 定义子节点2 ———— color
   */
  QTreeWidgetItem* PointStampedColor = new QTreeWidgetItem(QStringList() << "点颜色");
  //添加子节点2
  PointStamped->addChild(PointStampedColor);
  //初始化一个MyPalette对象
  MyPalette* palette = new MyPalette();
  //设置初始颜色
  palette->setColor(204, 41, 204);
  //添加mypalette于子节点2的第一列
  treeWidget->setItemWidget(PointStampedColor, 1, palette);
  //连接mypalette颜色改变信号与修改带点戳图层属性槽函数
  connect(palette, &MyPalette::colorchanged, this, &MainWindow::slot_modify_PointStamped_attribute);

  //调用qrviz类的接口创建图层对象实体并取得其指针
  rviz_common::Display* display = myviz->CreateDisplay("rviz_default_plugins/PointStamped", "myPointStamped");
  //向Hash表中插入键值对
  m_LayerHash.insert(PointStamped, display);
  //默认调用一次修改图层属性函数
  myviz->ModifyPointStampedAttribute(display, combobox->currentText(), palette->currentColor(), checkbox->isChecked());

  return PointStamped;
}


/**
 * @brief 向QTreeidget添加Marker顶层节点
 * @param QTreeWidget* treeWidget 待添加Marker顶层节点的QTreeWidget指针
 */
QTreeWidgetItem* MainWindow::add_TreeWidgetItem_Marker(QTreeWidget* treeWidget, QString& displayName, QString& topicName)
{
  QTreeWidgetItem* Marker = new QTreeWidgetItem(QStringList() << displayName);
  //设置图标
  Marker->setIcon(0, QIcon("://rViz/Marker.png"));
  //初始化一个checkbox
  QCheckBox* checkbox = new QCheckBox();
  //设置复选框默认选中
  checkbox->setCheckState(Qt::Checked);
  //加载根节点
  treeWidget->addTopLevelItem(Marker);
  //设置默认展开(默认展开应写在加载根节点后)
  //Marker->addChild(MarkerCBox);
  //添加checkbox于根节点的第一列
  treeWidget->setItemWidget(Marker, 1, checkbox);
  //连接checkbox状态改变信号与修改标记图层属性槽函数
  connect(checkbox, &QCheckBox::stateChanged, this, &MainWindow::slot_modify_Marker_attribute);
  //设置用户数据
  Marker->setData(0, Qt::UserRole, AddLayer::marker);

  /*
   * 定义子节点1 ———— topic
   */
  QTreeWidgetItem* MarkerTopic = new QTreeWidgetItem(QStringList() << "标记话题");
  //添加子节点1
  Marker->addChild(MarkerTopic);
  //初始化一个combobox
  QComboBox* combobox = new QComboBox();
  combobox->addItems(AddLayer::getTopicListByLayerCode(AddLayer::marker));            //添加项
  combobox->setCurrentIndex(-1);                                                      //设置默认不显示项
  combobox->setCurrentText(topicName);                                                //设置项(需要注意的是，如果topicName不匹配任何一个已经添加好的项，那么setCurrentText()方法将不起作用)
  combobox->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLength);            //设置尺寸调整策略为适应最小内容
  combobox->setEditable(true);                                                        //设置可编辑
  //添加combobox于子节点1的第一列
  treeWidget->setItemWidget(MarkerTopic, 1, combobox);
  //连接combobox当前文本改变信号与修改标记图层属性槽函数
  connect(combobox, &QComboBox::currentTextChanged, this, &MainWindow::slot_modify_Marker_attribute);

  //调用qrviz类的接口创建图层对象实体并取得其指针
  rviz_common::Display* display = myviz->CreateDisplay("rviz_default_plugins/Marker", "myMarker");
  //向Hash表中插入键值对
  m_LayerHash.insert(Marker, display);
  //默认调用一次修改图层属性函数
  myviz->ModifyMarkerAttribute(display, combobox->currentText(), checkbox->isChecked());
  return Marker;
}


/**
 * @brief 向QTreeidget添加MarkerArray顶层节点
 * @param QTreeWidget* treeWidget 待添加MarkerArray顶层节点的QTreeWidget指针
 */
QTreeWidgetItem* MainWindow::add_TreeWidgetItem_MarkerArray(QTreeWidget* treeWidget, QString& displayName, QString& topicName)
{
  QTreeWidgetItem* MarkerArray = new QTreeWidgetItem(QStringList() << displayName);
  //设置图标
  MarkerArray->setIcon(0, QIcon("://rViz/MarkerArray.png"));
  //初始化一个checkbox
  QCheckBox* checkbox = new QCheckBox();
  //设置复选框默认选中
  checkbox->setCheckState(Qt::Checked);
  //加载根节点
  treeWidget->addTopLevelItem(MarkerArray);
  //设置默认展开
  //MarkerArray->setExpanded(true);
  //添加checkbox于根节点的第一列
  treeWidget->setItemWidget(MarkerArray, 1, checkbox);
  //连接checkbox状态改变信号与修改标记数组图层属性槽函数
  connect(checkbox, &QCheckBox::stateChanged, this, &MainWindow::slot_modify_MarkerArray_attribute);
  //设置用户数据
  MarkerArray->setData(0, Qt::UserRole, AddLayer::markerarray);

  /*
   * 定义子节点1 ———— topic
   */
  QTreeWidgetItem* MarkerArrayTopic = new QTreeWidgetItem(QStringList() << "标记话题");
  //添加子节点1
  MarkerArray->addChild(MarkerArrayTopic);
  //初始化一个combobox
  QComboBox* combobox = new QComboBox();
  combobox->addItems(AddLayer::getTopicListByLayerCode(AddLayer::markerarray));   //添加项
  combobox->setCurrentIndex(-1);                                              //设置默认不显示项
  combobox->setCurrentText(topicName);                                        //设置项(需要注意的是，如果topicName不匹配任何一个已经添加好的项，那么setCurrentText()方法将不起作用)
  combobox->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLength);    //设置尺寸调整策略为适应最小内容
  combobox->setEditable(true);                                                //设置可编辑
  //添加combobox于子节点1的第一列
  treeWidget->setItemWidget(MarkerArrayTopic, 1, combobox);
  //连接combobox当前文本改变信号与修改标记数组图层属性槽函数
  connect(combobox, &QComboBox::currentTextChanged, this, &MainWindow::slot_modify_MarkerArray_attribute);

  //调用qrviz类的接口创建图层对象实体并取得其指针
  rviz_common::Display* display = myviz->CreateDisplay("rviz_default_plugins/MarkerArray", "myMarkerArray");
  //向Hash表中插入键值对
  m_LayerHash.insert(MarkerArray, display);
  //默认调用一次修改图层属性函数
  myviz->ModifyMarkerArrayAttribute(display, combobox->currentText(), checkbox->isChecked());

  return MarkerArray;
}



/***********************************************************************************************
 *                                    修改图层相关属性方法定义                                     *
 ***********************************************************************************************/
/**
 * @brief 修改网格图层属性槽函数
 */
void MainWindow::slot_modify_grid_attribute()
{
  QTreeWidgetItem* grid;
  QCheckBox* checkBox;
  QSpinBox* spinBox;
  MyPalette* palette;
  if(ui.treeWidget_layer->currentItem()->parent() == nullptr)
  {
    //获取顶层节点
    grid = ui.treeWidget_layer->currentItem();
  }
  else
  {
    //获取顶层节点
    grid = ui.treeWidget_layer->currentItem()->parent();
  }
  //获取顶层节点第二列单元格中的QCheckBox控件
  checkBox = qobject_cast<QCheckBox*>(ui.treeWidget_layer->itemWidget(grid, 1));
  //获取顶层节点的子节点1第二列单元格中的QSpinBox控件
  spinBox = qobject_cast<QSpinBox*>(ui.treeWidget_layer->itemWidget(grid->child(0), 1));
  //获取顶层节点的子节点2第二列单元格中的MyPalette控件
  palette = qobject_cast<MyPalette*>(ui.treeWidget_layer->itemWidget(grid->child(1), 1));
  //根据复选框是否选中决定是否启用Grid树
  if(checkBox->isChecked())
  {
    for(int index = 0; index < grid->childCount(); index++)
    {
      grid->child(index)->setDisabled(false);
    }
    spinBox->setEnabled(true);
    palette->setEnabled(true);
  }
  else
  {
    for(int index = 0; index < grid->childCount(); index++)
    {
      grid->child(index)->setDisabled(true);
    }
    spinBox->setEnabled(false);
    palette->setEnabled(false);
  }
  //修改属性
  myviz->ModifyGridAttribute(m_LayerHash.value(grid), spinBox->value(), palette->currentColor(), checkBox->isChecked());
}



/**
 * @brief 修改TF图层属性槽函数
 */
void MainWindow::slot_modify_TF_attribute()
{
  QTreeWidgetItem* tf;
  QCheckBox* checkBox;
  if(ui.treeWidget_layer->currentItem()->parent() == nullptr)
  {
    //获取顶层节点
    tf = ui.treeWidget_layer->currentItem();
  }
  else
  {
    //获取顶层节点
    tf = ui.treeWidget_layer->currentItem()->parent();
  }
  //获取顶层节点第二列单元格中的QCheckBox控件
  checkBox = qobject_cast<QCheckBox*>(ui.treeWidget_layer->itemWidget(tf, 1));
  //修改属性
  myviz->ModifyTFAttribute(m_LayerHash.value(tf), checkBox->isChecked());
}



/**
 * @brief 修改激光扫描图层属性槽函数
 */
void MainWindow::slot_modify_LaserScan_attribute()
{
  QTreeWidgetItem* laserscan;
  QCheckBox* checkBox;
  QComboBox* comboBox;
  QComboBox* StyleComboBox;
  QLineEdit* PointSizeLineEdit;
  if(ui.treeWidget_layer->currentItem()->parent() == nullptr)
  {
    //获取顶层节点
    laserscan = ui.treeWidget_layer->currentItem();
  }
  else
  {
    //获取顶层节点
    laserscan = ui.treeWidget_layer->currentItem()->parent();
  }
  //获取顶层节点第二列单元格中的QCheckBox控件
  checkBox = qobject_cast<QCheckBox*>(ui.treeWidget_layer->itemWidget(laserscan, 1));
  //获取顶层节点的子节点1第二列单元格中的QComboBox控件
  comboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(laserscan->child(0), 1));
  //获取顶层节点的子节点2第二列单元格中的QComboBox控件
  StyleComboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(laserscan->child(1), 1));
  //获取顶层节点的子节点3第二列单元格中的QLineEdit控件
  PointSizeLineEdit = qobject_cast<QLineEdit*>(ui.treeWidget_layer->itemWidget(laserscan->child(2), 1));
  if(checkBox->isChecked())
  {
    for(int index = 0; index < laserscan->childCount(); index++)
    {
      laserscan->child(index)->setDisabled(false);
    }
    comboBox->setEnabled(true);
    StyleComboBox->setEnabled(true);
    PointSizeLineEdit->setEnabled(true);
  }
  else
  {
    for(int index = 0; index < laserscan->childCount(); index++)
    {
      laserscan->child(index)->setDisabled(true);
    }
    comboBox->setEnabled(false);
    StyleComboBox->setEnabled(false);
    PointSizeLineEdit->setEnabled(false);
  }
  double doubleSize = PointSizeLineEdit->text().toDouble();
  if(doubleSize < 0.0001)
  {
    doubleSize = 0.0001;    //限制最小输入
  }
  myviz->ModifyLaserScanAttribute(m_LayerHash.value(laserscan), comboBox->currentText(), StyleComboBox->currentText(), doubleSize, checkBox->isChecked());
}



/**
 * @brief 修改点云2图层属性槽函数
 */
void MainWindow::slot_modify_PointCloud2_attribute()
{
  QTreeWidgetItem* pointcloud2;
  QCheckBox* checkBox;
  QComboBox* comboBox;
  QComboBox* StyleComboBox;
  QLineEdit* PointSizeLineEdit;
  if(ui.treeWidget_layer->currentItem()->parent() == nullptr)
  {
    //获取顶层节点
    pointcloud2 = ui.treeWidget_layer->currentItem();
  }
  else
  {
    //获取顶层节点
    pointcloud2 = ui.treeWidget_layer->currentItem()->parent();
  }
  //获取顶层节点第二列单元格中的QCheckBox控件
  checkBox = qobject_cast<QCheckBox*>(ui.treeWidget_layer->itemWidget(pointcloud2, 1));
  //获取顶层节点的子节点1第二列单元格中的QComboBox控件
  comboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(pointcloud2->child(0), 1));
  //获取顶层节点的子节点2第二列单元格中的QComboBox控件
  StyleComboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(pointcloud2->child(1), 1));
  //获取顶层节点的子节点3第二列单元格中的QLineEdit控件
  PointSizeLineEdit = qobject_cast<QLineEdit*>(ui.treeWidget_layer->itemWidget(pointcloud2->child(2), 1));
  if(checkBox->isChecked())
  {
    for(int index = 0; index < pointcloud2->childCount(); index++)
    {
      pointcloud2->child(index)->setDisabled(false);
    }
    comboBox->setEnabled(true);
    StyleComboBox->setEnabled(true);
    PointSizeLineEdit->setEnabled(true);
  }
  else
  {
    for(int index = 0; index < pointcloud2->childCount(); index++)
    {
      pointcloud2->child(index)->setDisabled(true);
    }
    comboBox->setEnabled(false);
    StyleComboBox->setEnabled(false);
    PointSizeLineEdit->setEnabled(false);
  }
  double doubleSize = PointSizeLineEdit->text().toDouble();
  if(doubleSize < 0.0001)
  {
    doubleSize = 0.0001;    //限制最小输入
  }
  myviz->ModifyPointCloud2Attribute(m_LayerHash.value(pointcloud2), comboBox->currentText(), StyleComboBox->currentText(), doubleSize, checkBox->isChecked());
}


/**
 * @brief style改变槽函数
 */
void MainWindow::slot_onStyleComboBox_text_changed()
{
  //获取当前item
  QTreeWidgetItem* current_item = ui.treeWidget_layer->currentItem();
  //获取父节点
  QTreeWidgetItem* parent_item = current_item->parent();
  //获取当前item的索引
  int current_index = parent_item->indexOfChild(current_item);
  //获取下一个item
  QTreeWidgetItem* next_item = parent_item->child(current_index + 1);
  /*Qt 5访问TreeWidgetItem的兄弟节点比较麻烦，总体逻辑如上代码。如果您的Qt版本为Qt 6及以上，
    则可以使用QTreeWidgetItem类的sibling(int i)方法快速地访问相对位置为i（i可以为负数）兄弟节点。
    在使用指针返回值之前，建议做非空检查；这里我偷懒没有做任何非空检查*/

  //获取当前item第二列的ComboBox
  QComboBox* combobox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(current_item, 1));
  //获取下一个item第二列的LineEdit
  QLineEdit* lineedit = qobject_cast<QLineEdit*>(ui.treeWidget_layer->itemWidget(next_item, 1));
  if(combobox->currentText() == "Points")
  {
    next_item->setText(0, "大小 (Pixels)");
    lineedit->setText("3");
  }
  else
  {
    next_item->setText(0, "大小 (m)");
    lineedit->setText("0.1");
  }
}



/**
 * @brief point size编辑结束槽函数
 */
void MainWindow::slot_onPointSizeLineEdit_edit_finished()
{
  //获取当前item第二列的Linedit
  QLineEdit* lineedit = qobject_cast<QLineEdit*>(ui.treeWidget_layer->itemWidget(ui.treeWidget_layer->currentItem(), 1));
  double doubleSize = lineedit->text().toDouble();
  if(doubleSize < 0.0001)
  {
    doubleSize = 0.0001;      //限制最小输入
  }
  lineedit->setText(QString::number(doubleSize));
}


/**
 * @brief 修改机器人模型图层属性槽函数
 */
void MainWindow::slot_modify_RobotModel_attribute()
{
  QTreeWidgetItem* robotmodel;
  QCheckBox* checkBox;
  if(ui.treeWidget_layer->currentItem()->parent() == nullptr)
  {
    //获取顶层节点
    robotmodel = ui.treeWidget_layer->currentItem();
  }
  else
  {
    //获取顶层节点
    robotmodel = ui.treeWidget_layer->currentItem()->parent();
  }
  //获取顶层节点第二列单元格中的QCheckBox控件
  checkBox = qobject_cast<QCheckBox*>(ui.treeWidget_layer->itemWidget(robotmodel, 1));
  //修改属性
  myviz->ModifyRobotModelAttribute(m_LayerHash.value(robotmodel), checkBox->isChecked());
}


/**
 * @brief 修改地图图层属性槽函数
 */
void MainWindow::slot_modify_Map_attribute()
{
  QTreeWidgetItem* map;
  QCheckBox* checkBox;
  QComboBox* comboBox1;
  QComboBox* comboBox2;
  if(ui.treeWidget_layer->currentItem()->parent() == nullptr)
  {
    //获取顶层节点
    map = ui.treeWidget_layer->currentItem();
  }
  else
  {
    //获取顶层节点
    map = ui.treeWidget_layer->currentItem()->parent();
  }
  //获取顶层节点第二列单元格中的QCheckBox控件
  checkBox = qobject_cast<QCheckBox*>(ui.treeWidget_layer->itemWidget(map, 1));
  //获取顶层节点的子节点1第二列单元格中的QComboBox控件
  comboBox1 = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(map->child(0), 1));
  //获取顶层节点的子节点2第二列单元格中的QComboBox控件
  comboBox2 = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(map->child(1), 1));
  if(checkBox->isChecked())
  {
    for(int index = 0; index < map->childCount(); index++)
    {
      map->child(index)->setDisabled(false);
    }
    comboBox1->setEnabled(true);
    comboBox2->setEnabled(true);
  }
  else
  {
    for(int index = 0; index < map->childCount(); index++)
    {
      map->child(index)->setDisabled(true);
    }
    comboBox1->setEnabled(false);
    comboBox2->setEnabled(false);
  }
  myviz->ModifyMapAttribute(m_LayerHash.value(map), comboBox1->currentText(), comboBox2->currentText(), checkBox->isChecked());
}


/**
 * @brief 修改路径图层属性槽函数
 */
void MainWindow::slot_modify_Path_attribute()
{
  QTreeWidgetItem* path;
  QCheckBox* checkBox;
  QComboBox* comboBox;
  MyPalette* palette;
  if(ui.treeWidget_layer->currentItem()->parent() == nullptr)
  {
    //获取顶层节点
    path = ui.treeWidget_layer->currentItem();
  }
  else
  {
    //获取顶层节点
    path = ui.treeWidget_layer->currentItem()->parent();
  }
  //获取顶层节点第二列单元格中的QCheckBox控件
  checkBox = qobject_cast<QCheckBox*>(ui.treeWidget_layer->itemWidget(path, 1));
  //获取顶层节点的子节点1第二列单元格中的QComboBox控件
  comboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(path->child(0), 1));
  //获取顶层节点的子节点2第二列单元格中的MyPalette控件
  palette = qobject_cast<MyPalette*>(ui.treeWidget_layer->itemWidget(path->child(1), 1));
  if(checkBox->isChecked())
  {
    for(int index = 0; index < path->childCount(); index++)
    {
      path->child(index)->setDisabled(false);
    }
    comboBox->setEnabled(true);
    palette->setEnabled(true);
  }
  else
  {
    for(int index = 0; index < path->childCount(); index++)
    {
      path->child(index)->setDisabled(true);
    }
    comboBox->setEnabled(false);
    palette->setEnabled(false);
  }
  //调用接口
  myviz->ModifyPathAttribute(m_LayerHash.value(path), comboBox->currentText(), palette->currentColor(), checkBox->isChecked());
}


/**
 * @brief 修改带点戳图层属性槽函数
 */
void MainWindow::slot_modify_PointStamped_attribute()
{
  QTreeWidgetItem* pointStamped;
  QCheckBox* checkBox;
  QComboBox* comboBox;
  MyPalette* palette;
  if(ui.treeWidget_layer->currentItem()->parent() == nullptr)
  {
    //获取顶层节点
    pointStamped = ui.treeWidget_layer->currentItem();
  }
  else
  {
    //获取顶层节点
    pointStamped = ui.treeWidget_layer->currentItem()->parent();
  }
  //获取顶层节点第二列单元格中的QCheckBox控件
  checkBox = qobject_cast<QCheckBox*>(ui.treeWidget_layer->itemWidget(pointStamped, 1));
  //获取顶层节点的子节点1第二列单元格中的QComboBox控件
  comboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(pointStamped->child(0), 1));
  //获取顶层节点的子节点2第二列单元格中的MyPalette控件
  palette = qobject_cast<MyPalette*>(ui.treeWidget_layer->itemWidget(pointStamped->child(1), 1));
  if(checkBox->isChecked())
  {
    for(int index = 0; index < pointStamped->childCount(); index++)
    {
      pointStamped->child(index)->setDisabled(false);
    }
    comboBox->setEnabled(true);
    palette->setEnabled(true);
  }
  else
  {
    for(int index = 0; index < pointStamped->childCount(); index++)
    {
      pointStamped->child(index)->setDisabled(true);
    }
    comboBox->setEnabled(false);
    palette->setEnabled(false);
  }
  //调用接口
  myviz->ModifyPointStampedAttribute(m_LayerHash.value(pointStamped), comboBox->currentText(), palette->currentColor(), checkBox->isChecked());
}


/**
 * @brief 修改标记图层属性槽函数
 */
void MainWindow::slot_modify_Marker_attribute()
{
  QTreeWidgetItem* marker;
  QCheckBox* checkBox;
  QComboBox* comboBox;
  if(ui.treeWidget_layer->currentItem()->parent() == nullptr)
  {
    //获取顶层节点
    marker = ui.treeWidget_layer->currentItem();
  }
  else
  {
    //获取顶层节点
    marker = ui.treeWidget_layer->currentItem()->parent();
  }
  //获取顶层节点第二列单元格中的QCheckBox控件
  checkBox = qobject_cast<QCheckBox*>(ui.treeWidget_layer->itemWidget(marker, 1));
  //获取顶层节点的子节点1第二列单元格中的QComboBox控件
  comboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(marker->child(0), 1));
  if(checkBox->isChecked())
  {
    for(int index = 0; index < marker->childCount(); index++)
    {
      marker->child(index)->setDisabled(false);
    }
    comboBox->setEnabled(true);
  }
  else
  {
    for(int index = 0; index < marker->childCount(); index++)
    {
      marker->child(index)->setDisabled(true);
    }
    comboBox->setEnabled(false);
  }
  //调用接口
  myviz->ModifyMarkerAttribute(m_LayerHash.value(marker), comboBox->currentText(), checkBox->isChecked());
}


/**
 * @brief 修改标记数组图层属性槽函数
 */
void MainWindow::slot_modify_MarkerArray_attribute()
{
  QTreeWidgetItem* markerArray;
  QCheckBox* checkBox;
  QComboBox* comboBox;
  if(ui.treeWidget_layer->currentItem()->parent() == nullptr)
  {
    //获取顶层节点
    markerArray = ui.treeWidget_layer->currentItem();
  }
  else
  {
    //获取顶层节点
    markerArray = ui.treeWidget_layer->currentItem()->parent();
  }
  //获取顶层节点第二列单元格中的QCheckBox控件
  checkBox = qobject_cast<QCheckBox*>(ui.treeWidget_layer->itemWidget(markerArray, 1));
  //获取顶层节点的子节点1第二列单元格中的QComboBox控件
  comboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(markerArray->child(0), 1));
  if(checkBox->isChecked())
  {
    for(int index = 0; index < markerArray->childCount(); index++)
    {
      markerArray->child(index)->setDisabled(false);
    }
    comboBox->setEnabled(true);
  }
  else
  {
    for(int index = 0; index < markerArray->childCount(); index++)
    {
      markerArray->child(index)->setDisabled(true);
    }
    comboBox->setEnabled(false);
  }
  //调用接口
  myviz->ModifyMarkerArrayAttribute(m_LayerHash.value(markerArray), comboBox->currentText(), checkBox->isChecked());
}



/***********************************************************************************************
 *                             从UI界面移除待删除的图层相关控件方法定义                               *
 ***********************************************************************************************/
/**
 * @brief 从QTreeidget移除Grid顶层节点
 * @param QTreeWidget* treeWidget ———— 待移除Grid顶层节点的QTreeWidget指针
 * @param QTreeWidgetItem* item   ———— 待移除的QTreeWidgetItem指针
 */
void MainWindow::remove_TreeWidgetItem_Grid(QTreeWidget* treeWidget, QTreeWidgetItem* item)
{
  //删除rviz_common::Display对象
  myviz->DeleteDisplayLayer(m_LayerHash.value(item));
  //从哈希表中移除键值对
  m_LayerHash.remove(item);
  //获取第二列单元格中的QCheckBox控件
  QCheckBox* checkBox = qobject_cast<QCheckBox*>(treeWidget->itemWidget(item, 1));
  if(checkBox)
  {
    //如果该单元格中包含QCheckBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item, 1);
    //堆中创建的对象需要手动释放内存
    delete checkBox;
  }
  //获取子节点1(plane cell count)第二列单元格中的QSpinBox控件
  QSpinBox* spinBox = qobject_cast<QSpinBox*>(treeWidget->itemWidget(item->child(0), 1));
  if(spinBox)
  {
    //如果该单元格中包含QSpinBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item->child(0), 1);
    //堆中创建的对象需要手动释放内存
    delete spinBox;
  }
  //获取子节点2(color)第二列单元格中的MyPalette控件
  MyPalette* palette = qobject_cast<MyPalette*>(treeWidget->itemWidget(item->child(1), 1));
  if(palette)
  {
    //如果该单元格中包含MyPalette控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item->child(1), 1);
    //堆中创建的对象需要手动释放内存
    delete palette;
  }

  //删除顶层节点的所有子节点
  removeChildren(item);
  //从QtreeWidget中移除顶层节点并删除QTreeWidgetItem对象
  treeWidget->takeTopLevelItem(treeWidget->indexOfTopLevelItem(item));
  delete item;
}



/**
 * @brief 从QTreeidget移除TF顶层节点
 * @param QTreeWidget* treeWidget ———— 待移除TF顶层节点的QTreeWidget指针
 * @param QTreeWidgetItem* item   ———— 待移除的QTreeWidgetItem指针
 */
void MainWindow::remove_TreeWidgetItem_TF(QTreeWidget* treeWidget, QTreeWidgetItem* item)
{
  //删除rviz_common::Display对象
  myviz->DeleteDisplayLayer(m_LayerHash.value(item));
  //从哈希表中移除键值对
  m_LayerHash.remove(item);
  //获取第二列单元格中的QCheckBox控件
  QCheckBox* checkBox = qobject_cast<QCheckBox*>(treeWidget->itemWidget(item, 1));
  if(checkBox)
  {
    //如果该单元格中包含QCheckBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item, 1);
    //堆中创建的对象需要手动释放内存
    delete checkBox;
  }

  //删除顶层节点的所有子节点
  removeChildren(item);
  //从QtreeWidget中移除顶层节点并删除QTreeWidgetItem对象
  treeWidget->takeTopLevelItem(treeWidget->indexOfTopLevelItem(item));
  delete item;
}



/**
 * @brief 从QTreeidget移除LaserScan顶层节点
 * @param QTreeWidget* treeWidget ———— 待移除LaserScan顶层节点的QTreeWidget指针
 * @param QTreeWidgetItem* item   ———— 待移除的QTreeWidgetItem指针
 */
void MainWindow::remove_TreeWidgetItem_LaserScan(QTreeWidget* treeWidget, QTreeWidgetItem* item)
{
  //删除rviz_common::Display对象
  myviz->DeleteDisplayLayer(m_LayerHash.value(item));
  //从哈希表中移除键值对
  m_LayerHash.remove(item);
  //获取第二列单元格中的QCheckBox控件
  QCheckBox* checkBox = qobject_cast<QCheckBox*>(treeWidget->itemWidget(item, 1));
  if(checkBox)
  {
    //如果该单元格中包含QCheckBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item, 1);
    //堆中创建的对象需要手动释放内存
    delete checkBox;
  }
  //获取子节点1(topic)第二列单元格中的QComboBox控件
  QComboBox* comboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(item->child(0), 1));
  if(comboBox)
  {
    //如果该单元格中包含QSpinBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item->child(0), 1);
    //堆中创建的对象需要手动释放内存
    delete comboBox;
  }
  //获取子节点2(style)第二列单元格中的QComboBox控件
  QComboBox* StyleComboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(item->child(1), 1));
  if(StyleComboBox)
  {
    //如果该单元格中包含MyPalette控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item->child(1), 1);
    //堆中创建的对象需要手动释放内存
    delete StyleComboBox;
  }
  //获取子节点3(point size)第二列单元格中的QLineEdit控件
  QLineEdit* PointSizeLineEdit = qobject_cast<QLineEdit*>(ui.treeWidget_layer->itemWidget(item->child(2), 1));
  if(PointSizeLineEdit)
  {
    //如果该单元格中包含MyPalette控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item->child(2), 1);
    //堆中创建的对象需要手动释放内存
    delete PointSizeLineEdit;
  }

  //删除顶层节点的所有子节点
  removeChildren(item);
  //从QtreeWidget中移除顶层节点并删除QTreeWidgetItem对象
  treeWidget->takeTopLevelItem(treeWidget->indexOfTopLevelItem(item));
  delete item;
}



/**
 * @brief 从QTreeidget移除PointCloud2顶层节点
 * @param QTreeWidget* treeWidget ———— 待移除PointCloud2顶层节点的QTreeWidget指针
 * @param QTreeWidgetItem* item   ———— 待移除的QTreeWidgetItem指针
 */
void MainWindow::remove_TreeWidgetItem_PointCloud2(QTreeWidget* treeWidget, QTreeWidgetItem* item)
{
  //删除rviz_common::Display对象
  myviz->DeleteDisplayLayer(m_LayerHash.value(item));
  //从哈希表中移除键值对
  m_LayerHash.remove(item);
  //获取第二列单元格中的QCheckBox控件
  QCheckBox* checkBox = qobject_cast<QCheckBox*>(treeWidget->itemWidget(item, 1));
  if(checkBox)
  {
    //如果该单元格中包含QCheckBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item, 1);
    //堆中创建的对象需要手动释放内存
    delete checkBox;
  }
  //获取子节点1(topic)第二列单元格中的QComboBox控件
  QComboBox* comboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(item->child(0), 1));
  if(comboBox)
  {
    //如果该单元格中包含QSpinBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item->child(0), 1);
    //堆中创建的对象需要手动释放内存
    delete comboBox;
  }
  //获取子节点2(style)第二列单元格中的QComboBox控件
  QComboBox* StyleComboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(item->child(1), 1));
  if(StyleComboBox)
  {
    //如果该单元格中包含QComboBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item->child(1), 1);
    //堆中创建的对象需要手动释放内存
    delete StyleComboBox;
  }
  //获取子节点3(point size)第二列单元格中的QLineEdit控件
  QLineEdit* PointSizeLineEdit = qobject_cast<QLineEdit*>(ui.treeWidget_layer->itemWidget(item->child(2), 1));
  if(PointSizeLineEdit)
  {
    //如果该单元格中包含QLineEdit控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item->child(2), 1);
    //堆中创建的对象需要手动释放内存
    delete PointSizeLineEdit;
  }

  //删除顶层节点的所有子节点
  removeChildren(item);
  //从QtreeWidget中移除顶层节点并删除QTreeWidgetItem对象
  treeWidget->takeTopLevelItem(treeWidget->indexOfTopLevelItem(item));
  delete item;
}



/**
 * @brief 从QTreeidget移除RobotModel顶层节点
 * @param QTreeWidget* treeWidget ———— 待移除RobotModel顶层节点的QTreeWidget指针
 * @param QTreeWidgetItem* item   ———— 待移除的QTreeWidgetItem指针
 */
void MainWindow::remove_TreeWidgetItem_RobotModel(QTreeWidget* treeWidget, QTreeWidgetItem* item)
{
  //删除rviz_common::Display对象
  myviz->DeleteDisplayLayer(m_LayerHash.value(item));
  //从哈希表中移除键值对
  m_LayerHash.remove(item);
  //获取第二列单元格中的QCheckBox控件
  QCheckBox* checkBox = qobject_cast<QCheckBox*>(treeWidget->itemWidget(item, 1));
  if(checkBox)
  {
    //如果该单元格中包含QCheckBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item, 1);
    //堆中创建的对象需要手动释放内存
    delete checkBox;
  }

  //删除顶层节点的所有子节点
  removeChildren(item);
  //从QtreeWidget中移除顶层节点并删除QTreeWidgetItem对象
  treeWidget->takeTopLevelItem(treeWidget->indexOfTopLevelItem(item));
  delete item;
}




/**
 * @brief 从QTreeidget移除Map顶层节点
 * @param QTreeWidget* treeWidget ———— 待移除Map顶层节点的QTreeWidget指针
 * @param QTreeWidgetItem* item   ———— 待移除的QTreeWidgetItem指针
 */
void MainWindow::remove_TreeWidgetItem_Map(QTreeWidget* treeWidget, QTreeWidgetItem* item)
{
  //删除rviz_common::Display对象
  myviz->DeleteDisplayLayer(m_LayerHash.value(item));
  //从哈希表中移除键值对
  m_LayerHash.remove(item);
  //获取第二列单元格中的QCheckBox控件
  QCheckBox* checkBox = qobject_cast<QCheckBox*>(treeWidget->itemWidget(item, 1));
  if(checkBox)
  {
    //如果该单元格中包含QCheckBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item, 1);
    //堆中创建的对象需要手动释放内存
    delete checkBox;
  }
  //获取子节点1(topic)第二列单元格中的QComboBox控件
  QComboBox* comboBox1 = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(item->child(0), 1));
  if(comboBox1)
  {
    //如果该单元格中包含QSpinBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item->child(0), 1);
    //堆中创建的对象需要手动释放内存
    delete comboBox1;
  }
  //获取子节点2(style)第二列单元格中的QComboBox控件
  QComboBox* comboBox2 = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(item->child(1), 1));
  if(comboBox2)
  {
    //如果该单元格中包含MyPalette控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item->child(1), 1);
    //堆中创建的对象需要手动释放内存
    delete comboBox2;
  }

  //删除顶层节点的所有子节点
  removeChildren(item);
  //从QtreeWidget中移除顶层节点并删除QTreeWidgetItem对象
  treeWidget->takeTopLevelItem(treeWidget->indexOfTopLevelItem(item));
  delete item;
}




/**
 * @brief 从QTreeidget移除Path顶层节点
 * @param QTreeWidget* treeWidget ———— 待移除Path顶层节点的QTreeWidget指针
 * @param QTreeWidgetItem* item   ———— 待移除的QTreeWidgetItem指针
 */
void MainWindow::remove_TreeWidgetItem_Path(QTreeWidget* treeWidget, QTreeWidgetItem* item)
{
  //删除rviz_common::Display对象
  myviz->DeleteDisplayLayer(m_LayerHash.value(item));
  //从哈希表中移除键值对
  m_LayerHash.remove(item);
  //获取第二列单元格中的QCheckBox控件
  QCheckBox* checkBox = qobject_cast<QCheckBox*>(treeWidget->itemWidget(item, 1));
  if(checkBox)
  {
    //如果该单元格中包含QCheckBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item, 1);
    //堆中创建的对象需要手动释放内存
    delete checkBox;
  }
  //获取子节点1(topic)第二列单元格中的QComboBox控件
  QComboBox* comboBox = qobject_cast<QComboBox*>(treeWidget->itemWidget(item->child(0), 1));
  if(comboBox)
  {
    //如果该单元格中包含QSpinBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item->child(0), 1);
    //堆中创建的对象需要手动释放内存
    delete comboBox;
  }
  //获取子节点2(color)第二列单元格中的MyPalette控件
  MyPalette* palette = qobject_cast<MyPalette*>(treeWidget->itemWidget(item->child(1), 1));
  if(palette)
  {
    //如果该单元格中包含MyPalette控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item->child(1), 1);
    //堆中创建的对象需要手动释放内存
    delete palette;
  }

  //删除顶层节点的所有子节点
  removeChildren(item);
  //从QtreeWidget中移除顶层节点并删除QTreeWidgetItem对象
  treeWidget->takeTopLevelItem(treeWidget->indexOfTopLevelItem(item));
  delete item;
}



/**
 * @brief 从QTreeidget移除PointStamped顶层节点
 * @param QTreeWidget* treeWidget ———— 待移除PointStamped顶层节点的QTreeWidget指针
 * @param QTreeWidgetItem* item   ———— 待移除的QTreeWidgetItem指针
 */
void MainWindow::remove_TreeWidgetItem_PointStamped(QTreeWidget* treeWidget, QTreeWidgetItem* item)
{
  //删除rviz_common::Display对象
  myviz->DeleteDisplayLayer(m_LayerHash.value(item));
  //从哈希表中移除键值对
  m_LayerHash.remove(item);
  //获取第二列单元格中的QCheckBox控件
  QCheckBox* checkBox = qobject_cast<QCheckBox*>(treeWidget->itemWidget(item, 1));
  if(checkBox)
  {
    //如果该单元格中包含QCheckBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item, 1);
    //堆中创建的对象需要手动释放内存
    delete checkBox;
  }
  //获取子节点1(topic)第二列单元格中的QComboBox控件
  QComboBox* comboBox = qobject_cast<QComboBox*>(treeWidget->itemWidget(item->child(0), 1));
  if(comboBox)
  {
    //如果该单元格中包含QSpinBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item->child(0), 1);
    //堆中创建的对象需要手动释放内存
    delete comboBox;
  }
  //获取子节点2(color)第二列单元格中的MyPalette控件
  MyPalette* palette = qobject_cast<MyPalette*>(treeWidget->itemWidget(item->child(1), 1));
  if(palette)
  {
    //如果该单元格中包含MyPalette控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item->child(1), 1);
    //堆中创建的对象需要手动释放内存
    delete palette;
  }

  //删除顶层节点的所有子节点
  removeChildren(item);
  //从QtreeWidget中移除顶层节点并删除QTreeWidgetItem对象
  treeWidget->takeTopLevelItem(treeWidget->indexOfTopLevelItem(item));
  delete item;
}



/**
 * @brief 从QTreeidget移除Marker顶层节点
 * @param QTreeWidget* treeWidget ———— 待移除Marker顶层节点的QTreeWidget指针
 * @param QTreeWidgetItem* item   ———— 待移除的QTreeWidgetItem指针
 */
void MainWindow::remove_TreeWidgetItem_Marker(QTreeWidget* treeWidget, QTreeWidgetItem* item)
{
  //删除rviz_common::Display对象
  myviz->DeleteDisplayLayer(m_LayerHash.value(item));
  //从哈希表中移除键值对
  m_LayerHash.remove(item);
  //获取第二列单元格中的QCheckBox控件
  QCheckBox* checkBox = qobject_cast<QCheckBox*>(treeWidget->itemWidget(item, 1));
  if(checkBox)
  {
    //如果该单元格中包含QCheckBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item, 1);
    //堆中创建的对象需要手动释放内存
    delete checkBox;
  }
  //获取子节点1(topic)第二列单元格中的QComboBox控件
  QComboBox* comboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(item->child(0), 1));
  if(comboBox)
  {
    //如果该单元格中包含QSpinBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item->child(0), 1);
    //堆中创建的对象需要手动释放内存
    delete comboBox;
  }

  //删除顶层节点的所有子节点
  removeChildren(item);
  //从QtreeWidget中移除顶层节点并删除QTreeWidgetItem对象
  treeWidget->takeTopLevelItem(treeWidget->indexOfTopLevelItem(item));
  delete item;
}



/**
 * @brief 从QTreeidget移除MarkerArray顶层节点
 * @param QTreeWidget* treeWidget ———— 待移除MarkerArray顶层节点的QTreeWidget指针
 * @param QTreeWidgetItem* item   ———— 待移除的QTreeWidgetItem指针
 */
void MainWindow::remove_TreeWidgetItem_MarkerArray(QTreeWidget* treeWidget, QTreeWidgetItem* item)
{
  //删除rviz_common::Display对象
  myviz->DeleteDisplayLayer(m_LayerHash.value(item));
  //从哈希表中移除键值对
  m_LayerHash.remove(item);
  //获取第二列单元格中的QCheckBox控件
  QCheckBox* checkBox = qobject_cast<QCheckBox*>(treeWidget->itemWidget(item, 1));
  if(checkBox)
  {
    //如果该单元格中包含QCheckBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item, 1);
    //堆中创建的对象需要手动释放内存
    delete checkBox;
  }
  //获取子节点1(topic)第二列单元格中的QComboBox控件
  QComboBox* comboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget(item->child(0), 1));
  if(comboBox)
  {
    //如果该单元格中包含QSpinBox控件，则先将其从QTreeWidget控件中移除
    treeWidget->removeItemWidget(item->child(0), 1);
    //堆中创建的对象需要手动释放内存
    delete comboBox;
  }

  //删除顶层节点的所有子节点
  removeChildren(item);
  //从QtreeWidget中移除顶层节点并删除QTreeWidgetItem对象
  treeWidget->takeTopLevelItem(treeWidget->indexOfTopLevelItem(item));
  delete item;
}



/**
 * @brief 设置导航初始点槽函数
 */
void MainWindow::slot_set_initial_pose()
{
  myviz->SetInitialPose();
}



/**
 * @brief 设置导航目标点槽函数
 */
void MainWindow::slot_set_goal_pose()
{
  myviz->SetGoalPose();
}



/**
 * @brief 在MainWindow界面更新实时位置槽函数
 */
void MainWindow::slot_update_pos(double x, double y, double z)
{
  ui.pose_x->setText(QString::number(x));
  ui.pose_y->setText(QString::number(y));
  ui.pose_z->setText(QString::number(z));
  currRobotPosition[0] = x;
  currRobotPosition[1] = y;
}



/**
 * @brief 发布点位槽函数
 */
void MainWindow::slot_publish_point()
{
  myviz->PublishPoint();
}



/**
 * @brief 任务路径预览槽函数
 */
void MainWindow::slot_task_path_preview(const QString& taskName)
{
  QTaskHandle th;     //创建任务处理句柄
  QTask* task = th.query_task_by_name(taskName);
  if(task){
    bool noNeedToAddPathLayer = false;                  //使用一个bool值保存是否需要新增Path图层用于显示任务路径预览
    QTreeWidgetItemIterator it(ui.treeWidget_layer);    //使用一个迭代器遍历顶层节点
    while(*it){
      if((*it)->data(0, Qt::UserRole) == AddLayer::path){
        QComboBox* pathTopicComboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget((*it)->child(0), 1));
        if(pathTopicComboBox && pathTopicComboBox->currentText() == "/task_path_preview")
          noNeedToAddPathLayer = true;
      }
      it++;
    }
    if(!noNeedToAddPathLayer){
      //新建一个路径图层
      QString displayName = "任务路径发布预览";
      QString topicName = "/task_path_preview";
      add_TreeWidgetItem_Path(ui.treeWidget_layer, displayName, topicName, QColor(255, 48, 2));
    }

    qnode.publish_task_path_preview(task->path());
  }
  else{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = rclcpp::Clock().now();
    qnode.publish_task_path_preview(path);
  }
}



/**
 * @brief 生成任务槽函数
 */
void MainWindow::slot_generate_task()
{
  if(ui.btn_GenerateTask->isChecked()){
    this->isGenerateTask = true;
    modeTypes.clear();
    qnode.set_generate_task_state(isGenerateTask);
    ui.btn_GenerateTask->setIcon(QIcon("://taskManagement/cancel.svg"));
    ui.btn_GenerateTask->setText("取消生成");
    ui.widget_SaveTask->setVisible(true);

    bool noNeedToAddMarkerLayer = false;    //使用一个bool值保存是否需要新增Marker图层用于显示用户点击的点
    bool noNeedToAddPathLayer = false;      //使用一个bool值保存是否需要新增Path图层用于显示用户创建的路径
    QTreeWidgetItemIterator it(ui.treeWidget_layer);    //使用一个迭代器遍历顶层节点
    while(*it){
      if((*it)->data(0, Qt::UserRole) == AddLayer::marker){
        QComboBox* markerTopicComboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget((*it)->child(0), 1));
        if(markerTopicComboBox && markerTopicComboBox->currentText() == "/visualization_marker")
          noNeedToAddMarkerLayer = true;
      }
      else if((*it)->data(0, Qt::UserRole) == AddLayer::path){
        QComboBox* pathTopicComboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget((*it)->child(0), 1));
        if(pathTopicComboBox && pathTopicComboBox->currentText() == "/global_plan")
          noNeedToAddPathLayer = true;
      }
      it++;
    }
    if(!noNeedToAddMarkerLayer){
      //新建一个标记图层
      QString displayName("任务路径点");
      QString topicName("/visualization_marker");
      add_TreeWidgetItem_Marker(ui.treeWidget_layer, displayName, topicName);
    }
    if(!noNeedToAddPathLayer){
      //新建一个路径图层
      QString displayName = "任务路径";
      QString topicName = "/global_plan";
      add_TreeWidgetItem_Path(ui.treeWidget_layer, displayName, topicName);
    }
    QSystemTrayIcon trayIcon;
    trayIcon.setIcon(QIcon("://icon/prompt.svg"));
    trayIcon.show();
    trayIcon.showMessage("提示", "使用「发布点位」工具创建一条任务路径吧！\n\r tips：选中“使用标准点”可以使用保存的标准点规划路径～", QSystemTrayIcon::Information, 2000);
  }
  else{
    this->isGenerateTask = false;
    qnode.set_generate_task_state(isGenerateTask);
    ui.btn_GenerateTask->setIcon(QIcon("://taskManagement/generate_task.svg"));
    ui.btn_GenerateTask->setText("生成任务");
    ui.widget_SaveTask->setVisible(false);
  }
}



/**
 * @brief 保存任务槽函数
 */
void MainWindow::slot_save_task()
{
  if(!modeTypes.isEmpty()){
    nav_msgs::msg::Path path = qnode.slot_publish_global_plan();
    QString startPointName(qnode.start_point_name_search());
    QString endPointName(qnode.end_point_name_search());
    qnode.clear_marker_and_path();
    //GenerateTask::saveTaskToFile(path, qnode.get_yaml_file_path());
    gt_ptr = new GenerateTask(path, modeTypes, qnode.get_yaml_file_path());
    connect(gt_ptr, &GenerateTask::taskUpdated, this, &MainWindow::slot_on_task_updated, Qt::UniqueConnection);
    gt_ptr->setStartPointName(startPointName);
    gt_ptr->setEndPointName(endPointName);
    gt_ptr->exec();
    delete gt_ptr;
    this->isGenerateTask = false;
    qnode.set_generate_task_state(isGenerateTask);
    ui.btn_GenerateTask->setIcon(QIcon("://taskManagement/generate_task.svg"));
    ui.btn_GenerateTask->setText("生成任务");
    ui.btn_GenerateTask->setChecked(isGenerateTask);
    ui.widget_SaveTask->setVisible(false);
  }
}



/***********************************************************************************************
 *                                      打开地图相关方法定义                                       *
 ***********************************************************************************************/
/**
 * @brief QProcess启动进程错误处理槽函数
 * @param QProcess::ProcessError errorcode  ———— 错误代码
 */
void MainWindow::on_QProcessErrorOccurred(QProcess::ProcessError errorcode)
{
  switch (errorcode)
  {
    case QProcess::FailedToStart:
      qnode.log(QNode::Error, std::string("进程启动失败！未找到程序或者缺少可执行权限"));
    break;
    case QProcess::Crashed:
      qnode.log(QNode::Error, std::string("进程运行中发生崩溃！"));
    break;
    case QProcess::Timedout:
      qnode.log(QNode::Error, std::string("启动超时！"));
    break;
    case QProcess::ReadError:
      qnode.log(QNode::Error, std::string("尝试从进程中读取时发生错误！"));
    break;
    case QProcess::WriteError:
      qnode.log(QNode::Error, std::string("尝试向进程中写入时发生错误！"));
    break;
    case QProcess::UnknownError:
      qnode.log(QNode::Error, std::string("未知错误！"));
    break;
  }
}



/**
 * @brief QProcess打开地图槽函数
 */
void MainWindow::QProcessOpenMap()
{
  QSystemTrayIcon trayIcon;
  trayIcon.setIcon(QIcon("://mapManager/OpenMap.svg"));
  trayIcon.show();
  if(rclcpp::ok())
  {
    //向ROS参数服务器写入yaml文件路径
    qnode.set_yaml_file_path(yamlFilePath);
    if(mapServerProcess->state() == QProcess::Running)
    {
      //优雅地退出进程（发送SIGTERM信号）
      mapServerProcess->terminate();
      if(!mapServerProcess->waitForFinished(5000)){
        mapServerProcess->kill();    //如果5秒内没有退出，强制终止进程
      }
    }
    mapServerProcess->start("ros2", QStringList()
                            << "run" << "nav2_map_server" << "map_server"
                            << "--ros-args" << "-p" << QString("yaml_filename:=%1").arg(yamlFilePath));
    mapServerProcess->waitForStarted();
  }
  else
    trayIcon.showMessage("提示", "请先启动 ROS 2 节点", QSystemTrayIcon::Warning, 2000);
}



/**
 * @brief system()打开地图槽函数(打开一个终端执行命令)
 */
void MainWindow::GnomeOpenMap()
{
  QSystemTrayIcon trayIcon;
  trayIcon.setIcon(QIcon("://mapManager/OpenMap.svg"));
  trayIcon.show();
  if(rclcpp::ok())
  {
    //向ROS参数服务器写入yaml文件路径
    qnode.set_yaml_file_path(yamlFilePath);
    //命令字符串(命令字符串后面加上";read"，这样可以让终端等待用户输入回车键才关闭)
    QString str("gnome-terminal -- bash -c 'ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=%1;read'&");
    //脚本文件绝对路径插入到命令中
    str = str.arg(yamlFilePath);
    //执行命令
    std::system(str.toLatin1().data());
  }
  else
    trayIcon.showMessage("提示", "请先启动 ROS 2 节点", QSystemTrayIcon::Warning, 2000);
}


/**
 * @brief 打开地图后处理槽函数(跳转到“地图页面”并自动新增一个地图图层)
 */
void MainWindow::slot_map_opened()
{
  //跳转至地图界面
  ui.stackedWidget->setCurrentWidget(ui.Map);
  ui.docktreeWidget->setHidden(false);
  ui.dockTaskManagerWidget->setHidden(false);
  ui.rViztoolBar->setHidden(false);
  bool noNeedToAddMapLayer = false;    //使用一个bool值保存是否需要新增地图图层用于显示地图
  QTreeWidgetItemIterator it(ui.treeWidget_layer);    //使用一个迭代器遍历顶层节点
  while(*it){
    if((*it)->data(0, Qt::UserRole) == AddLayer::map){
      QComboBox* mapTopicComboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget((*it)->child(0), 1));
      if(mapTopicComboBox && mapTopicComboBox->currentText() == "/map"){
        noNeedToAddMapLayer = true;
        break;
      }
    }
    it++;
  }
  if(!noNeedToAddMapLayer){
    //新建一个地图图层
    AddLayer addlayer;
    QString displayName("地图");
    QString topicName("/map");
    add_TreeWidgetItem_Map(ui.treeWidget_layer, displayName, topicName);
  }
  QString path_to_json = qnode.get_yaml_file_path().remove((qnode.get_yaml_file_path().size() - 5), 5).append("Task.json");
  OpenTasks(path_to_json);                //加载打开地图的所有任务
  path_to_json = path_to_json.remove((path_to_json.size() - 9), 9).append("Point.json");
  LoadStandardPoints(path_to_json);       //加载打开地图的所有标准点
  ui.checkBox_ShowAllStandardPoints->setChecked(false);  //默认不显示所有标准点
}



/***********************************************************************************************
 *                                      打开任务和加载点位相关方法定义                              *
 ***********************************************************************************************/
/**
 * @brief 加载打开地图的所有任务
 * @param 任务文件路径path_to_json
 */
void MainWindow::OpenTasks(QString path_to_json)
{
  QString _temp = path_to_json;
  QString pgmFilePath = _temp.remove((_temp.size() - 9), 9).append(".pgm");    //获取当前打开的地图的pgm文件路径
  QFileInfo pgmFileInfo(pgmFilePath);
  if(pgmFileInfo.exists() && pgmFileInfo.isFile())
  {
    //任务管理窗口显示当前打开地图
    QPixmap image(pgmFilePath);                               //初始化一个pixmap对象
    image = image.scaledToWidth(100);                         //限制图片大小
    ui.label_currOpenedMap->setPixmap(image);                 //设置pixmap
  }
  ui.TaskChoose->clear();     //清空任务选择栏
  ui.TaskChoose->addItem("null");
  //清空任务栏窗口
  remove_TableWidgetAllRows(ui.tableWidget_taskManager);
  //创建任务处理句柄
  QTaskHandle th;
  //每打开一个地图，清空一次任务数组并重新向其中填充当前打开地图的所有任务
  th.clear_task_vector();
  //判断任务JSON文件是否真的存在
  QFileInfo fileInfo(path_to_json);
  if(fileInfo.exists() && fileInfo.isFile()){
    //打开jsonline格式的文件(jsonline是一种存储多个json对象的格式，每个对象占一行，没有逗号分隔)
    QFile file(path_to_json);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)){
      QMessageBox::critical(this, "ERROR", "无法打开文件");
      return;
    }
    //创建文本流
    QTextStream stream(&file);
    //按行读取每个json对象
    while(!stream.atEnd()){
      QString line = stream.readLine();
      //解析json数据
      QJsonParseError error;
      //根据jsonline创建JSON文档
      QJsonDocument doc = QJsonDocument::fromJson(line.toUtf8(), &error);
      if(error.error != QJsonParseError::NoError){
        QMessageBox::critical(this, "ERROR", QString("解析错误：").append(error.errorString()));
        continue;
      }
      //获取json对象
      QJsonObject taskJson = doc.object();
      //根据json对象创建任务对象并将其填充到任务数组的尾部
      QTask task(taskJson);
      th.push_back_task(task);
      //根据键值获取对应的数据
      QString taskID = taskJson.value("id").toString();
      QString taskName = taskJson.value("name").toString();
      ui.TaskChoose->addItem(taskName);
      QString start = taskJson.value("start").toString();
      QString finish = taskJson.value("finish").toString();
      QString taskDescription = taskJson.value("description").toString();
      add_TableWidgetRow(ui.tableWidget_taskManager, taskID, taskName, start, finish, taskDescription);
    }
  }
}



/**
 * @brief 任务更新槽函数(每当用户保存了一个任务，界面实时更新)
 */
void MainWindow::slot_on_task_updated(QString taskname)
{
  QTaskHandle th;
  QString id = th.last().id();
  QString name = th.last().name();
  QString start = th.last().start();
  QString finish = th.last().finish();
  QString description = th.last().description();
  ui.TaskChoose->addItem(taskname);
  add_TableWidgetRow(ui.tableWidget_taskManager, id, name, start, finish, description);
}



/**
 * @brief 加载打开地图的所有标准点
 * @param 标准点文件路径path_to_json
 */
void MainWindow::LoadStandardPoints(QString path_to_json)
{
  //首先清空标准点数组
  qnode.standardPoints_clear();
  //判断任务JSON文件是否真的存在
  QFileInfo fileInfo(path_to_json);
  if(fileInfo.exists() && fileInfo.isFile()){
    //打开jsonline格式的文件(jsonline是一种存储多个json对象的格式，每个对象占一行，没有逗号分隔)
    QFile file(path_to_json);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)){
      QMessageBox::critical(this, "ERROR", "无法打开文件");
      return;
    }
    //创建文本流
    QTextStream stream(&file);
    //按行读取每个json对象
    while(!stream.atEnd()){
      QString line = stream.readLine();
      //解析json数据
      QJsonParseError error;
      //根据jsonline创建JSON文档
      QJsonDocument doc = QJsonDocument::fromJson(line.toUtf8(), &error);
      if(error.error != QJsonParseError::NoError){
        QMessageBox::critical(this, "ERROR", QString("解析错误：").append(error.errorString()));
        continue;
      }
      //获取json对象
      QJsonObject pointJson = doc.object();
      QJsonArray coordinate = pointJson.value("coordinate").toArray();
      StandardPoint sp(pointJson.value("pointName").toString(), coordinate.at(0).toDouble(), coordinate.at(1).toDouble());
      qnode.standardPoints_append(sp);
    }
  }
}



/**
 * @brief 标准点更新槽函数(每当用户保存了一个标准点，界面实时更新)
 */
void MainWindow::slot_on_point_updated(StandardPoint& newPoint)
{
  qnode.standardPoints_append(newPoint);
  if(!ui.checkBox_ShowAllStandardPoints->isChecked())
    ui.checkBox_ShowAllStandardPoints->setChecked(true);
  else
    slot_show_all_standard_points(true);
}



/**
 * @brief 显示所有标准点位槽函数
 */
void MainWindow::slot_show_all_standard_points(bool visible)
{
  if(visible){
    bool noNeedToAddLayer = false;    //使用一个bool值保存是否需要新增图层用于显示所有标准点
    QTreeWidgetItemIterator it(ui.treeWidget_layer);    //使用一个迭代器遍历顶层节点
    while(*it){
      if((*it)->data(0, Qt::UserRole) == AddLayer::marker){
        QComboBox* markerTopicComboBox = qobject_cast<QComboBox*>(ui.treeWidget_layer->itemWidget((*it)->child(0), 1));
        if(markerTopicComboBox && markerTopicComboBox->currentText() == "/standard_point"){
          noNeedToAddLayer = true;
          break;
        }
      }
      it++;
    }
    if(!noNeedToAddLayer){
      //新建一个标记图层
      QString displayName("标准点");
      QString topicName("/standard_point");
      add_TreeWidgetItem_Marker(ui.treeWidget_layer, displayName, topicName);
    }
  }
  qnode.show_all_standard_points(visible);
}



/***********************************************************************************************
 *                             向UI界面添加一个地图管理相关控件方法定义                               *
 ***********************************************************************************************/
/**
 * @brief 向TableWidget中添加一行管理一个地图的相关控件
 * @param QTableWidget* tablewidget ———— 待添加行的QTableWidget控件指针
 * @param QString& mapPath          ———— 地图路径(.pgm文件路径)
 * @param QString& mapName          ———— 地图名称(由用户自行决定)
 * @param QString& mapDescription   ———— 地图描述(由用户自行决定)
 */
void MainWindow::add_TableWidgetRow(QTableWidget* tablewidget, QString& mapPath, QString& mapName, QString mapDescription)
{
  //先取表格总行数
  int row_count = tablewidget->rowCount();
  //在末尾插入新行
  tablewidget->insertRow(row_count);
  //为该行每个单元格分配单元格对象(QtableWidgetItem：可以包含文本setText、图标setIcon或者复选框setCheckState)
  for(int i = 0; i < tablewidget->columnCount(); i++)
    tablewidget->setItem(row_count, i, new QTableWidgetItem());

  //创建一个QLabel对象用于显示地图的缩略图(或者叫预览图)
  QLabel* image_label = new QLabel();
  //首先检查pgm文件是否存在
  QFileInfo pgmFileInfo(mapPath);
  //QFileInfo::exists()用于检查文件是否存在；QFileInfo::isFile()用于检查文件是否是普通文件(不是目录或者符号链接)
  if(pgmFileInfo.exists() && pgmFileInfo.isFile()){
    //使用地图文件的绝对路径创建一个QPixmap象图对象
    QPixmap image(mapPath);
    //限制图片大小为100*100
    image = image.scaledToWidth(100);
    //设置QLabel对象的pixmap
    image_label->setPixmap(image);
    //设置QLabel对象的对齐方式为居中对齐
    image_label->setAlignment(Qt::AlignCenter);
  }
  else{
    image_label->setText("(加载预览图失败)");
    QFont font = image_label->font();
    font.setPointSize(8);
    image_label->setFont(font);
    image_label->setAlignment(Qt::AlignCenter);
  }
  //将QLabel对象设置到当前行的第一列(image_label本不属于tablewidget，就像两寸照片本不属于登记表的照片栏)
  tablewidget->setCellWidget(row_count, 0, image_label);


  //设置该行第二列和第三列单元格信息
  tablewidget->item(row_count, 1)->setText(mapName);
  tablewidget->item(row_count, 1)->setTextAlignment(Qt::AlignCenter);
  tablewidget->item(row_count, 2)->setText(mapDescription);
  tablewidget->item(row_count, 2)->setTextAlignment(Qt::AlignCenter);

  //新建两个按钮用于打开地图和取消地图管理
  QPushButton *btn_OpenMap = new QPushButton(QIcon("://mapManager/OpenMap.svg"), "打开地图");
  QPushButton *btn_CancelManage = new QPushButton(QIcon("://mapManager/CancelManage.svg"), "取消管理");
  btn_OpenMap->setMinimumWidth(150);
  btn_CancelManage->setMinimumWidth(150);
  btn_OpenMap->setIconSize(QSize(20, 20));
  btn_CancelManage->setIconSize(QSize(20, 20));
  //在地图(.pgm)绝对路径路径的基础上删掉最后三个字符并附加"yaml"四个字符就得到了yaml文件的绝对路径
  QString mapFilePath = mapPath.remove((mapPath.size() - 3), 3).append("yaml");
  //将yaml文件的绝对路径存储到“打开地图”按钮的ToolTip中
  btn_OpenMap->setToolTip(mapFilePath);
  //连接“打开地图”按钮点击信号与QProcess打开地图槽函数
  connect(btn_OpenMap, &QPushButton::clicked, this, [=](){
    btn_OpenMap->parentWidget()->setFocus();
    mapFileName = mapName;      //地图名称信息(根据用户输入获得)
    this->yamlFilePath = btn_OpenMap->toolTip();
    QFileInfo fileInfo(this->yamlFilePath);
    //QFileInfo::exists()用于检查文件是否存在；QFileInfo::isFile()用于检查文件是否是普通文件(不是目录或者符号链接)
    if(fileInfo.exists() && fileInfo.isFile())
      //QProcessOpenMap();
      GnomeOpenMap();
    else{
      QMessageBox::critical(this, "错误", "未找到指定目录下的地图文件(文件不存在或者目录错误)");
    }
  });
  //连接“取消管理”按钮点击信号与移除行槽函数
  connect(btn_CancelManage, &QPushButton::clicked, this, [=](){
    //让按钮所在的表格项获得焦点(按钮的父widget获得焦点，因为父widget通过setCellWidget方法设置到表格项上了，所以表格项也会获得焦点)
    btn_CancelManage->parentWidget()->setFocus();
    delete_TableWidgetRow(ui.tableWidget_mapManager, ui.tableWidget_mapManager->currentRow());
  });
  //因为表格接受添加widget，所以要把这两个按钮布局后放到一个widget里面去，这里两个按钮选择纵向布局
  QVBoxLayout *layout = new QVBoxLayout();
  layout->addWidget(btn_OpenMap);
  layout->addWidget(btn_CancelManage);
  //设置内容边距为0
  layout->setContentsMargins(0, 0, 0, 0);
  //设置对齐方式为居中对齐
  layout->setAlignment(Qt::AlignCenter);
  //创建一个QWidget对象
  QWidget *widget = new QWidget();
  //设置QWidget对象布局
  widget->setLayout(layout);
  tablewidget->setCellWidget(row_count, 3, widget);
}



/**
 * @brief 从TableWidget中移除一行(取消管理一个地图)
 * @attention 该方法是取消管理方法，由每行的“取消管理”按钮的点击信号连接该槽函数
 * @param QTableWidget* tablewidget ———— 待移除行的QTableWidget控件指针
 * @param int row                   ———— 待移除的行号
 */
void MainWindow::delete_TableWidgetRow(QTableWidget* tablewidget, int row)
{
  //取得该行第三列的那个QWidget对象
  QWidget* widget = tablewidget->cellWidget(row, 3);
  //取得该对象的布局管理器
  QLayout* tempLayout = widget->layout();
  //由于我们设置的是垂直布局，所以先把QLayout*类型的指针类型转换为QVBoxLayout*类型的指针
  QVBoxLayout* layout = qobject_cast<QVBoxLayout*>(tempLayout);
  //取出布局管理器中第一个项(即“打开地图”按钮)
  QWidget* tempWidget = tempLayout->itemAt(0)->widget();
  //同样做指针类型转换
  QPushButton* btn_OpenMap = qobject_cast<QPushButton*>(tempWidget);
  //取出布局管理器中第二个项(即“取消管理”按钮)
  tempWidget = tempLayout->itemAt(1)->widget();
  //指针类型转换
  QPushButton* btn_CancelManage = qobject_cast<QPushButton*>(tempWidget);
  //判断所有指针都不为空
  if(btn_CancelManage && btn_OpenMap && layout && widget)
  {
    layout->takeAt(1);            //移除UI
    delete btn_CancelManage;      //释放内存
    layout->takeAt(0);            //移除UI
    delete btn_OpenMap;           //释放内存
    //layout直接delete
    delete layout;                //释放内存
    tablewidget->removeCellWidget(row, 3);  //移除UI
    delete widget;                          //释放内存
  }
  //取出该行第一列的image_label
  tempWidget = tablewidget->cellWidget(row, 0);
  //指针类型转换
  QLabel* image_label = qobject_cast<QLabel*>(tempWidget);
  if(image_label)
  {
    //移除UI
    tablewidget->removeCellWidget(row, 0);
    //释放内存
    delete image_label;
  }

  //移除row行所有列的单元格对象并释放其内存
  for(int column = 0; column < tablewidget->columnCount(); column++)
    delete tablewidget->takeItem(row, column);
  //移除行UI
  tablewidget->removeRow(row);
}



/***********************************************************************************************
 *                             向UI界面添加一个任务管理相关控件方法定义                               *
 ***********************************************************************************************/
/**
 * @brief 向TableWidget中添加一行任务
 * @param QTableWidget* tablewidget ———— 待添加行的QTableWidget控件指针
 * @param QString& taskID           ———— 任务ID
 * @param QString& taskName         ———— 任务名称
 * @param QString &start            ———— 起始点
 * @param QString &finish           ———— 终止点
 * QString &taskDescription         ———— 任务描述
 */
void MainWindow::add_TableWidgetRow(QTableWidget* tablewidget, QString& taskID, QString& taskName, QString &start, QString &finish, QString &taskDescription)
{
  //先取表格总行数
  int row_count = tablewidget->rowCount();
  //在末尾插入新行
  tablewidget->insertRow(row_count);
  //为该行每个单元格分配单元格对象(QtableWidgetItem：可以包含文本setText、图标setIcon或者复选框setCheckState)
  for(int i = 0; i < tablewidget->columnCount(); i++)
    tablewidget->setItem(row_count, i, new QTableWidgetItem());

  //设置该行单元格信息
  tablewidget->item(row_count, 0)->setText(taskID);
  tablewidget->item(row_count, 0)->setTextAlignment(Qt::AlignCenter);
  tablewidget->item(row_count, 1)->setText(taskName);
  tablewidget->item(row_count, 1)->setTextAlignment(Qt::AlignCenter);
  tablewidget->item(row_count, 2)->setText(start);
  tablewidget->item(row_count, 2)->setTextAlignment(Qt::AlignCenter);
  tablewidget->item(row_count, 3)->setText(finish);
  tablewidget->item(row_count, 3)->setTextAlignment(Qt::AlignCenter);
  tablewidget->item(row_count, 4)->setText(taskDescription);
  tablewidget->item(row_count, 4)->setTextAlignment(Qt::AlignCenter);
}



/**
 * @brief 移除QTableWidget的所有行
 * @attention 需要注意的是移除行需要按倒序移除，至于为什么嘛其实自己想想也就明白啦~
 */
void MainWindow::remove_TableWidgetAllRows(QTableWidget* tablewidget)
{
  int rowsNum = tablewidget->rowCount();
  for (int row = rowsNum - 1; row >= 0; row--) {
    for(int line = 0; line < tablewidget->columnCount(); line++)
      delete tablewidget->takeItem(row, line);
    tablewidget->removeRow(row);
  }
}



/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>qviz_demo 1.0.0</h2><p>Copyright © 2023 NJUPT by XJY. All rights reserved.</p><p>这是一个基于 Qt 与 RViz2 的 ROS 2 可视化机器人上位机软件，可以实现基础的RViz数据可视化功能以及地图管理、任务管理、外部程序管理等。</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "class1_ros_qt_demo");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString ros_domain_id = settings.value("ros_domain_id", QString("0")).toString();
    QString ros_node_name = settings.value("ros_node_name", QString("qviz_demo")).toString();
    ui.line_edit_domain_id->setText(ros_domain_id);
    ui.line_edit_node_name->setText(ros_node_name);
    ui.line_edit_topic->setText("RMW/QoS/localhost 由环境变量控制");
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_domain_id->setEnabled(false);
    	ui.line_edit_node_name->setEnabled(false);
    }
    ui.lineEditCarFixedFrame->setText(settings.value("dual_map/car_fixed_frame", "map").toString());
    ui.lineEditCarMapTopic->setText(settings.value("dual_map/car_map_topic", "/map").toString());
    ui.lineEditDroneFixedFrame->setText(settings.value("dual_map/drone_fixed_frame", "map").toString());
    ui.lineEditDroneMapTopic->setText(settings.value("dual_map/drone_map_topic", "/map").toString());
    QStringList mapManagers_mapPath = settings.value("mapManagers_mapPath").toStringList();
    QStringList mapManagers_mapName = settings.value("mapManagers_mapName").toStringList();
    QStringList mapManagers_mapDescription = settings.value("mapManagers_mapDescription").toStringList();
    for (int i = 0; i < mapManagers_mapPath.count(); i++) {
      add_TableWidgetRow(ui.tableWidget_mapManager, mapManagers_mapPath[i], mapManagers_mapName[i], mapManagers_mapDescription[i]);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "class1_ros_qt_demo");
    settings.setValue("ros_domain_id",ui.line_edit_domain_id->text());
    settings.setValue("ros_node_name",ui.line_edit_node_name->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
    settings.setValue("dual_map/car_fixed_frame", ui.lineEditCarFixedFrame->text());
    settings.setValue("dual_map/car_map_topic", ui.lineEditCarMapTopic->text());
    settings.setValue("dual_map/drone_fixed_frame", ui.lineEditDroneFixedFrame->text());
    settings.setValue("dual_map/drone_map_topic", ui.lineEditDroneMapTopic->text());
    QStringList mapManagers_mapPath;
    QStringList mapManagers_mapName;
    QStringList mapManagers_mapDescription;
    if(ui.tableWidget_mapManager->rowCount() > 0)
    {
      for(int row = 0; row < ui.tableWidget_mapManager->rowCount(); row++)
      {
        QString mapPath = ui.tableWidget_mapManager->cellWidget(row, 3)->layout()->itemAt(0)->widget()->toolTip();
        mapManagers_mapPath.append(mapPath.remove((mapPath.size() - 4), 4).append("pgm"));
        mapManagers_mapName.append(ui.tableWidget_mapManager->item(row, 1)->text());
        mapManagers_mapDescription.append(ui.tableWidget_mapManager->item(row, 2)->text());
      }
    }
    settings.setValue("mapManagers_mapPath", mapManagers_mapPath);
    settings.setValue("mapManagers_mapName", mapManagers_mapName);
    settings.setValue("mapManagers_mapDescription", mapManagers_mapDescription);
}

bool MainWindow::eventFilter(QObject *watched, QEvent *event)
{
  QLabel* label = qobject_cast<QLabel*>(watched);
  if (label != nullptr && zoomablePreviewPixmaps.contains(label) && event->type() == QEvent::Wheel) {
    QWheelEvent* wheelEvent = static_cast<QWheelEvent*>(event);
    const double step = wheelEvent->angleDelta().y() > 0 ? 1.12 : 0.89;
    const double currentScale = zoomablePreviewScales.value(label, 1.0);
    zoomablePreviewScales[label] = qBound(0.2, currentScale * step, 8.0);
    refreshZoomablePreview(label);
    return true;
  }
  return QMainWindow::eventFilter(watched, event);
}

void MainWindow::registerZoomablePreview(QLabel* label)
{
  if (label == nullptr) {
    return;
  }
  label->installEventFilter(this);
  label->setMouseTracking(true);
  zoomablePreviewScales.insert(label, 1.0);
}

void MainWindow::setZoomablePreview(QLabel* label, const QPixmap& pixmap)
{
  if (label == nullptr || pixmap.isNull()) {
    return;
  }
  zoomablePreviewPixmaps[label] = pixmap;
  zoomablePreviewScales[label] = 1.0;
  refreshZoomablePreview(label);
}

void MainWindow::refreshZoomablePreview(QLabel* label)
{
  if (label == nullptr || !zoomablePreviewPixmaps.contains(label)) {
    return;
  }

  const QPixmap original = zoomablePreviewPixmaps.value(label);
  if (original.isNull()) {
    return;
  }

  const QSize targetSize = label->size();
  if (!targetSize.isValid()) {
    return;
  }

  const QSize fittedSize = original.size().scaled(targetSize, Qt::KeepAspectRatio);
  const double scale = zoomablePreviewScales.value(label, 1.0);
  const QSize scaledSize = QSize(
    qMax(1, qRound(fittedSize.width() * scale)),
    qMax(1, qRound(fittedSize.height() * scale))
  );

  label->setPixmap(original.scaled(scaledSize, Qt::KeepAspectRatio, Qt::SmoothTransformation));
  label->setText(QString());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
  QMainWindow::resizeEvent(event);
  updateFpvOverlayGeometry();
  updateDualMapVideoOverlayGeometry();
  refreshZoomablePreview(ui.labelStitchingPreview);
  refreshZoomablePreview(ui.labelUxodDetectionPreview);
  refreshZoomablePreview(ui.labelStrikeMissionMosaic);
}

}  // namespace class1_ros_qt_demo
