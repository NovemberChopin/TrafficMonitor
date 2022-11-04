
#include <QtGui>
#include <QMessageBox>
#include <QPushButton>
#include <QDebug>
#include <QString>
#include <QLabel>
#include <QStringList>
#include <QHeaderView>
#include <QDesktopWidget>
#include <iostream>
#include "../include/mul_t/main_window.hpp"
#include "../include/mul_t/config_panel.hpp"
#include "../include/mul_t/qnode.hpp"
#include <time.h>


namespace mul_t {

using namespace Qt;

MainWindow::~MainWindow() {}

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc, argv)
{
	ui.setupUi(this); 
    configP = new ConfigPanel();        // 初始化 connect 页面
    trafficD = new TrafficDetail();     // 交通事件展示对话框
    objectD = new ObjectDetection(4);    // 初始化 检测对象对象

    hasLoadCameraMatrix = false;        // 当前未加载相加参数
    image_size = cv::Size(1280, 720);
    cam_num = 4;        // 临时设置相机数量
    event_num = 5;
    interval = 5;       // 物体检测间隔

    std::vector<Rect> temp;
    std::vector<std::vector<double>> temp_p;
    std::vector<double> temp_pp;
    temp_pp.push_back(0.0);
    temp_pp.push_back(0.0);
    temp_p.push_back(temp_pp);
    temp_p.push_back(temp_pp);
    cur_frame0 = cv::imread("./test.png");
    cur_frame1 = cv::imread("./test.png");
    cur_frame2 = cv::imread("./test.png");
    cur_frame3 = cv::imread("./test.png");

    Mat m(3, 5, CV_32FC1, Scalar(1));
    for (int i=0; i < cam_num; i++) {       // 初始化 ROI
        // cur_frame.push_back(fill_img);
        this->vec_hasLoadCameraMatrix.push_back(false);
        this->vec_cameraMatrix.push_back(m);
        this->vec_distCoeffs.push_back(m);
        this->vec_rotationMatrix.push_back(m);
        this->vec_transVector.push_back(m);
        this->vec_map1.push_back(m);
        this->vec_map2.push_back(m);
        this->vec_cameraCoord.push_back(cv::Point3f(0.12, -2.9, 0));  
        temp.clear();
        for (int j=0; j < event_num; j++) {
            temp.push_back(cv::Rect(0, 0, 0, 0));
        }
        vec_roi.push_back(temp);
        vec_line.push_back(temp_p);
    }
    
    // QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(ui.btn_config_ros, &QPushButton::clicked, this, &MainWindow::showConfigPanel);
    QObject::connect(ui.btn_quit, &QPushButton::clicked, this, &MainWindow::exit);
    // QTableWidget 单元格点击事件 SLOT
    QObject::connect(ui.consoleTable ,SIGNAL(itemClicked(QTableWidgetItem*)), this, SLOT(consoleClick(QTableWidgetItem*)));
    QObject::connect(&qnode, SIGNAL(getImage(cv::Mat, int)), this, SLOT(setImage(cv::Mat, int)));
    
    //接收登录页面传来的数据
    QObject::connect(configP, SIGNAL(getConfigInfo(ConfigInfo*)), this, SLOT(connectByConfig(ConfigInfo*)));
    QObject::connect(trafficD, SIGNAL(getROI(QRect, int, int)), this, SLOT(setROI(QRect, int, int)));
    QObject::connect(trafficD, SIGNAL(getLine(double, double, int, int)), this, SLOT(setLine(double, double, int, int)));
    // 初始化
    initial();
}

/**
 * @brief 初始化图像界面相关
 * 
 */
void MainWindow::initial() {
    setWindowIcon(QIcon(":/images/icon.png"));
    setWindowTitle(tr("视频场景监控"));
    
    // 设置页面以全屏显示
    QDesktopWidget desktop;
    int screenX = desktop.availableGeometry().width();
    int screenY = desktop.availableGeometry().height();
    this->resize(screenX, screenY);
    // QLabel * label = new QLabel();
    // QRect *re = label->geometry();
    
    // 添加右键弹出菜单
    m_pOptMenu = new QMenu(this);
    m_pConfigAction = new QAction(QStringLiteral("配置相机"), this);
    m_pLoadAction = new QAction(QStringLiteral("加载配置"), this);
    m_pSaveAction = new QAction(QStringLiteral("保存配置"), this);
    m_pOptMenu->addAction(m_pConfigAction);
	m_pOptMenu->addAction(m_pLoadAction);
    m_pOptMenu->addAction(m_pSaveAction);
    ui.camera->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(m_pConfigAction, &QAction::triggered, this, &MainWindow::menu_pop_config);
    QObject::connect(m_pLoadAction, &QAction::triggered, this, &MainWindow::menu_pop_load_config);
    QObject::connect(m_pSaveAction, &QAction::triggered, this, &MainWindow::menu_pop_save_config);

    // 对相机画面右键弹出菜单
    m_p_camera_index = 0;
    connect(ui.camera, &QLabel::customContextMenuRequested, [=](const QPoint &pos) {
        if (videoMax) {     // 当前处于最大化显示情况下，当前的相机index可以直接从maxVideoIndex获取
            qDebug() << "maxVideoIndex: " << maxVideoIndex;
        } else {
            //参数pos用来传递右键点击时的鼠标的坐标，这个坐标一般是相对于控件左上角而言的
            int center_x = ui.camera->geometry().width() / 2;
            int center_y = ui.camera->geometry().height() / 2;
            int x = pos.x(), y = pos.y();
            if (x < center_x && y < center_y) {
                qDebug()<< pos << ": camera_0";
                m_p_camera_index = 0;
            } else if (x > center_x && y < center_y) {
                qDebug()<< pos << ": camera_1";
                m_p_camera_index = 1;
            } else if (x < center_x && y > center_y) {
                qDebug()<< pos << ": camera_2";
                m_p_camera_index = 2;
            } else {
                qDebug()<< pos << ": camera_3";
                m_p_camera_index = 3;
            }
        }
        this->m_pOptMenu->exec(QCursor::pos());
    });

    // 设置可变相机画面大小
    firstImage = true;
    labelWidth = ui.camera_0->width();
    labelHeight = ui.camera_0->height();
    std::cout << "init labelWidth: "<< labelWidth << " labelHeight: " << labelHeight << std::endl;
    std::cout << ui.camera->width() << " " << ui.camera->height() << std::endl;
    videoMax=false;         // 是否有相机最大化播放的标志
    maxVideoIndex = -1;     // -1 表示所有相机正产尺寸显示
    layoutMargin = 9;       // 默认 Widget 的 Margin 为 9
    ui.camera_0->installEventFilter(this);
    ui.camera_1->installEventFilter(this);
    ui.camera_2->installEventFilter(this);
    ui.camera_3->installEventFilter(this);

    // 设置左侧面板
    this->needDetectPerson = false;
    this->needDetectCar = false;
    // QCheckBox *m_checkbox1 = new QCheckBox("check_box1", this);
    // m_checkbox1->setCheckState(Qt::PartiallyChecked)
    QObject::connect(ui.cb_person, &QCheckBox::stateChanged, this, &MainWindow::slot_checkbox_change);
    QObject::connect(ui.cb_car, &QCheckBox::stateChanged, this, &MainWindow::slot_checkbox_change);
    QObject::connect(ui.btn_reverse, &QPushButton::clicked, this, &MainWindow::slot_reverse_event);
    QObject::connect(ui.btn_block, &QPushButton::clicked, this, &MainWindow::slot_block_event);
    QObject::connect(ui.btn_changeLine, &QPushButton::clicked, this, &MainWindow::slot_changeLine_event);
    QObject::connect(ui.btn_park, &QPushButton::clicked, this, &MainWindow::slot_park_event);
    QObject::connect(ui.btn_intrude, &QPushButton::clicked, this, &MainWindow::slot_intrude_event);


    // 设置下方事件区域
    ui.consoleTable->setSelectionMode(QAbstractItemView::NoSelection);  // 禁止点击输出窗口的 item
    setEventTable();
    // 事件添加样例
    // addTrafficEvent("Person", "hight", "right", "detail");
    
    // 设置默认主题
    QString qss = darcula_qss;
    qApp->setStyleSheet(qss);
}


void MainWindow::setEventTable() {
    qDebug() << "setEventTable---";
    ui.consoleTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui.consoleTable->verticalHeader()->setDefaultSectionSize(22); // 设置行距
    ui.consoleTable->setContextMenuPolicy(Qt::CustomContextMenu);
}

void MainWindow::addTrafficEvent(QString type, QString level, QString result, QString opera) {
  int rows = ui.consoleTable->rowCount();
  ui.consoleTable->setRowCount(++rows);
  QDateTime time = QDateTime::currentDateTime();  // 获取系统现在的时间
  QString time_str = time.toString("MM-dd hh:mm:ss"); // 设置显示格式
  qDebug() << rows;
  ui.consoleTable->setItem(rows - 1, 0, new QTableWidgetItem(QString::number(rows)));
  ui.consoleTable->setItem(rows - 1, 1, new QTableWidgetItem(time_str));
  ui.consoleTable->setItem(rows - 1, 2, new QTableWidgetItem(type));
  ui.consoleTable->setItem(rows - 1, 3, new QTableWidgetItem(level));
  ui.consoleTable->setItem(rows - 1, 4, new QTableWidgetItem(result));
  ui.consoleTable->setItem(rows - 1, 5, new QTableWidgetItem(opera));
  ui.consoleTable->scrollToBottom(); // 滑动自动滚到最底部
}


void MainWindow::addTrafficEvent(TrafficEvent* traffic) {
    int rows = ui.consoleTable->rowCount();
    ui.consoleTable->setRowCount(++rows);
    ui.consoleTable->setItem(rows - 1, 0, new QTableWidgetItem(QString::number(rows)));
    ui.consoleTable->setItem(rows - 1, 1, new QTableWidgetItem(traffic->time));
    ui.consoleTable->setItem(rows - 1, 2, new QTableWidgetItem(traffic->type));
    ui.consoleTable->setItem(rows - 1, 3, new QTableWidgetItem(traffic->level));
    ui.consoleTable->setItem(rows - 1, 4, new QTableWidgetItem(traffic->result));
    ui.consoleTable->setItem(rows - 1, 5, new QTableWidgetItem(QString("查看")));
    ui.consoleTable->scrollToBottom(); // 滑动自动滚到最底部
}


/**
 * @brief QWidgetTable item 事件点击槽函数
 * 
 * @param item 
 */
void MainWindow::consoleClick(QTableWidgetItem* item) {
    qDebug() << "dgv click: " << item->text() << ": " << item->row();
    // 根据 item->row() 获取当前事件的 num
    int index = item->row();        // 获取事件的 index
    trafficD->switchPage(0);        // 设置要显示的页面
    trafficD->showImage(trafficList.at(index)->image);
    trafficD->show();
}

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("找不到ROS服务器");
	msgBox.exec();
    close();
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event) {
    if (event->type() == QEvent::MouseButtonDblClick) {
        QLabel *widget = (QLabel *) obj;
        if (videoMax) {
            // 当前为最大化状态，首先把当前的画面移除
            labelWidth = widget->width() / 2;
            labelHeight = widget->height() / 2;
            ui.camera_0->setVisible(true);
            ui.camera_1->setVisible(true);
            ui.camera_2->setVisible(true);
            ui.camera_3->setVisible(true);
            this->maxVideoIndex = -1;
        } else {
            // 根据widget坐标获取widget的index
            int x = widget->geometry().x();
            int y = widget->geometry().y();
            if (x == layoutMargin && y == layoutMargin) {
                this->maxVideoIndex = 0;    // camera_0
            } else if (x > layoutMargin && y == layoutMargin) {
                this->maxVideoIndex = 1;    // cmaera_1
            } else if (x == layoutMargin && y > layoutMargin) {
                this->maxVideoIndex = 2;    // cmaera_2
            } else {
                this->maxVideoIndex = 3;    // cmaera_3
            }
            // 当前未正产状态，需要将其最大化
            labelWidth = widget->width() * 2;
            labelHeight = widget->height() * 2;
            ui.camera_0->setVisible(false);
            ui.camera_1->setVisible(false);
            ui.camera_2->setVisible(false);
            ui.camera_3->setVisible(false);

            widget->setVisible(true);
        }
        videoMax = !videoMax;
        qDebug() << "videoMax: " << videoMax;
        qDebug() << "maxVideoIndex: " << maxVideoIndex;
    }
    return QObject::eventFilter(obj, event);
}



/**
 * @brief 把像素坐标转化为世界坐标
 * 
 * @pax 像素 x 坐标
 * @param y 像素 y 坐标
 * @return cv::Point3f 世界坐标
 */
cv::Point3f MainWindow::cameraToWorld(cv::Point2f point, int cam_index) {
    cv::Mat invR_x_invM_x_uv1, invR_x_tvec, wcPoint;
    double Z = 0;   // Hypothesis ground:

	cv::Mat screenCoordinates = cv::Mat::ones(3, 1, cv::DataType<double>::type);
	screenCoordinates.at<double>(0, 0) = point.x;
	screenCoordinates.at<double>(1, 0) = point.y;
	screenCoordinates.at<double>(2, 0) = 1; // f=1

	// s and point calculation, described here:
	// https://stackoverflow.com/questions/12299870/computing-x-y-coordinate-3d-from-image-point
	// invR_x_invM_x_uv1 = rotationMatrix.inv() * cameraMatrix.inv() * screenCoordinates;
	// invR_x_tvec = rotationMatrix.inv() * transVector;
	// wcPoint = (Z + invR_x_tvec.at<double>(2, 0)) / invR_x_invM_x_uv1.at<double>(2, 0) * invR_x_invM_x_uv1 - invR_x_tvec;
	// //wcPoint = invR_x_invM_x_uv1 - invR_x_tvec;
	// cv::Point3f worldCoordinates(wcPoint.at<double>(0, 0), wcPoint.at<double>(1, 0), wcPoint.at<double>(2, 0));

	invR_x_invM_x_uv1 = this->vec_rotationMatrix[cam_index].inv() * this->vec_cameraMatrix[cam_index].inv() * screenCoordinates;
    invR_x_tvec = this->vec_rotationMatrix[cam_index].inv() * this->vec_transVector[cam_index];
    wcPoint = (Z + invR_x_tvec.at<double>(2, 0)) / invR_x_invM_x_uv1.at<double>(2, 0) * invR_x_invM_x_uv1 - invR_x_tvec;
    cv::Point3f worldCoordinates(wcPoint.at<double>(0, 0), wcPoint.at<double>(1, 0), wcPoint.at<double>(2, 0));
    return worldCoordinates;
}


/******************** 槽函数 *********************/


void MainWindow::menu_pop_config() {
    qDebug() << "menu_pop_config";
    trafficD->switchPage(3);    // 切换要显示的页面
    trafficD->show();
}

void MainWindow::menu_pop_load_config() {
    qDebug() << "menu_pop_load_config";
    QString filename = QFileDialog::getOpenFileName(this, "Open", "./src/mul_t/config/", "(*.yml)");
    if (filename.isEmpty()) 
        return;

    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat rotationVector, rotationMatrix, transVector;
    cv::Mat map1, map2;
    std::vector<cv::Point2f> image_points;
    std::vector<cv::Point3f> world_points;

    cv::FileStorage cameraPameras(filename.toStdString(), cv::FileStorage::READ);
    cameraPameras["camera_matrix"] >> cameraMatrix;
    cameraPameras["dist_coeffs"] >> distCoeffs;
    this->vec_cameraMatrix[m_p_camera_index] = cameraMatrix;
    this->vec_distCoeffs[m_p_camera_index] = distCoeffs;
    // 计算相机姿态
    cameraPameras["imagepoints"] >> image_points;
    cameraPameras["worldpoints"] >> world_points;
    solvePnP(world_points, image_points, cameraMatrix, distCoeffs, rotationVector, transVector);
    Rodrigues(rotationVector, rotationMatrix);
    this->vec_rotationMatrix[m_p_camera_index] = rotationMatrix;
    this->vec_transVector[m_p_camera_index] = transVector;
    // 计算修复畸变的映射矩阵
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, image_size, CV_16SC2, map1, map2);
    this->vec_map1[m_p_camera_index] = map1;
    this->vec_map2[m_p_camera_index] = map2;

    this->vec_cameraCoord[m_p_camera_index] = cv::Point3f(0.12, -2.9, 0);
    this->vec_hasLoadCameraMatrix[m_p_camera_index] = true;
}

void MainWindow::menu_pop_save_config() {
    qDebug() << "menu_pop_save_config";
}

/**
 * @brief 处理复选框状态改变槽函数
 * 
 */
void MainWindow::slot_checkbox_change() {
    if (ui.cb_person->isChecked()) {    // person
        this->needDetectPerson = true;
    } else {
        this->needDetectPerson = false;
    }

    if(ui.cb_car->isChecked()) {        // car
        this->needDetectCar = true;
    } else {
        this->needDetectCar = false;
    }
    qDebug()<<"needDetectPerson: " << needDetectPerson << " needDetectCar: " << needDetectCar;
}


void MainWindow::showPopInfo() {
    QMessageBox::information(this, "注意", "请先选择相机画面并双击最大化！");
}

/// 左侧面板事件槽函数

void MainWindow::slot_reverse_event() {
    qDebug() << "slot_reverse_event";
    if (!videoMax) {
        this->showPopInfo();
        return;
    }
    cv::Mat img;
    switch (maxVideoIndex) {
        case 0: img = cur_frame0;
            break;
        case 1: img = cur_frame1;
            break;
        case 2: img = cur_frame2;
            break;
        default: img = cur_frame3;
            break;
    }
    this->trafficD->setIndexParam(this->maxVideoIndex, 0);
    this->trafficD->switchPage(2);
    this->trafficD->showImage(img);
    this->trafficD->show();
    this->trafficD->showPopInfo(0);
}

void MainWindow::slot_block_event() {
    qDebug() << "slot_block_event";
    if (!videoMax) {
        this->showPopInfo();
        return;
    }
    cv::Mat img;
    switch (maxVideoIndex) {
        case 0: img = cur_frame0;
            break;
        case 1: img = cur_frame1;
            break;
        case 2: img = cur_frame2;
            break;
        default: img = cur_frame3;
            break;
    }
    this->trafficD->setIndexParam(this->maxVideoIndex, 1);
    this->trafficD->switchPage(1);
    this->trafficD->showImage(img);
    this->trafficD->show();
    this->trafficD->showPopInfo(1);
}

void MainWindow::slot_changeLine_event() {
    qDebug() << "slot_changeLine_event";
    if (!videoMax) {
        this->showPopInfo();
        return;
    }
    cv::Mat img;
    switch (maxVideoIndex) {
        case 0: img = cur_frame0;
            break;
        case 1: img = cur_frame1;
            break;
        case 2: img = cur_frame2;
            break;
        default: img = cur_frame3;
            break;
    }
    this->trafficD->setIndexParam(this->maxVideoIndex, 1);
    this->trafficD->switchPage(2);
    this->trafficD->showImage(img);
    this->trafficD->show();
    this->trafficD->showPopInfo(0);
}

void MainWindow::slot_park_event() {
    qDebug() << "slot_park_event";
    if (!videoMax) {
        this->showPopInfo();
        return;
    }
    cv::Mat img;
    switch (maxVideoIndex) {
        case 0: img = cur_frame0;
            break;
        case 1: img = cur_frame1;
            break;
        case 2: img = cur_frame2;
            break;
        default: img = cur_frame3;
            break;
    }
    this->trafficD->setIndexParam(this->maxVideoIndex, 3);
    this->trafficD->switchPage(1);
    this->trafficD->showImage(img);
    this->trafficD->show();
    this->trafficD->showPopInfo(1);
}

void MainWindow::slot_intrude_event() {
    qDebug() << "slot_intrude_event";
    if (!videoMax) {
        this->showPopInfo();
        return;
    }
    cv::Mat img;
    switch (maxVideoIndex) {
        case 0: img = cur_frame0;
            break;
        case 1: img = cur_frame1;
            break;
        case 2: img = cur_frame2;
            break;
        default: img = cur_frame3;
            break;
    }
    this->trafficD->setIndexParam(this->maxVideoIndex, 4);
    this->trafficD->switchPage(1);
    this->trafficD->showImage(img);
    this->trafficD->show();
    this->trafficD->showPopInfo(1);
}


void MainWindow::showConfigPanel() {
    std::cout << "Show Config Panel" << std::endl;
    configP->show();
}

void MainWindow::connectByConfig(ConfigInfo *config) {
    qDebug() << "--------- Config Info ---------";
    qDebug() << config->ros_address;
    qDebug() << config->port;
    if (!qnode.init(config)) {
        // 连接失败
        showNoMasterMessage();
    } else {
        // 连接成功
        ui.btn_config_ros->setEnabled(false);
    }
}


void MainWindow::setROI(QRect roi, int cam_index, int event_index) {
    vec_roi[cam_index][event_index].x = roi.x();
    vec_roi[cam_index][event_index].y = roi.y();
    vec_roi[cam_index][event_index].width = roi.width();
    vec_roi[cam_index][event_index].height = roi.height();

    // std::cout << "vec_roi[0][4]: " << vec_roi[cam_index][event_index] << std::endl;
}


void MainWindow::setLine(double k, double b, int cam_index, int event_index) {
    vec_line[cam_index][event_index][0] = k;
    vec_line[cam_index][event_index][1] = b;
    std::cout << "vec_line[0][1]: " << vec_line[cam_index][event_index][0]
    <<" "<<vec_line[cam_index][event_index][1]<< std::endl;
}


void MainWindow::setImage(cv::Mat image, int cam_index)
{   
    cv::Mat imageCalib;     // 畸变修复后的图像
    if (this->vec_hasLoadCameraMatrix[cam_index]) {      // 如果已经加载了相机参数，则修复畸变
        // cv::remap(image, imageCalib, map1, map2, INTER_LINEAR);
        cv::remap(image, imageCalib, this->vec_map1[cam_index], this->vec_map2[cam_index], INTER_LINEAR);
    } else {
        imageCalib = image;
    }

    // 物体检测（跟踪）过程
    if (this->needDetectPerson || this->needDetectCar) {
        processOD(imageCalib, interval, cam_index);
    }
    else{
        DetectionInfo *detec_info =  objectD->detecRes.at(cam_index);
        detec_info->track_boxes.clear();
        detec_info->track_classIds.clear();
        detec_info->track_confidences.clear();
        detec_info->track_speeds.clear();
        detec_info->track_distances.clear();
    }

    cv::Mat cur_frame;
    switch (cam_index) {
        case 0: 
            imageCalib.copyTo(this->cur_frame0);
            cur_frame = cur_frame0;
            break;
        case 1: 
            imageCalib.copyTo(this->cur_frame1);
            cur_frame = cur_frame1;
            break;
        case 2: 
            imageCalib.copyTo(this->cur_frame2);
            cur_frame = cur_frame2;
            break;
        default: 
            imageCalib.copyTo(this->cur_frame3);
            cur_frame = cur_frame3;
            break;
    }
    // imageCalib.copyTo(this->cur_frame[cam_index]);
    
    // 检测交通逆行事件
    if(vec_line[cam_index][0][1] != 0) {
        double k = vec_line[cam_index][0][0];
        double b = vec_line[cam_index][0][1];
        DetectionInfo *detec_info =  objectD->detecRes.at(cam_index);
        for (int i=0; i < detec_info->track_classIds.size(); i++) {  
            int c_x = detec_info->track_boxes[i].x + detec_info->track_boxes[i].width / 2;
            int c_y = detec_info->track_boxes[i].y + detec_info->track_boxes[i].height;
            cv::Point2i c_point = cv::Point2i(c_x, c_y);
            int nowPosition = this->leftOrRight(k, b, c_point);
            if((nowPosition == 0 && detec_info->track_speeds[i] < 0)||
                (nowPosition == 1 && detec_info->track_speeds[i] > 0)){ 
                double x = (-700-b)/k;
                cv::Point2i p1(x, 700);
                std::cout<<"p1: "<<p1.x<<" "<<p1.y<<std::endl;
                double y = k * 1300.0 + b;
                cv::Point2i p2(1300, -(int)y);
                std::cout<<"p2: "<<p2.x<<" "<<p2.y<<std::endl;
                cv::line(cur_frame, p1, p2, Scalar(50, 178, 255), 3);
                QDateTime time = QDateTime::currentDateTime();  // 获取系统现在的时间
                QString time_str = time.toString("MM-dd hh:mm:ss"); // 设置显示格式
                cv::Mat tmpImg ;
                cur_frame.copyTo(tmpImg);
                TrafficEvent* traffic = new TrafficEvent(time_str, "交通逆行", "高", "正确", tmpImg);
                trafficList.push_back(traffic);
                this->addTrafficEvent(traffic);             // 添加进事件展示列表
                vec_line[cam_index][0][1] = 0;
                break;
            }    
        }             
    }

    // 检测交通拥堵事件
    if (vec_roi[cam_index][1].width != 0) {
        // std::cout << "rect: " << vec_roi[cam_index][3] << std::endl;
        DetectionInfo *detec_info =  objectD->detecRes.at(cam_index);
        int carInRoi_num = 0;
        for (int i=0; i < detec_info->track_classIds.size(); i++) {
             // 如果物体id大于0 : car. 且速度小于10
            if (detec_info->track_classIds[i] > 0 &&
             detec_info->track_speeds[i] < 10 && detec_info->track_speeds[i] > -10) { 
                // 计算检测框的质心
                int c_x = detec_info->track_boxes[i].x + detec_info->track_boxes[i].width / 2;
                int c_y = detec_info->track_boxes[i].y + detec_info->track_boxes[i].height / 2;
                cv::Point2i c_point = cv::Point2i(c_x, c_y);
                if (vec_roi[cam_index][1].contains(c_point)) {
                    carInRoi_num += 1;
                }
            }
        }
        int width = vec_roi[cam_index][1].width;     // ROI 宽
        int height = vec_roi[cam_index][1].height;   // ROI 高
        int square = width * height;                 // ROI 面积
        int threshold = square/15000;                // 车辆数量阈值
        if (carInRoi_num > threshold){              // 若ROI区域内车辆数量大于等于阈值
            // std::cout << "square: " << square << std::endl;
            // std::cout << "threshold: " << threshold << std::endl;
            int left = vec_roi[cam_index][1].x;
            int top = vec_roi[cam_index][1].y;
            int right = left + width;
            int bottom = top + height;
            cv::rectangle(cur_frame, Point(left, top), Point(right, bottom), Scalar(50, 178, 255), 2);
            QDateTime time = QDateTime::currentDateTime();  // 获取系统现在的时间
            QString time_str = time.toString("MM-dd hh:mm:ss"); // 设置显示格式
            cv::Mat tmpImg ;
            cur_frame.copyTo(tmpImg);
            TrafficEvent* traffic = new TrafficEvent(time_str, "交通拥堵", "高", "正确", tmpImg);
            trafficList.push_back(traffic);
            this->addTrafficEvent(traffic);             // 添加进事件展示列表
            vec_roi[cam_index][1].width = 0;
        }
    }


    // 检测异常变道事件
    if(vec_line[cam_index][1][1] != 0) {
        double k = vec_line[cam_index][1][0];
        double b = vec_line[cam_index][1][1];
        DetectionInfo *detec_info =  objectD->detecRes.at(cam_index);
        if(detec_info->leftOrRight.empty()){
            for (int i=0; i < detec_info->track_classIds.size(); i++) {
                // 计算检测框的底部中心
                int c_x = detec_info->track_boxes[i].x + detec_info->track_boxes[i].width / 2;
                int c_y = detec_info->track_boxes[i].y + detec_info->track_boxes[i].height;
                cv::Point2i c_point = cv::Point2i(c_x, c_y);
                detec_info->leftOrRight.push_back(this->leftOrRight(k, b, c_point));
            }
        }
        else{
            for (int i=0; i < detec_info->track_classIds.size(); i++) {          
                // for (auto j : detec_info->leftOrRight)
                //     std::cout << j << ' ';
                // std::cout<<std::endl;
                // 如果物体id为1 : car
                if (detec_info->track_classIds[i] >= 0) { 
                    int c_x = detec_info->track_boxes[i].x + detec_info->track_boxes[i].width / 2;
                    int c_y = detec_info->track_boxes[i].y + detec_info->track_boxes[i].height;
                    cv::Point2i c_point = cv::Point2i(c_x, c_y);
                    int nowPosition = this->leftOrRight(k, b, c_point);
                    // std::cout<<"old: "<<detec_info->leftOrRight[i]<<"; new: "<<nowPosition<<std::endl;
                    if(detec_info->leftOrRight[i]^nowPosition == 1){ // 前后不同
                        double x = (-700-b)/k;
                        cv::Point2i p1(x, 700);
                        std::cout<<"p1: "<<p1.x<<" "<<p1.y<<std::endl;
                        double y = k * 1300.0 + b;
                        cv::Point2i p2(1300, -(int)y);
                        std::cout<<"p2: "<<p2.x<<" "<<p2.y<<std::endl;
                        cv::line(cur_frame, p1, p2, Scalar(50, 178, 255), 3);
                        QDateTime time = QDateTime::currentDateTime();  // 获取系统现在的时间
                        QString time_str = time.toString("MM-dd hh:mm:ss"); // 设置显示格式
                        cv::Mat tmpImg ;
                        cur_frame.copyTo(tmpImg);
                        TrafficEvent* traffic = new TrafficEvent(time_str, "异常变道", "高", "正确", tmpImg);
                        trafficList.push_back(traffic);
                        this->addTrafficEvent(traffic);             // 添加进事件展示列表
                        vec_line[cam_index][1][1] = 0;
                        detec_info->leftOrRight.clear();
                        break;
                    }
                }
            }
        }        
    }


    // 检测异常停车事件
    if (vec_roi[cam_index][3].width != 0) {
        // std::cout << "rect: " << vec_roi[cam_index][3] << std::endl;
        DetectionInfo *detec_info =  objectD->detecRes.at(cam_index);
        for (int i=0; i < detec_info->track_classIds.size(); i++) {
             // 如果物体id为1 : car 且速度为0
            if (detec_info->track_classIds[i] > 0 && detec_info->track_speeds[i] == 0) { 
                // 计算检测框的质心
                int c_x = detec_info->track_boxes[i].x + detec_info->track_boxes[i].width / 2;
                int c_y = detec_info->track_boxes[i].y + detec_info->track_boxes[i].height / 2;
                cv::Point2i c_point = cv::Point2i(c_x, c_y);
                if (vec_roi[cam_index][3].contains(c_point)) {
                    int left = vec_roi[cam_index][3].x;
                    int top = vec_roi[cam_index][3].y;
                    int right = vec_roi[cam_index][3].x + vec_roi[cam_index][3].width;
                    int bottom = vec_roi[cam_index][3].y + vec_roi[cam_index][3].height;
                    cv::rectangle(cur_frame, Point(left, top), Point(right, bottom), Scalar(50, 178, 255), 2);
                    QDateTime time = QDateTime::currentDateTime();  // 获取系统现在的时间
                    QString time_str = time.toString("MM-dd hh:mm:ss"); // 设置显示格式
                    cv::Mat tmpImg ;
                    cur_frame.copyTo(tmpImg);
                    TrafficEvent* traffic = new TrafficEvent(time_str, "异常停车", "高", "正确", tmpImg);
                    trafficList.push_back(traffic);
                    this->addTrafficEvent(traffic);             // 添加进事件展示列表
                    vec_roi[cam_index][3].width = 0;
                    break;
                }
            }
        }
    }

    // 检测弱势交通参与者闯入事件
    if (vec_roi[cam_index][4].width != 0) {
        DetectionInfo *detec_info =  objectD->detecRes.at(cam_index);
        for (int i=0; i < detec_info->track_classIds.size(); i++) {
            if (detec_info->track_classIds[i] == 0) {     // 如果物体id为 0: person
                // 计算检测框的质心
                int c_x = detec_info->track_boxes[i].x + detec_info->track_boxes[i].width / 2;
                int c_y = detec_info->track_boxes[i].y + detec_info->track_boxes[i].height / 2;
                cv::Point2i c_point = cv::Point2i(c_x, c_y);
                if (vec_roi[cam_index][4].contains(c_point)) {
                    int left = vec_roi[cam_index][4].x;
                    int top = vec_roi[cam_index][4].y;
                    int right = vec_roi[cam_index][4].x + vec_roi[cam_index][4].width;
                    int bottom = vec_roi[cam_index][4].y + vec_roi[cam_index][4].height;
                    cv::rectangle(cur_frame, Point(left, top), Point(right, bottom), Scalar(50, 178, 255), 2);
                    QDateTime time = QDateTime::currentDateTime();  // 获取系统现在的时间
                    QString time_str = time.toString("MM-dd hh:mm:ss"); // 设置显示格式
                    cv::Mat tmpImg ;
                    cur_frame.copyTo(tmpImg);
                    TrafficEvent* traffic = new TrafficEvent(time_str, "弱势闯入", "高", "正确", tmpImg);
                    trafficList.push_back(traffic);
                    this->addTrafficEvent(traffic);             // 添加进事件展示列表
                    vec_roi[cam_index][4].width = 0;
                    break;
                }
            }
        }
    }

    // 保存接收到的第一帧图片（测试用）
    // if (needSave) {
    //     cv::imwrite("./test.png", imageCalib);
    //     needSave = !needSave;
    // }
    
    // qimage_mutex_.lock();
    cv::Mat showImg = imageCalib;
    // cvtColor(imageCalib, showImg, CV_BGR2RGB);
    QImage img = QImage((const unsigned char*)(showImg.data), showImg.cols, 
                                        showImg.rows, showImg.step, QImage::Format_RGB888);
    QImage scaleImage;
    QLabel *label;
    switch (cam_index) {
        case 0: label = ui.camera_0;
            break;
        case 1: label = ui.camera_1;
            break;
        case 2: label = ui.camera_2;
            break;
        default: label = ui.camera_3;
            break;
    }
    // scaleImage = img.scaled(label->width(), label->height());
    if (!firstImage) {
        label->setFixedSize(labelWidth, labelHeight);
    }
    scaleImage = img.scaled(labelWidth, labelHeight);
    // label->setScaledContents(true);
    label->setPixmap(QPixmap::fromImage(scaleImage));
    // qimage_mutex_.unlock();
    if (firstImage) {
        // 第一次渲染时候获取到画面宽度
        labelHeight = label->height();
        labelWidth = label->width();
        firstImage = false;
    }
}


/**
 * @brief 处理检测（跟踪）过程
 * 
 * @param image 需要检测的图像
 * @param interval 检测间隔
 * @param cam_index 对应摄像头的index
 */
void MainWindow::processOD(cv::Mat &image, int interval, int cam_index) {
    // 当前相机的检测结果
    DetectionInfo *detec_info =  objectD->detecRes.at(cam_index);

    if (detec_info->index % interval == 0) {
        detec_info->track_boxes_pre.clear();
        // 将当前帧检测到的框赋值给 track_boxes_pre
        detec_info->track_boxes_pre.assign(
            detec_info->track_boxes.begin(), detec_info->track_boxes.end());
        
        detec_info->track_boxes.clear();
        detec_info->track_classIds.clear();
        detec_info->track_confidences.clear();
        objectD->runODModel(image, cam_index, needDetectPerson, needDetectCar);
    } else {
        // 跟踪算法
        // objectD->runTrackerModel(image);
    }
    detec_info->index = (detec_info->index + 1) % 25;
    
    // 计算各个目标的速度与距离
    // vector<float> speeds, distances;
    detec_info->track_speeds.clear();
    detec_info->track_distances.clear();
    if (detec_info->track_boxes_pre.empty() || detec_info->track_boxes.empty() ||
        detec_info->track_boxes_pre.size() != detec_info->track_boxes.size() ||
        !vec_hasLoadCameraMatrix[cam_index]) {
        // 如果track_boxes_pre没有数据，则表示第一次检测，所有物体速度为0
        detec_info->track_speeds = vector<float>(detec_info->track_boxes.size(), 0.0);
        detec_info->track_distances = vector<float>(detec_info->track_boxes.size(), 0.0);
    } else {
        // 只有前后两帧 bboxes 长度一样时
        for (int i=0; i<detec_info->track_boxes.size(); i++) {
            // 这里 type=1 表示返回底部中心坐标
            cv::Point2f pre = getPixelPoint(detec_info->track_boxes_pre[i], 1);
            cv::Point2f cur = getPixelPoint(detec_info->track_boxes[i], 1);

            // 计算真实世界坐标
            cv::Point3f wd_pre = cameraToWorld(pre, cam_index);
            cv::Point3f wd_cur = cameraToWorld(cur, cam_index);
            float delta = sqrt(pow(wd_cur.x-wd_pre.x, 2) + pow(wd_cur.y-wd_pre.y, 2));
            float speed = delta / float(interval / 25.0);
            // 这里计算的速度为 像素/秒
            // float dist = sqrt(pow(cur.x-pre.x, 2) + pow(cur.y-pre.y, 2));
            // float speed = dist / float(interval / 25.0);      // 25表示每秒25帧
            
            // 计算物体距离（不考虑高度）
            float dist_pre = sqrt(pow(wd_pre.x-this->vec_cameraCoord[cam_index].x, 2) + 
                                    pow(wd_pre.y-vec_cameraCoord[cam_index].y, 2));
            float dist = sqrt(pow(wd_cur.x-this->vec_cameraCoord[cam_index].x, 2) + 
                                    pow(wd_cur.y-vec_cameraCoord[cam_index].y, 2));
            // 速度为负表示背离相机运动
            if(dist_pre < dist){
                speed *= -1;
            }
            detec_info->track_speeds.push_back(speed);
            detec_info->track_distances.push_back(dist);
        }
    }

    // 如果当前帧没有检测，则直接用上次的检测结果
    for (unsigned int i=0; i< detec_info->track_boxes.size(); i++) {
        int x = detec_info->track_boxes[i].x;
        int y = detec_info->track_boxes[i].y;
        int width = detec_info->track_boxes[i].width;
        int height = detec_info->track_boxes[i].height;
        objectD->drawPred(detec_info->track_classIds[i], detec_info->track_confidences[i], 
                      detec_info->track_speeds[i], detec_info->track_distances[i],  x, y, x+width, y+height, 
                           image, needDetectPerson, needDetectCar);
    }

    // if (detec_info->track_confidences.size() > 1) {
    //     for (int i=0; i<detec_info->track_confidences.size(); i++) {
    //         std::cout << detec_info->track_confidences[i] << " ";
    //     }
    //     std::cout << std::endl;
    // }
}

void MainWindow::exit() { 
    close();
}

/**
 * @brief 返回物体像素坐标
 * 
 * @param rect 
 * @param type 返回质心:0或底部中心:1
 * @return cv::Point2f 
 */
cv::Point2f MainWindow::getPixelPoint(Rect &rect, int type) {
    int x = rect.x;
    int y = rect.y;
    int width = rect.width;
    int height = rect.height;
    if (type == 0) {
        return cv::Point2f(x+width/2, y+height/2);   // 质心坐标
    }
    return cv::Point2f(x+width/2, y+height);        // 底部中心坐标
}

int MainWindow::leftOrRight(double k, double b, cv::Point2i p){
    double x = (double)p.x;
    double y = -(double)p.y;
    double l = k * x + b;
    if(k > 0) {
         if(y > l) return 0;
         else return 1;
    }
    else{
        if(y > l) return 0;
        else return 1;
    }
}


}  // namespace mul_t

