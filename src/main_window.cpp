
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
    objectD = new ObjectDetection(2);    // 初始化 检测对象对象

    hasLoadCameraMatrix = false;        // 当前未加载相加参数
    image_size = cv::Size(1280, 720);
    cam_num = 2;        // 临时设置相机数量
    interval = 5;       // 物体检测间隔
    
    // QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(ui.btn_config_ros, &QPushButton::clicked, this, &MainWindow::showConfigPanel);
    QObject::connect(ui.btn_quit, &QPushButton::clicked, this, &MainWindow::exit);
    // QTableWidget 单元格点击事件 SLOT
    QObject::connect(ui.consoleTable ,SIGNAL(itemClicked(QTableWidgetItem*)), this, SLOT(consoleClick(QTableWidgetItem*)));
    QObject::connect(ui.btn_loadMatrix, &QPushButton::clicked, this, &MainWindow::loadCameraMatrix2);
    QObject::connect(&qnode, SIGNAL(getImage(cv::Mat, int)), this, SLOT(setImage(cv::Mat, int)));
    
    //接收登录页面传来的数据
    connect(configP, SIGNAL(getConfigInfo(ConfigInfo*)), this, SLOT(connectByConfig(ConfigInfo*)));

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

    connect(ui.camera, &QLabel::customContextMenuRequested, [=](const QPoint &pos) {
        //参数pos用来传递右键点击时的鼠标的坐标，这个坐标一般是相对于控件左上角而言的
        int center_x = ui.camera->geometry().width() / 2;
        int center_y = ui.camera->geometry().height() / 2;
        int x = pos.x(), y = pos.y();
        
        if (x < center_x && y < center_y) {
            qDebug()<< pos << ": camera_0";
        } else if (x > center_x && y < center_y) {
            qDebug()<< pos << ": camera_1";
        } else if (x < center_x && y > center_y) {
            qDebug()<< pos << ": camera_2";
        } else {
            qDebug()<< pos << ": camera_3";
        }
        this->m_pOptMenu->exec(QCursor::pos());
    });

    // 设置可变相机画面大小
    firstImage = true;
    labelWidth = ui.camera_0->width();
    labelHeight = ui.camera_0->height();
    std::cout << "init labelWidth: "<< labelWidth << " labelHeight: " << labelHeight << std::endl;
    std::cout << ui.camera->width() << " " << ui.camera->height() << std::endl;
    videoMax=false;
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


/**
 * @brief 手动加载相机参数，计算相机姿态
 * 
 */
void MainWindow::loadCameraMatrix2() {
    QString filename = QFileDialog::getOpenFileName(this, "Open", "./", "(*.yml)");
    if (filename.isEmpty()) 
        return;

    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat rotationVector, rotationMatrix, transVector;
    cv::Mat map1, map2;
    std::vector<cv::Point2f> image_points;
    std::vector<cv::Point3f> world_points;
    for (int i=0; i<cam_num; i++) {
        cv::FileStorage cameraPameras(filename.toStdString(), cv::FileStorage::READ);
        cameraPameras[format("camera_matrix_%d", i)] >> cameraMatrix;
        cameraPameras[format("dist_coeffs_%d", i)] >> distCoeffs;
        // std::cout << "cameraMatrix: " << cameraMatrix << std::endl;
        // std::cout << "distCoeffs: " << distCoeffs << std::endl;
        this->vec_cameraMatrix.push_back(cameraMatrix);
        this->vec_distCoeffs.push_back(distCoeffs);
        // 计算相机姿态
        cameraPameras["imagepoints"] >> image_points;
        cameraPameras["worldpoints"] >> world_points;
        // std::cout << image_points << std::endl;
        // std::cout << world_points << std::endl;
        solvePnP(world_points, image_points, cameraMatrix, distCoeffs, rotationVector, transVector);
	    Rodrigues(rotationVector, rotationMatrix);
        this->vec_rotationMatrix.push_back(rotationMatrix);
        this->vec_transVector.push_back(transVector);
        // 计算修复畸变的映射矩阵
        cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, image_size, CV_16SC2, map1, map2);
        this->vec_map1.push_back(map1);
        this->vec_map2.push_back(map2);

        this->vec_cameraCoord.push_back(cv::Point3f(0.12, -2.9, 0));  
    }
    this->hasLoadCameraMatrix = true;
}



// void MainWindow::loadCameraMatrix() {
//     cv::Mat rotationVector, rotationMatrix, transVector;
//     std::vector<cv::Point2f> image_points;
//     std::vector<cv::Point3f> world_points;
//     // Load intrinsics and points.
// 	cv::FileStorage intrin("/home/js/leishen_ws/src/mul_t/config/intrinsics.yml", cv::FileStorage::READ);
// 	cv::FileStorage points("/home/js/leishen_ws/src/mul_t/config/pointssets.yml", cv::FileStorage::READ);
// 	cv::FileStorage extrin("/home/js/leishen_ws/src/mul_t/config/extrinsics.yml", cv::FileStorage::WRITE);
// 	intrin["camera_matrix"] >> cameraMatrix;
// 	intrin["dist_coeffs"] >> distCoeffs;
// 	points["imagepoints"] >> image_points;
// 	points["worldpoints"] >> world_points;
//     std::cout << image_points << std::endl;
//     std::cout << world_points << std::endl;
// 	// Generate our matrix; rvec and tvec are the output.
// 	solvePnP(world_points, image_points, cameraMatrix, distCoeffs, rotationVector, transVector);
// 	Rodrigues(rotationVector, rotationMatrix);

//     this->rotationMatrix = rotationMatrix;
//     this->transVector = transVector;
// 	// Save the results
// 	// extrin << "rotationmatrix" << rotationMatrix;
// 	// extrin << "translationvector" << transVector;

//     // 初始化相机坐标，不考虑高度
//     this->cameraCoord = cv::Point3f(0.12, -2.9, 0);

//     this->hasLoadCameraMatrix = true;
// }


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


 
void MainWindow::consoleClick(QTableWidgetItem* item) {
    qDebug() << "dgv click: " << item->text() << ": " << item->row();
    // 根据 item->row() 获取当前事件的 num
    int index = item->row();
    trafficD->showTrafficImage(trafficList.at(index)->image);
    trafficD->show();
    // std::cout << trafficList.at(index)->time.toStdString() << " " << trafficList.at(index)->level.toStdString() << std::endl;
    // cv::Mat frame = trafficList.at(index)->image;
    // std::cout << frame.rows << " " << frame.cols << std::endl;
    // cv::imshow("frame", trafficList.at(index)->image);
    // waitKey(0);
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
        } else {
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
}

void MainWindow::menu_pop_load_config() {
    qDebug() << "menu_pop_load_config";
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


/// 左侧面板事件槽函数

void MainWindow::slot_reverse_event() {
    qDebug() << "slot_reverse_event";
}

void MainWindow::slot_block_event() {
    qDebug() << "slot_block_event";
}

void MainWindow::slot_changeLine_event() {
    qDebug() << "slot_changeLine_event";
}

void MainWindow::slot_park_event() {
    qDebug() << "slot_park_event";
}

void MainWindow::slot_intrude_event() {
    qDebug() << "slot_intrude_event";
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


void MainWindow::setImage(cv::Mat image, int cam_index)
{   
    cv::Mat imageCalib;     // 畸变修复后的图像
    if (hasLoadCameraMatrix) {      // 如果已经加载了相机参数，则修复畸变
        // cv::remap(image, imageCalib, map1, map2, INTER_LINEAR);
        cv::remap(image, imageCalib, this->vec_map1[cam_index], this->vec_map2[cam_index], INTER_LINEAR);
    } else {
        imageCalib = image;
    }

    // 物体检测（跟踪）过程
    if (this->needDetectPerson || this->needDetectCar) {
        processOD(imageCalib, interval, cam_index);
    }
    

    // 保存接收到的第一帧图片（测试用）
    if (needSave) {
        cv::imwrite("./test.png", imageCalib);
        needSave = !needSave;
        // 添加事件列表的样例
        QDateTime time = QDateTime::currentDateTime();  // 获取系统现在的时间
        QString time_str = time.toString("MM-dd hh:mm:ss"); // 设置显示格式
        TrafficEvent* traffic = new TrafficEvent(time_str, "Person", "高", "正确", imageCalib);
        trafficList.push_back(traffic);
        this->addTrafficEvent(traffic);             // 添加进事件展示列表
    }
    
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
        objectD->runODModel(image, cam_index);
    } else {
        // 跟踪算法
        // objectD->runTrackerModel(image);
    }
    detec_info->index = (detec_info->index + 1) % 30;
    
    // 计算各个目标的速度与距离
    vector<float> speeds, distances;
    if (detec_info->track_boxes_pre.empty() || detec_info->track_boxes.empty() ||
        detec_info->track_boxes_pre.size() != detec_info->track_boxes.size() ||
        !hasLoadCameraMatrix) {
        // 如果track_boxes_pre没有数据，则表示第一次检测，所有物体速度为0
        speeds = vector<float>(detec_info->track_boxes.size(), 0.0);
        distances = vector<float>(detec_info->track_boxes.size(), 0.0);
    } else {
        // 只有前后两帧 bboxes 长度一样时
        for (int i=0; i<detec_info->track_boxes.size(); i++) {
            cv::Point2f pre = getPixelPoint(detec_info->track_boxes_pre[i]);
            cv::Point2f cur = getPixelPoint(detec_info->track_boxes[i]);

            // 计算真实世界坐标
            cv::Point3f wd_pre = cameraToWorld(pre, cam_index);
            cv::Point3f wd_cur = cameraToWorld(cur, cam_index);
            float delta = sqrt(pow(wd_cur.x-wd_pre.x, 2) + pow(wd_cur.y-wd_pre.y, 2));
            float speed = delta / float(interval / 25.0);
            // 这里计算的速度为 像素/秒
            // float dist = sqrt(pow(cur.x-pre.x, 2) + pow(cur.y-pre.y, 2));
            // float speed = dist / float(interval / 25.0);      // 25表示每秒25帧
            speeds.push_back(speed);
            // 计算物体距离（不考虑高度）
            float dist = sqrt(pow(wd_cur.x-this->vec_cameraCoord[cam_index].x, 2) + 
                                    pow(wd_cur.y-vec_cameraCoord[cam_index].y, 2));
            distances.push_back(dist);
        }
    }

    // 如果当前帧没有检测，则直接用上次的检测结果
    for (unsigned int i=0; i< detec_info->track_boxes.size(); i++) {
        int x = detec_info->track_boxes[i].x;
        int y = detec_info->track_boxes[i].y;
        int width = detec_info->track_boxes[i].width;
        int height = detec_info->track_boxes[i].height;
        objectD->drawPred(detec_info->track_classIds[i], detec_info->track_confidences[i], 
                      speeds[i], distances[i],  x, y, x+width, y+height, image, needDetectPerson, needDetectCar);
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
 * @return cv::Point2f 
 */
cv::Point2f MainWindow::getPixelPoint(Rect &rect) {
    int x = rect.x;
    int y = rect.y;
    int width = rect.width;
    int height = rect.height;
    // return cv::Point2f(x+width/2, y+height/2);   // 质心坐标
    return cv::Point2f(x+width/2, y+height);        // 底部中心坐标
}


}  // namespace mul_t

