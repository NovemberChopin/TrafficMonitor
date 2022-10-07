
#include <QtGui>
#include <QMessageBox>
#include <QPushButton>
#include <QDebug>
#include <QString>
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
    objectD = new ObjectDetection(2);    // 初始化 检测对象对象

    interval = 5;

    // QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(ui.btn_config, &QPushButton::clicked, this, &MainWindow::showConfigPanel);
    QObject::connect(ui.btn_quit, &QPushButton::clicked, this, &MainWindow::exit);
    QObject::connect(ui.btn_test, &QPushButton::clicked, this, &MainWindow::testButton);
    QObject::connect(&qnode, SIGNAL(getImage1(cv::Mat)), this, SLOT(setImage1(cv::Mat)));
    QObject::connect(&qnode, SIGNAL(getImage2(cv::Mat)), this, SLOT(setImage2(cv::Mat)));

    //接收登录页面传来的数据
    connect(configP, SIGNAL(ros_input_over(QString, QString, QString)), 
            this, SLOT(connectByConfig(QString, QString, QString)));

    // 初始化
    initial();
}

void MainWindow::initial() {
    setWindowIcon(QIcon(":/images/icon.png"));
    setWindowTitle(tr("视频场景监控"));

    // 计算无畸变和修正转换映射
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0,0) = 795.28917;
	cameraMatrix.at<double>(0,2) = 624.15859;
	cameraMatrix.at<double>(1,1) = 799.53981;
	cameraMatrix.at<double>(1,2) = 356.17181;

    distCoeffs = Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0,0) = -0.387154;
	distCoeffs.at<double>(1,0) = 0.116917;
	distCoeffs.at<double>(2,0) = -0.004499;
	distCoeffs.at<double>(3,0) = 0.003409;
	distCoeffs.at<double>(4,0) = 0.000000;

    image_size = cv::Size(1280, 720);
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, image_size, CV_16SC2, map1, map2);


    // 设置页面以全屏显示
    QDesktopWidget desktop;
    int screenX = desktop.availableGeometry().width();
    int screenY = desktop.availableGeometry().height();
    this->resize(screenX, screenY);

    // 设置可变相机画面大小
    tempWidth = labelWidth = ui.camera_0->width();
    tempHeight = labelHeight = ui.camera_0->height();
    std::cout << "init labelWidth: "<< labelWidth << " labelHeight: " << labelHeight << std::endl;
    std::cout << ui.camera->width() << " " << ui.camera->height() << std::endl;
    video0Max=false;
    video1Max=false;
    video2Max=false;
    video3Max=false;
    ui.camera_0->installEventFilter(this);
    ui.camera_1->installEventFilter(this);
    ui.camera_2->installEventFilter(this);
    ui.camera_3->installEventFilter(this);

    // 设置下方事件区域
    ui.consoleTable->setSelectionMode(QAbstractItemView::NoSelection);  // 禁止点击输出窗口的 item
    setEventTable();
    consoleLog("hight", "right", "detial");

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

void MainWindow::consoleLog(QString level, QString result, QString opera) {
  int rows = ui.consoleTable->rowCount();
  ui.consoleTable->setRowCount(++rows);
  QDateTime time = QDateTime::currentDateTime();  // 获取系统现在的时间
  QString time_str = time.toString("MM-dd hh:mm:ss"); // 设置显示格式
  qDebug() << rows;
  ui.consoleTable->setItem(rows - 1, 0, new QTableWidgetItem(QString::number(rows)));
  ui.consoleTable->setItem(rows - 1, 1, new QTableWidgetItem(time_str));
  ui.consoleTable->setItem(rows - 1, 2, new QTableWidgetItem(level));
  ui.consoleTable->setItem(rows - 1, 3, new QTableWidgetItem(result));
  ui.consoleTable->setItem(rows - 1, 4, new QTableWidgetItem(opera));
  ui.consoleTable->scrollToBottom(); // 滑动自动滚到最底部
}



void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("找不到ROS服务器");
	msgBox.exec();
    close();
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event) {
    // std::cout << "init temp: "<< tempWidth << " label: " << labelWidth << std::endl;
    if (event->type() == QEvent::MouseButtonDblClick) {
        if (obj == ui.camera_0) {
            if (video0Max) {
                // 当前 camera_0 为最大化状态，要将其变为正常
                ui.camera_1->setVisible(true);
                ui.camera_2->setVisible(true);
                ui.camera_3->setVisible(true);

                labelWidth = ui.camera_0->width()/2;
                labelHeight = ui.camera_0->height()/2;
                std::cout << "恢复正常: "<< labelWidth << " label: " << labelHeight << std::endl;
            } else {
                // 当前 camera_0 为正常显示，要将其最大化
                ui.camera_1->setVisible(false);
                ui.camera_2->setVisible(false);
                ui.camera_3->setVisible(false);
                
                labelWidth = ui.camera->width();
                labelHeight = ui.camera->height();
                std::cout << "最大化: "<< labelWidth << " label: " << labelHeight << std::endl;
            }
            video0Max = !video0Max;
        } else {

        }
    }
    return QObject::eventFilter(obj, event);
}

/******************** 槽函数 *********************/

void MainWindow::testButton() {
    std::cout << " Test Button" << std::endl;
    std::cout << ui.camera_0->width() << " " << ui.camera_0->height();
}

void MainWindow::showConfigPanel() {
    std::cout << "Show Config Panel" << std::endl;
    configP->show();
}

void MainWindow::connectByConfig(QString ros_address, QString ros_port, QString ros_topic) {
    qDebug() << ros_address;
    qDebug() << ros_port;
    qDebug() << ros_topic;

    if (!qnode.init()) {
        // 连接失败
        showNoMasterMessage();
    } else {
        // 连接成功
        ui.btn_config->setEnabled(false);
    }
}

void MainWindow::setImage2(cv::Mat image) {
    // std::cout << "image size: " << image.size() << std::endl;
    qimage_mutex_.lock();
    cv::Mat imageCalib;     // 畸变修复后的图像
    cv::remap(image, imageCalib, map1, map2, INTER_LINEAR);
    

    int cam_index = 1;
    processOD(imageCalib, interval, cam_index);
    cv::Mat showImg;
    cvtColor(imageCalib, showImg, CV_BGR2RGB);
    QImage img = QImage((const unsigned char*)(showImg.data), showImg.cols, 
                                        showImg.rows, showImg.step, QImage::Format_RGB888);
    QImage scaleImage = img.scaled(ui.camera_1->width(), ui.camera_1->height());
    ui.camera_1->setScaledContents(true);
    ui.camera_1->setPixmap(QPixmap::fromImage(scaleImage));
    qimage_mutex_.unlock();
}

void MainWindow::setImage1(cv::Mat image)
{   
    qimage_mutex_.lock();
    cv::Mat imageCalib;     // 畸变修复后的图像
    cv::remap(image, imageCalib, map1, map2, INTER_LINEAR);

    int cam_index = 0;
    processOD(imageCalib, interval, cam_index);
    
    cv::Mat showImg;
    cvtColor(imageCalib, showImg, CV_BGR2RGB);
    QImage img = QImage((const unsigned char*)(showImg.data), showImg.cols, 
                                        showImg.rows, showImg.step, QImage::Format_RGB888);
    QImage scaleImage = img.scaled(ui.camera_0->width(), ui.camera_0->height());
    // QImage scaleImage = img.scaled(labelWidth, labelHeight);
    ui.camera_0->setScaledContents(true);
    ui.camera_0->setPixmap(QPixmap::fromImage(scaleImage));
    qimage_mutex_.unlock();
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
    
    // 计算各个目标的速度
    vector<float> speeds;
    if (detec_info->track_boxes_pre.empty() || detec_info->track_boxes.empty() ||
        detec_info->track_boxes_pre.size() != detec_info->track_boxes.size()) {
        // 如果track_boxes_pre没有数据，则表示第一次检测，所有物体速度为0
        speeds = vector<float>(detec_info->track_boxes.size(), 0.0);
    } else {
        // 只有前后两帧 bboxes 长度一样时
        for (int i=0; i<detec_info->track_boxes.size(); i++) {
            cv::Point pre = getCenterPoint(detec_info->track_boxes_pre[i]);
            cv::Point cur = getCenterPoint(detec_info->track_boxes[i]);
            // 这里计算的速度为 像素/秒
            float dist = sqrt(pow(cur.x-pre.x, 2) + pow(cur.y-pre.y, 2));
            float speed = dist / float(interval / 25.0);      // 25表示每秒25帧
            speeds.push_back(speed);
        }
    }

    // 如果当前帧没有检测，则直接用上次的检测结果
    for (unsigned int i=0; i< detec_info->track_boxes.size(); i++) {
        int x = detec_info->track_boxes[i].x;
        int y = detec_info->track_boxes[i].y;
        int width = detec_info->track_boxes[i].width;
        int height = detec_info->track_boxes[i].height;
        objectD->drawPred(detec_info->track_classIds[i], detec_info->track_confidences[i], 
                      speeds[i],  x, y, x+width, y+height, image);
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

// 返回物体检测框的中心点
cv::Point MainWindow::getCenterPoint(Rect &rect) {
    int x = rect.x;
    int y = rect.y;
    int width = rect.width;
    int height = rect.height;
    return Point(x+width/2, y+height/2);
}

// void MainWindow::ReadSettings() {
//     QSettings settings("Qt-Ros Package", "mul_t");
//     restoreGeometry(settings.value("geometry").toByteArray());
//     restoreState(settings.value("windowState").toByteArray());
//     QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
//     QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
//     //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
//     ui.line_edit_master->setText(master_url);
//     ui.line_edit_host->setText(host_url);
//     bool remember = settings.value("remember_settings", false).toBool();
//     ui.checkbox_remember_settings->setChecked(remember);
//     bool checked = settings.value("use_environment_variables", false).toBool();
//     ui.checkbox_use_environment->setChecked(checked);
//     if ( checked ) {
//     	ui.line_edit_master->setEnabled(false);
//     	ui.line_edit_host->setEnabled(false);
//     }
// }

// void MainWindow::WriteSettings() {
//     QSettings settings("Qt-Ros Package", "mul_t");
//     settings.setValue("master_url",ui.line_edit_master->text());
//     settings.setValue("host_url",ui.line_edit_host->text());
//     settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
//     settings.setValue("geometry", saveGeometry());
//     settings.setValue("windowState", saveState());
//     settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
// }

// void MainWindow::closeEvent(QCloseEvent *event)
// {
// 	// WriteSettings();
// 	QMainWindow::closeEvent(event);
// }

}  // namespace mul_t

