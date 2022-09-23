
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

    // QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(ui.btn_config, &QPushButton::clicked, this, &MainWindow::showConfigPanel);
    QObject::connect(ui.btn_quit, &QPushButton::clicked, this, &MainWindow::exit);

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

    // 设置页面以全屏显示
    QDesktopWidget desktop;
    int screenX = desktop.availableGeometry().width();
    int screenY = desktop.availableGeometry().height();
    this->resize(screenX, screenY);

    // 设置可变相机画面大小
    tempWidth = labelWidth = ui.camera->width();
    tempHeight = labelHeight = ui.camera_0->height();
    // std::cout << "init temp: "<< tempWidth << " label: " << labelWidth << std::endl;
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
                labelWidth = tempWidth;
                labelHeight = tempHeight;
                // std::cout << "恢复正常: "<< tempWidth << " label: " << labelWidth << std::endl;
                ui.camera_1->setVisible(true);
                ui.camera_2->setVisible(true);
                ui.camera_3->setVisible(true);
            } else {
                // 当前 camera_0 为正常显示，要将其最大化
                // 要记录正常状态大小
                // std::cout << "恢复正常" << std::endl;
                labelWidth = ui.camera->width();
                labelHeight = ui.camera->height();
                // std::cout << "最大化: "<< tempWidth << " label: " << labelWidth << std::endl;
                ui.camera_1->setVisible(false);
                ui.camera_2->setVisible(false);
                ui.camera_3->setVisible(false);
            }
            video0Max = !video0Max;
        } else {

        }
    }
    return QObject::eventFilter(obj, event);
}

/******************** 槽函数 *********************/

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
    qimage_mutex_.lock();
    cv::Mat showImg;

    int cam_index = 1;
    int interval = 3;
    processOD(image, interval, cam_index);

    cvtColor(image, showImg, CV_BGR2RGB);
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
    cv::Mat showImg;

    int cam_index = 0;
    int interval = 3;
    processOD(image, interval, cam_index);
    
    cvtColor(image, showImg, CV_BGR2RGB);
    QImage img = QImage((const unsigned char*)(showImg.data), showImg.cols, 
                                        showImg.rows, showImg.step, QImage::Format_RGB888);
    QImage scaleImage = img.scaled(ui.camera_0->width(), ui.camera_0->height());
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
        detec_info->track_boxes.clear();
        detec_info->track_classIds.clear();
        detec_info->track_confidences.clear();
        objectD->runODModel(image, cam_index);
    } else {
        // 跟踪算法
        // objectD->runTrackerModel(image);
    }
    detec_info->index = (detec_info->index + 1) % 30;
    
    for (unsigned int i=0; i< detec_info->track_boxes.size(); i++) {
        int x = detec_info->track_boxes[i].x;
        int y = detec_info->track_boxes[i].y;
        int width = detec_info->track_boxes[i].width;
        int height = detec_info->track_boxes[i].height;
        objectD->drawPred(detec_info->track_classIds[i], detec_info->track_confidences[i], 
                        x, y, x+width, y+height, image);
    }
}

void MainWindow::exit() { 
    close();
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

