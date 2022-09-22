
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


MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, argc(argc), argv(argv)
{
	ui.setupUi(this); 

    // 设置页面以全屏显示
    QDesktopWidget desktop;
    int screenX = desktop.availableGeometry().width();
    int screenY = desktop.availableGeometry().height();
    this->resize(screenX, screenY);

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
	
    // 初始化 connect 页面
    configP = new ConfigPanel();

    QObject::connect(ui.btn_quit, &QPushButton::clicked, this, &MainWindow::exit);

    // QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    // QObject::connect(&qnode,SIGNAL(loggingCamera()),this,SLOT(setImage1()));

    QObject::connect(ui.btn_config, &QPushButton::clicked, this, &MainWindow::showConfigPanel);

    QObject::connect(ui.btn_set_cam1, &QPushButton::clicked, this, [=](){
        QNode *node_1 = new QNode(argc, argv);
        connect(node_1, SIGNAL(getImage1(cv::Mat)), this, SLOT(setImage1(cv::Mat)));
        if (!node_1->init("Node1", "/hik_cam_node/hik_camera")) {
            showNoMasterMessage();
        } else {
            ui.btn_set_cam1->setEnabled(false);
        }
    });


    QObject::connect(ui.btn_set_cam2, &QPushButton::clicked, this, [=](){
        QNode *node_2 = new QNode(argc, argv);
        connect(node_2, SIGNAL(getImage2(cv::Mat)), this, SLOT(setImage2(cv::Mat)));
        if (!node_2->init("Node2", "/hik_image")) {
            showNoMasterMessage();
        } else {
            ui.btn_set_cam2->setEnabled(false);
        }
    });



    //接收登录页面传来的数据
    connect(configP, SIGNAL(ros_input_over(QString, QString, QString)), 
            this, SLOT(connectByConfig(QString, QString, QString)));

    // if ( ui.checkbox_remember_settings->isChecked() ) {
    //     on_button_connect_clicked(true);
    // }

    // 初始化
    initial();
}

void MainWindow::initial() {
    setWindowIcon(QIcon(":/images/icon.png"));
    setWindowTitle(tr("视频场景监控"));

    ui.consoleTable->setSelectionMode(QAbstractItemView::NoSelection);  // 禁止点击输出窗口的 item
    setEventTable();

    consoleLog("hight", "right", "detial");

    // 设置默认主题
    QString qss = darcula_qss;
    qApp->setStyleSheet(qss);

}

void MainWindow::connectByConfig(QString ros_address, QString ros_port, QString ros_topic) {
    qDebug() << ros_address;
    qDebug() << ros_port;
    qDebug() << ros_topic;

    // if (!qnode.init(ros_address.toStdString(), 
    //         ros_port.toStdString(), ros_topic.toStdString())) {
    // if (!qnode.init()) {
    //     // 连接失败后的操作
    //     showNoMasterMessage();
    // } else {
    //     // 连接成功
    //     ui.btn_config->setEnabled(false);
    // }
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

void MainWindow::showConfigPanel() {
    std::cout << "------------------" << std::endl;
    
    configP->show();
}

void MainWindow::Show_img(QImage image, QByteArray res)
{
    QImage mimage = image.scaled(ui.camera_0->width(),ui.camera_0->height());
    ui.camera_0->setScaledContents(true);
    ui.camera_0->setPixmap(QPixmap::fromImage(mimage));
}


MainWindow::~MainWindow() {}


void MainWindow::exit() {
    close();
}

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("找不到ROS服务器");
	msgBox.exec();
    close();
}

// void MainWindow::mouseDoubleClickEvent(QMouseEvent *event) {
//     if(event->button()==Qt::LeftButton) 
//         qDebug("Left");
//     if(event->button()==Qt::RightButton)
//         qDebug("Right"); 
// }

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


/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

// void MainWindow::on_button_connect_clicked(bool check ) {
// 	if ( ui.checkbox_use_environment->isChecked() ) {
// 		if ( !qnode.init() ) {
// 			showNoMasterMessage();
// 		} else {
// 			ui.button_connect->setEnabled(false);
// 		}
// 	} else {
// 		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
// 				   ui.line_edit_host->text().toStdString()) ) {
// 			showNoMasterMessage();
// 		} else {
// 			ui.button_connect->setEnabled(false);
// 			ui.line_edit_master->setReadOnly(true);
// 			ui.line_edit_host->setReadOnly(true);
// 		}
// 	}
// }


// void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
// 	bool enabled;
// 	if ( state == 0 ) {
// 		enabled = true;
// 	} else {
// 		enabled = false;
// 	}
// 	ui.line_edit_master->setEnabled(enabled);
// 	ui.line_edit_host->setEnabled(enabled);
// }


void MainWindow::setImage2(cv::Mat image) {
    qimage_mutex_.lock();
    cv::Mat showImg;

    cvtColor(image, showImg, CV_BGR2RGB);
    QImage img = QImage((const unsigned char*)(showImg.data), showImg.cols, 
                                        showImg.rows, showImg.step, QImage::Format_RGB888);
    QImage scaleImage = img.scaled(ui.camera_1->width(), ui.camera_1->height());
    ui.camera_1->setScaledContents(true);
    ui.camera_1->setPixmap(QPixmap::fromImage(scaleImage));
    qimage_mutex_.unlock();
}


//update camera topic messages
void MainWindow::setImage1(cv::Mat image)
{   
    qimage_mutex_.lock();
    cv::Mat showImg;
    
    if (objectD.index % 3 == 0) {
        
        objectD.track_boxes.clear();
        objectD.track_classIds.clear();
        objectD.track_confidences.clear();
        objectD.runODModel(image);
    } else {
        // 跟踪算法
        // objectD.runTrackerModel(image);
    }
    objectD.index = (objectD.index + 1) % 30;

    for (unsigned int i=0; i<objectD.track_boxes.size(); i++) {
        int x = objectD.track_boxes[i].x;
        int y = objectD.track_boxes[i].y;
        int width = objectD.track_boxes[i].width;
        int height = objectD.track_boxes[i].height;
        objectD.drawPred(objectD.track_classIds[i], objectD.track_confidences[i], 
                        x, y, x+width, y+height, image);
    }
    
    cvtColor(image, showImg, CV_BGR2RGB);
    QImage img = QImage((const unsigned char*)(showImg.data), showImg.cols, 
                                        showImg.rows, showImg.step, QImage::Format_RGB888);
    QImage scaleImage = img.scaled(ui.camera_0->width(), ui.camera_0->height());
    ui.camera_0->setScaledContents(true);
    ui.camera_0->setPixmap(QPixmap::fromImage(scaleImage));
    qimage_mutex_.unlock();
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

