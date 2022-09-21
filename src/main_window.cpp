
#include <QtGui>
#include <QMessageBox>
#include <QPushButton>
#include <QDebug>
#include <QString>
#include <QStringList>
#include <QHeaderView>
#include <iostream>
#include "../include/mul_t/main_window.hpp"
#include "../include/mul_t/config_panel.hpp"
#include <time.h>


namespace mul_t {

using namespace Qt;


MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); 
	
    // 初始化 connect 页面
    configP = new ConfigPanel();

    // qApp is a global variable for the application
    // QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); 
    QObject::connect(ui.btn_quit, &QPushButton::clicked, this, &MainWindow::exit);

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode,SIGNAL(loggingCamera()),this,SLOT(updateLogcamera()));

    QObject::connect(ui.btn_config, &QPushButton::clicked, this, &MainWindow::showConfigPanel);

    QObject::connect(ui.btn_connect, &QPushButton::clicked, this, [=](){
        if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.btn_connect->setEnabled(false);
		}
    });

    // 匿名函数
    // void (MainWindow:: *getRosConfig)(QString ros_address, QString ros_prot, QString ros_topic) = &MainWindow::connectByConfig;
    // QObject::connect(&configP, &ConfigPanel::ros_input_over, this, getRosConfig);
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
    QImage mimage = image.scaled(ui.label_camera->width(),ui.label_camera->height());
    ui.label_camera->setScaledContents(true);
    ui.label_camera->setPixmap(QPixmap::fromImage(mimage));
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


//update camera topic messages
void MainWindow::updateLogcamera()
{   
    qimage_mutex_.lock();
    cv::Mat showImg;
    // std::cout << qnode.img.rows << " " << qnode.img.cols << std::endl;
    // std::cout << qnode.img.step[0] << " " << qnode.img.step[1] << " " << qnode.img.step[2] << std::endl;
    
    if (objectD.index % 3 == 0) {
        objectD.boxes.clear();
        objectD.track_boxes.clear();
        objectD.track_classIds.clear();
        objectD.track_confidences.clear();
        objectD.runODModel(qnode.img);
    } else {
        // 跟踪算法
        // objectD.runTrackerModel(qnode.img);
    }
    objectD.index = (objectD.index + 1) % 30;

    for (unsigned int i=0; i<objectD.track_boxes.size(); i++) {
        int x = objectD.track_boxes[i].x;
        int y = objectD.track_boxes[i].y;
        int width = objectD.track_boxes[i].width;
        int height = objectD.track_boxes[i].height;
        objectD.drawPred(objectD.track_classIds[i], objectD.track_confidences[i], 
                        x, y, x+width, y+height, qnode.img);
    }
    
    cvtColor(qnode.img, showImg, CV_BGR2RGB);
    QImage image = QImage((const unsigned char*)(showImg.data), showImg.cols, 
                                        showImg.rows, showImg.step, QImage::Format_RGB888);
    ui.label_camera->setPixmap(QPixmap::fromImage(image));
    ui.label_camera->resize(ui.label_camera->pixmap()->size());
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

