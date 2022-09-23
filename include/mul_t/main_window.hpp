
#ifndef mul_t_MAIN_WINDOW_H
#define mul_t_MAIN_WINDOW_H

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "tools.hpp"
#include "object_detection.hpp"
#include "config_panel.hpp"
#include <QImage>
#include <QMutex>


namespace mul_t {

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	// void ReadSettings(); // Load up qt program settings at startup
	// void WriteSettings(); // Save qt program settings when closing

	// void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

	void setEventTable();

	void consoleLog(QString level, QString result, QString opera);

	void showConfigPanel();

	void processOD(cv::Mat &image, int interval, int cam_index);

	void initial();

protected:
	bool eventFilter(QObject *obj, QEvent *event);

public Q_SLOTS:
	// 处理配置弹窗
	void connectByConfig(QString ros_address, QString ros_port, QString ros_topic);

	// 处理接收图片的信号槽
    void setImage1(cv::Mat image);
	void setImage2(cv::Mat image);
	
	void exit();
private:
	Ui::MainWindowDesign ui;
  	mutable QMutex qimage_mutex_;
	QNode qnode;				// ros节点相关
	ConfigPanel *configP;		// 配置对话框
	ObjectDetection *objectD;	// 检测（跟踪）算法对象

	// 窗口尺寸
	int labelWidth, labelHeight;
	// 临时记录正常尺寸大小
    int tempWidth, tempHeight;
	bool video0Max;
    bool video1Max;
    bool video2Max;
    bool video3Max;
};

}  // namespace mul_t

#endif // mul_t_MAIN_WINDOW_H
