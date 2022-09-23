
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

	void Show_img(QImage image, QByteArray res);

	void setEventTable();
	void consoleLog(QString level, QString result, QString opera);

	void showConfigPanel();

	void initial();

protected:
	bool eventFilter(QObject *obj, QEvent *event);

public Q_SLOTS:
	void connectByConfig(QString ros_address, QString ros_port, QString ros_topic);
	// void on_button_connect_clicked(bool check );
	// void on_checkbox_use_environment_stateChanged(int state);

	// 处理接收图片的信号槽
    void setImage1(cv::Mat image);
	void setImage2(cv::Mat image);
	
	void exit();
private:
	Ui::MainWindowDesign ui;
  	QNode qnode;
  	mutable QMutex qimage_mutex_;

	ConfigPanel *configP;
	ObjectDetection *objectD;

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
