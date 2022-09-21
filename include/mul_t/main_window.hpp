
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

public Q_SLOTS:
	void connectByConfig(QString ros_address, QString ros_port, QString ros_topic);
	// void on_button_connect_clicked(bool check );
	// void on_checkbox_use_environment_stateChanged(int state);

    void updateLogcamera();
	void exit();
private:
	Ui::MainWindowDesign ui;
  	QNode qnode;
  	mutable QMutex qimage_mutex_;
	ConfigPanel *configP;

	ObjectDetection objectD;
};

}  // namespace mul_t

#endif // mul_t_MAIN_WINDOW_H
