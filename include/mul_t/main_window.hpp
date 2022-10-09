
#ifndef mul_t_MAIN_WINDOW_H
#define mul_t_MAIN_WINDOW_H

#include <opencv2/opencv.hpp>
#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "tools.hpp"
#include "object_detection.hpp"
#include "config_panel.hpp"
#include "traffic_detail.hpp"
#include <QImage>
#include <QFileDialog>
#include <QMutex>
#include <cmath>


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

	void addTrafficEvent(QString type, QString level, QString result, QString opera);
	void addTrafficEvent(TrafficEvent* traffic);

	void processOD(cv::Mat &image, int interval, int cam_index);

	void initial();

	void loadCameraMatrix2();
	void loadCameraMatrix();	// 生成相机的姿态（旋转和平移）

	cv::Point3f cameraToWorld(cv::Point2f point, int cam_index);

	cv::Point2f getPixelPoint(Rect &rect);		// 计算检测框的像素坐标

protected:
	bool eventFilter(QObject *obj, QEvent *event);

public Q_SLOTS:
	
	// 处理配置弹窗
	void connectByConfig(ConfigInfo*);

	// 处理接收图片的信号槽
    void setImage(cv::Mat image, int cam_index);
	
	void exit();
	// QTableWidget 单元格点击事件 (展示事件详情)
	void consoleClick(QTableWidgetItem* item);
	void showConfigPanel();

private:
	Ui::MainWindowDesign ui;
  	mutable QMutex qimage_mutex_;
	QNode qnode;				// ros节点相关
	ConfigPanel *configP;		// 配置对话框
	TrafficDetail *trafficD;	// 交通事件回放对话框
	ObjectDetection *objectD;	// 检测（跟踪）算法对象

	// 相机参数
	bool hasLoadCameraMatrix;						// 是否加载了相机相关参数
	std::vector<cv::Mat> vec_cameraMatrix;			// 相机内参矩阵
	std::vector<cv::Mat> vec_distCoeffs;			// 相机畸变参数
	std::vector<cv::Mat> vec_rotationMatrix, vec_transVector;	// 相机姿态
	std::vector<cv::Mat> vec_map1, vec_map2;		// 畸变修复映射矩阵
	std::vector<cv::Point3f> vec_cameraCoord;		// 相机在世界坐标下的位置
	cv::Size image_size;

	int cam_num; 							// 相机数量		
	int interval;							// 目标检测间隔
	bool needSave = true;					// 临时成员变量

	vector<TrafficEvent*> trafficList;		// 交通事件列表

	// 调整窗口相关成员变量
	bool firstImage;
	bool videoMax;							// 是否为单（最大化）窗口播放
	int labelWidth, labelHeight;
};

}  // namespace mul_t

#endif // mul_t_MAIN_WINDOW_H
