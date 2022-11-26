

#ifndef mul_t_QNODE_HPP_
#define mul_t_QNODE_HPP_



#include <ros/ros.h>
#include <ros/network.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

#include <QThread>
#include <QString>
#include <QStringList>

#include <thread>
#include <std_msgs/String.h>
#include <opencv2/dnn.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


struct ConfigInfo {
	QString ros_address;
	QString localhost;
	std::vector<QString> imageTopics;
};



class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init(ConfigInfo *config);

	void run();

  	void Callback(const sensor_msgs::ImageConstPtr &msg, int cam_index); //camera callback function

	void Callback_C1(const sensor_msgs::CompressedImageConstPtr &msg);
	void Callback_C2(const sensor_msgs::CompressedImageConstPtr &msg);
	void Callback_C3(const sensor_msgs::CompressedImageConstPtr &msg);
	void Callback_C4(const sensor_msgs::CompressedImageConstPtr &msg);

Q_SIGNALS:
    void rosShutdown();

    void getImage(cv::Mat, int cam_index);

private:
	int init_argc;
	char** init_argv;

	ConfigInfo *configInfo;
	
  	image_transport::Subscriber image_sub;
	image_transport::Subscriber image_sub2;
	image_transport::Subscriber image_sub3;
	image_transport::Subscriber image_sub4;

	ros::Subscriber sub_img_C1;		// 订阅压缩图像
	ros::Subscriber sub_img_C2;
	ros::Subscriber sub_img_C3;
	ros::Subscriber sub_img_C4;
	
};



#endif
