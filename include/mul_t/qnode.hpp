

#ifndef mul_t_QNODE_HPP_
#define mul_t_QNODE_HPP_


#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

#include <QThread>
#include <std_msgs/String.h>
#include <opencv2/dnn.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();

	bool init(std::string nodeName, std::string topic);

	bool init(const std::string &master_url, const std::string &host_url, const std::string &topic);
	// bool init(const std::string &master_url, const std::string &host_url);
	void run();

  	void myCallback_img(const sensor_msgs::ImageConstPtr& msg);//camera callback function

  	cv::Mat img;

Q_SIGNALS:
    void rosShutdown();
    void getImage1(cv::Mat);
	void getImage2(cv::Mat);
	
private:
	int init_argc;
	char** init_argv;
  	image_transport::Subscriber image_sub;
};



#endif
