

#ifndef mul_t_QNODE_HPP_
#define mul_t_QNODE_HPP_


#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <ros/callback_queue.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

#include <QThread>
#include <thread>
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
	bool init(const std::string &master_url, const std::string &host_url, const std::string &topic);
	
	void run();

  	void Callback(const sensor_msgs::ImageConstPtr &msg, int cam_index); //camera callback function


Q_SIGNALS:
    void rosShutdown();
    void getImage(cv::Mat, int cam_index);

private:
	int init_argc;
	char** init_argv;

	
  	image_transport::Subscriber image_sub;
	image_transport::Subscriber image_sub2;
};



#endif
