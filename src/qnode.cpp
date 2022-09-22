

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include "../include/mul_t/qnode.hpp"
#include "sensor_msgs/image_encodings.h"


#include <fstream>
#include <sstream>
#include <iostream>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <time.h>

using namespace cv;
using namespace dnn;
using namespace std;


QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
{
	
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}


void QNode::myCallback_img(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        img = cv_ptr->image;
		std::cout << ros::this_node::getName() << std::endl;
        Q_EMIT getImage1(img);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"Monitor");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start();
	ros::NodeHandle n;
  	image_transport::ImageTransport it(n);
    // image_sub = it.subscribe("/camera/hik_image", 5, &QNode::myCallback_img, this);
  	image_sub = it.subscribe("/hik_cam_node/hik_camera", 5, &QNode::myCallback_img, this);
	start();
	return true;
}

bool QNode::init(std::string nodeName, std::string topic) {
	ros::init(init_argc,init_argv, nodeName);
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start();
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_sub = it.subscribe(topic, 1, &QNode::myCallback_img, this);
	this->start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url, const std::string &topic) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"mul_t");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start();
	ros::NodeHandle n;
  	image_transport::ImageTransport it(n);

  	image_sub = it.subscribe(topic, 1, &QNode::myCallback_img, this);
	start();
	return true;
}

void QNode::run() {
  	ros::spin();
}


