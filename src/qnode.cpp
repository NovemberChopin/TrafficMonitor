

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


void QNode::Callback(const sensor_msgs::ImageConstPtr &msg, int cam_index) {
	cv::Mat img;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        img = cv_ptr->image;
		Q_EMIT getImage(img, cam_index);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


bool QNode::init() {
	ros::init(init_argc,init_argv,"Monitor");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start();

	this->start();	// 启动线程
	return true;
}


bool QNode::init(const std::string &master_url, const std::string &host_url, const std::string &topic) {
	// std::map<std::string,std::string> remappings;
	// remappings["__master"] = master_url;
	// remappings["__hostname"] = host_url;
	// ros::init(remappings,"mul_t");
	// if ( ! ros::master::check() ) {
	// 	return false;
	// }
	// ros::start();
	// ros::NodeHandle n;
  	// image_transport::ImageTransport it(n);

  	// image_sub = it.subscribe(topic, 1, &QNode::Callback_1, this);
	// start();
	return true;
}

void QNode::run() {
  	// ros::spin();

	// 订阅多个Topic，多个Spinner threads
	// ros::NodeHandle n;
  	// image_transport::ImageTransport it(n);
    // // image_sub = it.subscribe("/camera/hik_image", 5, &QNode::Callback_1, this);
  	// image_sub = it.subscribe("/hik_cam_node/hik_camera", 5, &QNode::Callback_1, this);
	// image_sub2 = it.subscribe("/hik_image", 1, &QNode::Callback_2, this);
	// ros::MultiThreadedSpinner spinner(2);
	// spinner.spin();


	// 订阅多个Topic，每个Subscriber有一个Callback queue
	ros::NodeHandle n_a;
	ros::CallbackQueue callback_queue_a;
	n_a.setCallbackQueue(&callback_queue_a);
	image_transport::ImageTransport it(n_a);
	// 0 表示 Callback 的第二个参数 cam_index
	image_sub = it.subscribe("/hik_cam_node/hik_camera", 1, boost::bind(&QNode::Callback, this, _1, 0));
	std::thread spinner_thread_a([&callback_queue_a](){
		ros::SingleThreadedSpinner spinner_a;
		spinner_a.spin(&callback_queue_a);
	});

	ros::NodeHandle n_b;
	ros::CallbackQueue callback_queue_b;
	n_b.setCallbackQueue(&callback_queue_b);
	image_transport::ImageTransport it_b(n_b);
	image_sub2 = it_b.subscribe("/hik_image", 1, boost::bind(&QNode::Callback, this, _1, 1));
	std::thread spinner_thread_b([&callback_queue_b](){
		ros::SingleThreadedSpinner spinner_b;
		spinner_b.spin(&callback_queue_b);
	});
	spinner_thread_a.join();
	spinner_thread_b.join();
}


