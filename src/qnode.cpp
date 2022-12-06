

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
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        img = cv_ptr->image;
		Q_EMIT getImage(img, cam_index);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


/**
 * @brief 压缩图像回调函数
 */
void QNode::Callback_C1(const sensor_msgs::CompressedImageConstPtr &msg) {
	cv::Mat img;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
		// cv::imshow("img", cv_ptr->image);
		// cv::waitKey(0);
		Q_EMIT getImage(cv_ptr->image, 0);		// cam_index 从 0 开始
    } catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->format.c_str());
    }
}


void QNode::Callback_C2(const sensor_msgs::CompressedImageConstPtr &msg) {
	cv::Mat img;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
		Q_EMIT getImage(cv_ptr->image, 1);		// cam_index 从 0 开始
    } catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->format.c_str());
    }
}


void QNode::Callback_C3(const sensor_msgs::CompressedImageConstPtr &msg) {
	cv::Mat img;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
		Q_EMIT getImage(cv_ptr->image, 2);		// cam_index 从 0 开始
    } catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->format.c_str());
    }
}


void QNode::Callback_C4(const sensor_msgs::CompressedImageConstPtr &msg) {
	cv::Mat img;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
		Q_EMIT getImage(cv_ptr->image, 3);		// cam_index 从 0 开始
    } catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->format.c_str());
    }
}


bool QNode::init(ConfigInfo *config) {
	this->configInfo = config;
	std::map<std::string,std::string> remappings;
	remappings["__master"] = config->ros_address.toStdString();
	remappings["__hostname"] = config->localhost.toStdString();
	// ros::init(remappings, "traffic_monitor");
	ros::init(init_argc, init_argv, "traffic_monitor");
	if (!ros::master::check()) {
		return false;
	}
	ros::start();

	start();
	return true;
}


void QNode::run() {
	// 订阅多个Topic，每个Subscriber有一个Callback queue
	std::string topic;
	ros::NodeHandle n_a;
	ros::CallbackQueue callback_queue_a;
	n_a.setCallbackQueue(&callback_queue_a);
	// 0 表示 Callback 的第二个参数 cam_index
	topic = this->configInfo->imageTopics.at(0).toStdString();
	if (topic.find("compressed") != std::string::npos) {
		sub_img_C1 = n_a.subscribe(topic, 1, &QNode::Callback_C1, this);
	} else {
		image_transport::ImageTransport it(n_a);
		image_sub = it.subscribe(topic, 1, boost::bind(&QNode::Callback, this, _1, 0));
	}
	std::thread spinner_thread_a([&callback_queue_a](){
		ros::SingleThreadedSpinner spinner_a;
		spinner_a.spin(&callback_queue_a);
	});
	
	ros::NodeHandle n_b;
	ros::CallbackQueue callback_queue_b;
	n_b.setCallbackQueue(&callback_queue_b);
	topic = this->configInfo->imageTopics.at(1).toStdString();
	if (topic.find("compressed") != std::string::npos) {
		sub_img_C2 = n_b.subscribe(topic, 1, &QNode::Callback_C2, this);
	} else {
		image_transport::ImageTransport it_b(n_b);
		image_sub2 = it_b.subscribe(topic, 1, boost::bind(&QNode::Callback, this, _1, 1));
	}
	std::thread spinner_thread_b([&callback_queue_b](){
		ros::SingleThreadedSpinner spinner_b;
		spinner_b.spin(&callback_queue_b);
	});

	ros::NodeHandle n_c;
	ros::CallbackQueue callback_queue_c;
	n_c.setCallbackQueue(&callback_queue_c);
	topic = this->configInfo->imageTopics.at(2).toStdString();
	if (topic.find("compressed") != std::string::npos) {
		sub_img_C3 = n_c.subscribe(topic, 1, &QNode::Callback_C3, this);
	} else {
		image_transport::ImageTransport it_c(n_c);
		image_sub3 = it_c.subscribe(topic, 1, boost::bind(&QNode::Callback, this, _1, 2));
	}
	std::thread spinner_thread_c([&callback_queue_c](){
		ros::SingleThreadedSpinner spinner_c;
		spinner_c.spin(&callback_queue_c);
	});
	
	ros::NodeHandle n_d;
	ros::CallbackQueue callback_queue_d;
	n_d.setCallbackQueue(&callback_queue_d);
	topic = this->configInfo->imageTopics.at(3).toStdString();
	if (topic.find("compressed") != std::string::npos) {
		sub_img_C4 = n_d.subscribe(topic, 1, &QNode::Callback_C4, this);
	} else {
		image_transport::ImageTransport it_d(n_d);
		image_sub4 = it_d.subscribe(topic, 1, boost::bind(&QNode::Callback, this, _1, 3));
	}
	std::thread spinner_thread_d([&callback_queue_d](){
		ros::SingleThreadedSpinner spinner_d;
		spinner_d.spin(&callback_queue_d);
	});
	spinner_thread_a.join();
	spinner_thread_b.join();
	spinner_thread_c.join();
	spinner_thread_d.join();

	// int topic_num = this->configInfo->imageTopics.size();
	// std::vector<std::thread> topic_threads;
	// std::vector<image_transport::Subscriber> image_subs;

	// for (int i=0; i<topic_num; i++) {
	// 	ros::NodeHandle n;
	// 	ros::CallbackQueue callback_queue;
	// 	n.setCallbackQueue(&callback_queue);
	// 	image_transport::ImageTransport it(n);
	// 	// 0 表示 Callback 的第二个参数 cam_index
	// 	image_transport::Subscriber image_sub;
	// 	image_sub = it.subscribe(this->configInfo->imageTopics[i].toStdString(), 
	// 								1, boost::bind(&QNode::Callback, this, _1, i));
	// 	image_subs.push_back(image_sub);
	// 	std::thread spinner_thread([&callback_queue](){
	// 		ros::SingleThreadedSpinner spinner;
	// 		spinner.spin(&callback_queue);
	// 	});
	// 	topic_threads.push_back(spinner_thread);
	// }

	// for (int i=0; i<topic_num; i++) {
	// 	topic_threads[i].join();
	// }
}


