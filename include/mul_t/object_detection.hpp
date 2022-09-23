
#ifndef mul_t_OBJECT_DETECTION_HPP
#define mul_t_OBJECT_DETECTION_HPP

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

#include <QThread>
#include <std_msgs/String.h>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/tracking.hpp>
#include <cv_bridge/cv_bridge.h>
#include <QImage>


using namespace cv;
using namespace dnn;
using namespace std;

struct DetectionInfo
{
	int index;
	vector<Rect> track_boxes;
	vector<int> track_classIds;
	vector<float> track_confidences;
	DetectionInfo(int index) : index(index) {}
};


class ObjectDetection
{
private:
    Net net;
	vector<string> classes;

	int inpWidth = 416;
	int inpHeight = 416;
	float confThreshold = 0.5; // Confidence threshold
	float nmsThreshold = 0.4;  // Non-maximum suppression threshold

	std::string classesFile = "/home/js/leishen_ws/src/mul_t/resources/coco.names";

	// cv::String modelConfiguration = "/home/js/leishen_ws/src/mul_t/resources/yolov4-tiny.cfg";
    // cv::String modelWeights = "/home/js/leishen_ws/src/mul_t/resources/yolov4-tiny.weights";

	cv::String modelConfiguration = "/home/js/leishen_ws/src/mul_t/resources/yolo-fastest-xl.cfg";
    cv::String modelWeights = "/home/js/leishen_ws/src/mul_t/resources/yolo-fastest-xl.weights";
	vector<string> trackerTypes = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"}; 

public:
	// vector<Rect> track_boxes;
	// vector<int> track_classIds;
	// vector<float> track_confidences;

	vector<DetectionInfo*> detecRes;

	Ptr<MultiTracker> multiTracker;

    ObjectDetection(int camera_num);
    ~ObjectDetection();

	// 物体检测相关函数
    void runODModel(cv::Mat& frame, int cam_index);
	vector<String> getOutputsNames(const Net& net);
	void postprocess(Mat& frame, const vector<Mat>& outs, int cam_index);
	void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);
	// 物体跟踪相关函数
	void runTrackerModel(cv::Mat & frame);
	Ptr<Tracker> createTrackerByName(string trackerType);
};

#endif
