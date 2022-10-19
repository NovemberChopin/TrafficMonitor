#ifndef mul_t_TRAFFIC_DETAIL_HPP
#define mul_t_TRAFFIC_DETAIL_HPP

#include <QWidget>
#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
#include <QStackedWidget>
#include <QString>
#include <QLabel>
#include"QMouseEvent"
#include "qnode.hpp"
#include "my_label.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>

// #include "ui_traffic_detail.h"

struct TrafficEvent
{
    QString time;
    QString type;
    QString level;
    QString result;
    cv::Mat image;
    TrafficEvent() {}
    TrafficEvent(QString time, QString type, QString level, QString result, cv::Mat image) : 
                time(time), type(type), level(level), result(result), image(image) {}
};


class TrafficDetail : public QMainWindow
{
    Q_OBJECT

public:
    explicit TrafficDetail(QWidget *parent = 0);
    ~TrafficDetail();

    void showImage(cv::Mat image);

    void closePanal();
    void confirmROI();      // 确认选择的ROI槽函数
    void confirmPoints();
    void showPopInfo(int index);

    QWidget* init_showImage_page();   // 初始化显示事件图片布局
    QWidget* init_configCamera_page();// 初始化配置相机布局
    QWidget* init_getROI_page();      // 选择ROI页面
    QWidget* init_trafficLine_page();  // 初始化选择车道线页面

    void switchPage(int index);         // 切换要显示的页面

    void removeLayout(QWidget *wdialog, QLayout *layout);

    void closeEvent(QCloseEvent *);  //当关闭窗口时设置firstImage为true

    void mousePressEvent(QMouseEvent *);

    void setIndexParam(int cam, int event);  //设置画roi时对应的相机和事件

Q_SIGNALS:
    void getROI(QRect roi, int cam_index, int event_index);
    void getLine(double k, double b, int cam_index, int event_index);

private:
    // Ui::TrafficDetail* ui;
    QLabel *label;          // 普通标签
    QLabel *label1;
    MyLabel *roiLabel;      // 支持ROI选择的标签

    int cam_index = -1;  // 设置roi时对应相机
    int  event_index = -1; // 设置roi时对应事件

    QStackedWidget *stackWidget;
    QWidget *showImagePage;
    QWidget *getROIPage;
    QWidget *configCameraPage;
    QWidget *trafficLinePage;
    std::vector<cv::Point2i> two_points;
};


#endif