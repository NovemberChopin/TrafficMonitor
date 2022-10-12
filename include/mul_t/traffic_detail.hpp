#ifndef mul_t_TRAFFIC_DETAIL_HPP
#define mul_t_TRAFFIC_DETAIL_HPP

#include <QWidget>
#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
#include <QStackedWidget>
#include <QString>
#include <QLabel>
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

    QWidget* init_showImage_page();   // 初始化显示事件图片布局
    QWidget* init_configCamera_page();// 初始化配置相机布局
    QWidget* init_getROI_page();      // 选择ROI页面

    void switchPage(int index);         // 切换要显示的页面

    void removeLayout(QWidget *wdialog, QLayout *layout);

Q_SIGNALS:
    void getROI(QRect roi, int cam_index, int event_index);

private:
    // Ui::TrafficDetail* ui;
    QLabel *label;          // 普通标签
    MyLabel *roiLabel;      // 支持ROI选择的标签

    QStackedWidget *stackWidget;
    QWidget *showImagePage;
    QWidget *getROIPage;
    QWidget *configCameraPage;
};


#endif