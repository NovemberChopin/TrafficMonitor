#ifndef mul_t_TRAFFIC_DETAIL_HPP
#define mul_t_TRAFFIC_DETAIL_HPP

#include <QWidget>
#include <QString>
#include <QLabel>
#include "qnode.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>

#include "ui_traffic_detail.h"

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


class TrafficDetail : public QWidget
{
    Q_OBJECT

public:
    explicit TrafficDetail(QWidget *parent = 0);
    ~TrafficDetail();

    void showTrafficImage(cv::Mat image);

    void closePanal() {
        this->close();
    }
Q_SIGNALS:


private:
    Ui::TrafficDetail* ui;
};


#endif