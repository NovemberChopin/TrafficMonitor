#ifndef mul_t_TRAFFIC_DETAIL_HPP
#define mul_t_TRAFFIC_DETAIL_HPP

#include <QWidget>
#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
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

    void showTrafficImage(cv::Mat image);

    void closePanal() {
        this->close();
    }

private:
    // Ui::TrafficDetail* ui;
    MyLabel *label;
    QWidget *centerWgt;
    QPushButton *btn_close;

};


#endif