
#include <QDebug>
#include <QDesktopWidget>
#include <QPushButton>
#include "../include/mul_t/traffic_detail.hpp"
#include <opencv2/opencv.hpp>
#include <QLabel>

using namespace std;

TrafficDetail::TrafficDetail(QWidget *parent) : QWidget(parent) {

    ui = new Ui::TrafficDetail();
    ui->setupUi(this);

    this->setFixedSize(800, 600);
    QDesktopWidget desktop;

    QObject::connect(ui->btn_close, &QPushButton::clicked, this, &TrafficDetail::closePanal);

    int screenX=desktop.availableGeometry().width();
    int screenY=desktop.availableGeometry().height();
    int winX = this->width();
    int winY = this->height();
    QPoint movePoint(screenX/2 - winX/2, screenY/2 - winY/2);
    this->move(movePoint);
}

TrafficDetail::~TrafficDetail() {
    delete ui;
}


void TrafficDetail::showTrafficImage(cv::Mat frame) {
    // std::cout << frame.rows << " " << frame.cols << std::endl;
    // cv::imshow("frame", frame);
    cv::Mat showImg = frame;
    // cvtColor(frame, showImg, CV_BGR2RGB);
    QImage img = QImage((const unsigned char*)(showImg.data), showImg.cols, 
                                        showImg.rows, showImg.step, QImage::Format_RGB888);
    QImage scaleImage = img.scaled(640, 360, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    ui->label->setScaledContents(true);
    ui->label->setPixmap(QPixmap::fromImage(scaleImage));
}

