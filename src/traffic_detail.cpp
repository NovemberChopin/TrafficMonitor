
#include <QDebug>
#include <QDesktopWidget>
#include <QPushButton>
#include "../include/mul_t/traffic_detail.hpp"
#include <opencv2/opencv.hpp>
#include <QLabel>

using namespace std;

TrafficDetail::TrafficDetail(QWidget *parent) : QMainWindow(parent) {

    centerWgt = new QWidget(this);
    label = new MyLabel(this);
    btn_close = new QPushButton("返回", this);
    
    QObject::connect(btn_close, &QPushButton::clicked, this, &TrafficDetail::closePanal);
    
    QDesktopWidget desktop;
    int screenX=desktop.availableGeometry().width();
    int screenY=desktop.availableGeometry().height();
    int winX = this->width();
    int winY = this->height();
    QPoint movePoint(screenX/2 - winX/2, screenY/2 - winY/2);
    this->move(movePoint);

    // 页面布局
    QVBoxLayout *mainLayout = new QVBoxLayout;
    QHBoxLayout *sub_layout = new QHBoxLayout;
    sub_layout->addStretch();
    sub_layout->addWidget(btn_close);
    sub_layout->addStretch();

    mainLayout->addWidget(label);
    mainLayout->addLayout(sub_layout);
    mainLayout->addStretch();
    centerWgt->setLayout(mainLayout);

    this->setCentralWidget(centerWgt);
    this->resize(800, 600);
}


TrafficDetail::~TrafficDetail() {}


void TrafficDetail::showTrafficImage(cv::Mat frame) {
    // std::cout << frame.rows << " " << frame.cols << std::endl;
    cv::Mat showImg = frame;
    QImage img = QImage((const unsigned char*)(showImg.data), showImg.cols, 
                                        showImg.rows, showImg.step, QImage::Format_RGB888);
    this->label->setBackImage(img);
}

