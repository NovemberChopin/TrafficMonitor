
#include <QDebug>
#include <QDesktopWidget>
#include <QPushButton>
#include "../include/mul_t/traffic_detail.hpp"
#include <opencv2/opencv.hpp>
#include <QLabel>

using namespace std;

TrafficDetail::~TrafficDetail() {}

TrafficDetail::TrafficDetail(QWidget *parent) : QMainWindow(parent) {
    this->resize(800, 600);
    
    QDesktopWidget desktop;
    int screenX=desktop.availableGeometry().width();
    int screenY=desktop.availableGeometry().height();
    int winX = this->width();
    int winY = this->height();
    QPoint movePoint(screenX/2 - winX/2, screenY/2 - winY/2);
    this->move(movePoint);

    this->showImagePage = this->init_showImage_page();
    this->getROIPage = this->init_getROI_page();
    this->configCameraPage = this->init_configCamera_page();
    this->stackWidget = new QStackedWidget(this);
    this->stackWidget->addWidget(showImagePage);
    this->stackWidget->addWidget(getROIPage);
    this->stackWidget->addWidget(configCameraPage);
    this->setCentralWidget(stackWidget);
}


void TrafficDetail::removeLayout(QWidget *wdialog, QLayout *layout) {
    wdialog->hide();
    QLayoutItem *child;
    if (layout == NULL)
        return;
    while ((child = layout->takeAt(0)) != NULL) {
        if (child->widget()) {
            child->widget()->setParent(NULL);
            delete child->widget();/* 修改备注,修复内存泄露 */
        } else if (child->layout()) {
            removeLayout(NULL, child->layout());
        }
        delete child;
        child = NULL;
    }
    delete layout;
    layout = NULL;
}


void TrafficDetail::closePanal() {
    this->close();
}


void TrafficDetail::switchPage(int index) {
    this->stackWidget->setCurrentIndex(index);
}


void TrafficDetail::confirmROI() {
    QRect roi = this->roiLabel->getRoiRect();
    qDebug() << roi;
    // TODO 这里获取到ROI信息，然后将 roi 和 cam_index 发送回去
    // 这里的 cam_index 应该是 MainWindow 发送过来的
    this->close();
}

// 简单展示页面
QWidget* TrafficDetail::init_showImage_page() {
    QWidget *image_page = new QWidget(this);
    QPushButton *btn_close = new QPushButton("返回", this);
    QObject::connect(btn_close, &QPushButton::clicked, this, &TrafficDetail::closePanal);
    // 页面布局
    QVBoxLayout *showImageLayout = new QVBoxLayout;
    QHBoxLayout *sub_layout = new QHBoxLayout;
    sub_layout->addStretch();
    sub_layout->addWidget(btn_close);
    sub_layout->addStretch();
    label = new QLabel(this);
    showImageLayout->addWidget(label);
    showImageLayout->addLayout(sub_layout);
    image_page->setLayout(showImageLayout);
    return image_page;
}

// 可以获取图像ROI区域的页面
QWidget* TrafficDetail::init_getROI_page() {
    QWidget *roi_image_page = new QWidget(this);
    QPushButton *btn_close = new QPushButton("返回", this);
    QPushButton *btn_confirm = new QPushButton("确认", this);
    QObject::connect(btn_close, &QPushButton::clicked, this, &TrafficDetail::closePanal);
    QObject::connect(btn_confirm, &QPushButton::clicked, this, &TrafficDetail::confirmROI);
    // 页面布局
    QVBoxLayout *getROILayout = new QVBoxLayout;
    QHBoxLayout *sub_layout = new QHBoxLayout;
    sub_layout->addStretch();
    sub_layout->addWidget(btn_confirm);
    sub_layout->addStretch();
    sub_layout->addWidget(btn_close);
    sub_layout->addStretch();
    roiLabel = new MyLabel(this);
    getROILayout->addWidget(roiLabel);
    getROILayout->addLayout(sub_layout);
    roi_image_page->setLayout(getROILayout);
    return roi_image_page;
}

// 配置相机页面（计算相机姿态角）
QWidget* TrafficDetail::init_configCamera_page() {
    QWidget *configCamera_page = new QWidget(this);
    QLabel *showLabel = new QLabel(this);
    showLabel->setText("这是配置相机姿态角的页面！");
    showLabel->setAlignment(Qt::AlignCenter);
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(showLabel);
    configCamera_page->setLayout(layout);
    return configCamera_page;
}


void TrafficDetail::showImage(cv::Mat frame) {
    // std::cout << frame.rows << " " << frame.cols << std::endl;
    cv::Mat showImg = frame;
    QImage img = QImage((const unsigned char*)(showImg.data), showImg.cols, 
                                        showImg.rows, showImg.step, QImage::Format_RGB888);
    if (stackWidget->currentIndex() == 0) {
        this->label->setMinimumSize(img.width(), img.height());
        this->label->setPixmap(QPixmap::fromImage(img));
    } else if (stackWidget->currentIndex() == 1) {
        this->roiLabel->setBackImage(img);
    }
    
}

