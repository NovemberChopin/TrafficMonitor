
#include <QDebug>
#include <QMessageBox>
#include <QDesktopWidget>
#include <QPushButton>
#include "../include/mul_t/traffic_detail.hpp"
#include <opencv2/opencv.hpp>
#include <QLabel>
#include"QMouseEvent"

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
    this->trafficLinePage = this->init_trafficLine_page();
    this->configCameraPage = this->init_configCamera_page();
    this->stackWidget = new QStackedWidget(this);
    this->stackWidget->addWidget(showImagePage);
    this->stackWidget->addWidget(getROIPage);
    this->stackWidget->addWidget(trafficLinePage);
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
    this->roiLabel->setFirst(true);
    this->close();
}


void TrafficDetail::switchPage(int index) {
    this->stackWidget->setCurrentIndex(index);
}


void TrafficDetail::confirmROI() {
    this->roiLabel->setFirst(true);
    QRect roi = this->roiLabel->getRoiRect();
    qDebug() << roi;
    // TODO 这里获取到ROI信息，然后将 roi 和 cam_index 发送回去
    // 这里的 cam_index 应该是 MainWindow 发送过来的
    Q_EMIT this->getROI(roi, this->cam_index, event_index);
    this->close();
}

void TrafficDetail::confirmPoints(){
    if(two_points.size()<2){
        QMessageBox::information(this, "提示", "选取点少于两个！请重新选择"); 
        two_points.clear();
    }
    else if(two_points.size()>2){
        QMessageBox::information(this, "提示", "选取点多于两个！请重新选择"); 
        two_points.clear();
    }
    else{
        int x1 = two_points[0].x;
        int y1 = -two_points[0].y;
        int x2 = two_points[1].x;
        int y2 = -two_points[1].y;
        double k = (double)(y2-y1)/(double)(x2-x1);
        double b = (double)y1 - k*(double)x1;
        std::cout<<"k: "<<k<<"b: "<<b<<std::endl;
        Q_EMIT this->getLine(k, b, this->cam_index, event_index);
        this->close();
    }
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

// 初始化选择车道线页面
QWidget* TrafficDetail::init_trafficLine_page(){
    QWidget *image_page = new QWidget(this);
    QPushButton *btn_close = new QPushButton("返回", this);
    QPushButton *btn_confirm = new QPushButton("确认", this);
    QObject::connect(btn_close, &QPushButton::clicked, this, &TrafficDetail::closePanal);
    QObject::connect(btn_confirm, &QPushButton::clicked, this, &TrafficDetail::confirmPoints);
    // 页面布局
    QVBoxLayout *showImageLayout = new QVBoxLayout;
    QHBoxLayout *sub_layout = new QHBoxLayout;
    sub_layout->addStretch();
    sub_layout->addWidget(btn_confirm);
    sub_layout->addStretch();
    sub_layout->addWidget(btn_close);
    sub_layout->addStretch();
    label1 = new QLabel(this);
    showImageLayout->addWidget(label1);
    showImageLayout->addLayout(sub_layout);
    image_page->setLayout(showImageLayout);
    return image_page;
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
    QImage img = QImage((const unsigned char*)(frame.data), frame.cols, 
                                        frame.rows, frame.step, QImage::Format_RGB888);
    if (stackWidget->currentIndex() == 0) {
        this->label->setMinimumSize(img.width(), img.height());
        this->label->setPixmap(QPixmap::fromImage(img));
    } else if (stackWidget->currentIndex() == 1) {
        this->roiLabel->setBackImage(img);
    } else if (stackWidget->currentIndex() == 2){
        this->label1->setMinimumSize(img.width(), img.height());
        this->label1->setPixmap(QPixmap::fromImage(img));
    }

}

void TrafficDetail::closeEvent(QCloseEvent *e){
	this->roiLabel->setFirst(true);
    two_points.clear();
}

void TrafficDetail::setIndexParam(int cam, int event){
    cam_index = cam;
    event_index = event;
}

void TrafficDetail::showPopInfo(int index) {
    if(index == 0){
       QMessageBox::information(this, "提示", "请点击车道线上任意两点以确定车道线位置！"); 
    }
    else{
        QMessageBox::information(this, "提示", "请拖动画面选择检测区域！");
    }
}

void TrafficDetail::mousePressEvent(QMouseEvent *e)
{
    if(e->button()==Qt::LeftButton)
    {   
        // stackWidget.currentWidget();
        int x = e->x();
        int y = e->y();
        qDebug()<< x <<" "<< y;
        cv::Point2i p(x, y);
        two_points.push_back(p);
    }
}

