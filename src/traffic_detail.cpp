
#include <QDebug>
#include <QDesktopWidget>
#include "../include/mul_t/traffic_detail.hpp"

using namespace std;

TrafficDetail::TrafficDetail(QWidget *parent) : QWidget(parent) {

    this->setFixedSize(600, 400);
    QDesktopWidget desktop;

    int screenX=desktop.availableGeometry().width();
    int screenY=desktop.availableGeometry().height();
    int winX = this->width();
    int winY = this->height();
    QPoint movePoint(screenX/2 - winX/2, screenY/2 - winY/2);
    this->move(movePoint);
}

TrafficDetail::~TrafficDetail() {}

