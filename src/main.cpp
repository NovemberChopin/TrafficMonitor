/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/

#include <QtGui>
#include <QApplication>
#include <QMetaType>
#include "../include/mul_t/main_window.hpp"


int main(int argc, char **argv) {

    QApplication app(argc, argv);
    mul_t::MainWindow w(argc,argv);
    w.show();

    qRegisterMetaType<cv::Mat>("cv::Mat");

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
