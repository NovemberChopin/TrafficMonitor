#ifndef mul_t_TRAFFIC_DETAIL_HPP
#define mul_t_TRAFFIC_DETAIL_HPP

#include <QWidget>
#include <QString>
#include "qnode.hpp"


class TrafficDetail : public QWidget
{
    Q_OBJECT

public:
    explicit TrafficDetail(QWidget *parent = 0);
    ~TrafficDetail();

    // void ros_connect_clicked();

Q_SIGNALS:
    // void getConfigInfo(ConfigInfo *config);

private:

    // void initWindow();
};


#endif