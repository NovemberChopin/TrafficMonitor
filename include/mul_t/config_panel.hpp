#ifndef mul_t_CONFIG_PANEL_HPP
#define mul_t_CONFIG_PANEL_HPP

#include <QWidget>
#include "ui_config_panel.h"
#include <QString>



class ConfigPanel : public QWidget
{
    Q_OBJECT

public:
    explicit ConfigPanel(QWidget *parent = 0);
    ~ConfigPanel();

    void ros_connect_clicked();

Q_SIGNALS:
    void ros_input_over(QString ros_address, QString ros_port, QString ros_topic);

private:

    void initWindow();

    Ui::ConfigPanel* ui;
};


#endif
