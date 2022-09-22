
#include <QDebug>
#include <QString>
#include <QPushButton>
#include <QDesktopWidget>
#include "../include/mul_t/config_panel.hpp"

using namespace std;

ConfigPanel::ConfigPanel(QWidget *parent) :
    QWidget(parent)
    // ui(new ConfigPanel)
{
    ui = new Ui::ConfigPanel();
    ui->setupUi(this);

    QObject::connect(ui->ros_connect, &QPushButton::clicked, this, &ConfigPanel::ros_connect_clicked);

    ui->ros_address->setText("http://192.168.50.23:11311");
    ui->ros_port->setText("11311");
    ui->ros_topic->setText("/camera/hik_image");

    initWindow();
}

ConfigPanel::~ConfigPanel()
{
    delete ui;
}


void ConfigPanel::initWindow() {

    // 设置窗体居中显示，并且不能更改大小
    this->setFixedSize(400, 300);
    QDesktopWidget desktop;
    int screenX=desktop.availableGeometry().width();
    int screenY=desktop.availableGeometry().height();
    int winX = this->width();
    int winY = this->height();
    QPoint movePoint(screenX/2 - winX/2, screenY/2 - winY/2);
    this->move(movePoint);

    //设置窗体阻塞所有窗体
    setWindowModality(Qt::WindowModality::ApplicationModal);
    //设置窗体标题栏为没有按钮
    // setWindowButtonType(QBasePara::TypeDialog::None_Dialog);
    //设置窗体关闭后是否释放内存
	setAttribute(Qt::WA_DeleteOnClose, false);
}

void ConfigPanel::ros_connect_clicked() {

    QString ros_address = ui->ros_address->text();
    QString ros_port = ui->ros_port->text();
    QString ros_topic = ui->ros_topic->text();
    Q_EMIT ros_input_over(ros_address, ros_port, ros_topic);

    // 判断是否连接成功
    // 1. 初始化 节点

    this->close();
}
