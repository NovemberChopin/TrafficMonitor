
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

    ui->ros_address->setText("http://127.0.0.1:11311");
    ui->ros_port->setText("127.0.0.1");
    ui->ros_topic_1->setText("/hik_cam_node1/hik_camera");
    ui->ros_topic_2->setText("/hik_cam_node2/hik_camera");
    ui->ros_topic_3->setText("/hik_cam_node3/hik_camera");
    ui->ros_topic_4->setText("/hik_cam_node4/hik_camera");
    initWindow();
}

ConfigPanel::~ConfigPanel()
{
    delete ui;
}


void ConfigPanel::initWindow() {

    // 设置窗体居中显示，并且不能更改大小
    this->setFixedSize(400, 500);
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
    ConfigInfo *configInfo = new ConfigInfo();
    configInfo->ros_address = ui->ros_address->text();
    configInfo->localhost = ui->ros_port->text();

    QString topic_1 = ui->ros_topic_1->text();
    if (!topic_1.isEmpty())
        configInfo->imageTopics.push_back(topic_1);
    QString topic_2 = ui->ros_topic_2->text();
    if (!topic_2.isEmpty())
        configInfo->imageTopics.push_back(topic_2);
    QString topic_3 = ui->ros_topic_3->text();
    if (!topic_3.isEmpty())
        configInfo->imageTopics.push_back(topic_3);    
    QString topic_4 = ui->ros_topic_4->text();
    if (!topic_4.isEmpty())
        configInfo->imageTopics.push_back(topic_4);
    
    Q_EMIT getConfigInfo(configInfo);

    // 判断是否连接成功
    // 1. 初始化 节点

    this->close();
}
