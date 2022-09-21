
#include <QDebug>
#include <QString>
#include <QPushButton>
#include "../include/mul_t/config_panel.hpp"

using namespace std;

ConfigPanel::ConfigPanel(QWidget *parent) :
    QWidget(parent)
    // ui(new ConfigPanel)
{
    ui = new Ui::ConfigPanel();
    ui->setupUi(this);

    QObject::connect(ui->ros_connect, &QPushButton::clicked, this, &ConfigPanel::ros_connect_clicked);

    ui->ros_address->setText("192.168.50.23");
    ui->ros_port->setText("11311");
    ui->ros_topic->setText("/camera/hik_image");

    initWindow();
}

ConfigPanel::~ConfigPanel()
{
    delete ui;
}


void ConfigPanel::initWindow() {
    resize(400, 300);
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
    // qDebug() << ros_address << " " << ros_port << " " << ros_topic;
    Q_EMIT ros_input_over(ros_address, ros_port, ros_topic);
    this->close();
}
