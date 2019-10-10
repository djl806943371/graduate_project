#include "ui_mainwindow.h"
#include "mainwindow.h"
#include "std_msgs/String.h"
#include <signal.h>
#include <iostream>
#include <QtGui>
#include <QMessageBox>
#include <QListWidgetItem>
#include <qt_app/turtle.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <QSerialPort>
#include <QtWidgets/QVBoxLayout>
#include <QString>
#include <crc.h>

using namespace std;

MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    argc1(argc),
    argv1(argv),
    m_settings(new SettingsDialog),
    m_serial(new QSerialPort(this)),
    m_status(new QLabel)
{
    ui->setupUi(this);
    //    this->setAttribute(Qt::WA_DeleteOnClose,true);
    ui->statusBar->addWidget(m_status);
    connect(ui->actionConfigure, &QAction::triggered, m_settings, &SettingsDialog::show);
    connect(ui->actionConnect, &QAction::triggered, this, &MainWindow::openSerialPort);
    connect(ui->actionDisconnect, &QAction::triggered, this, &MainWindow::closeSerialPort);
    connect(m_serial, &QSerialPort::readyRead, this, &MainWindow::readMyCom);

    //    setLayout(ui->horizontalLayout);
}

MainWindow::~MainWindow()
{
    delete m_settings;
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    delete m_settings;
    delete ui;
}

void MainWindow::openSerialPort()
{
    const SettingsDialog::Settings p = m_settings->settings();
    m_serial->setPortName(p.name);
    m_serial->setBaudRate(p.baudRate);
    m_serial->setDataBits(p.dataBits);
    m_serial->setParity(p.parity);
    m_serial->setStopBits(p.stopBits);
    m_serial->setFlowControl(p.flowControl);
    if (m_serial->open(QIODevice::ReadWrite)) {
        ui->actionConnect->setEnabled(false);
        ui->actionDisconnect->setEnabled(true);
        ui->actionConfigure->setEnabled(false);
        showStatusMessage(tr("Connected to %1 : %2, %3, %4, %5, %6")
                          .arg(p.name).arg(p.stringBaudRate).arg(p.stringDataBits)
                          .arg(p.stringParity).arg(p.stringStopBits).arg(p.stringFlowControl));
    } else {
        QMessageBox::critical(this, tr("Error"), m_serial->errorString());

        showStatusMessage(tr("Open error"));
    }
}

void MainWindow::closeSerialPort()
{
    if (m_serial->isOpen())
        m_serial->close();
    ui->actionConnect->setEnabled(true);
    ui->actionDisconnect->setEnabled(false);
    ui->actionConfigure->setEnabled(true);
    showStatusMessage(tr("Disconnected"));
}

void MainWindow::showStatusMessage(const QString &message)
{
    m_status->setText(message);
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void MainWindow::on_pushButton_clicked(bool checked)
{
    if(checked){
        ros::init(argc1, argv1, "test_gui");
        if (!ros::master::check())
        {
            return;
        }
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle n;
        ros::NodeHandle nSub;
        // Add your ros communications here.
        chatter_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
        //        chatter_subscriber = nSub.subscribe("testgui_chat", 100, chatterCallback);
        return;
    }
    else
        ros::shutdown();
}

void MainWindow::on_pushButton_2_pressed()
{
    geometry_msgs::Twist msg;
    msg.linear.x = 2.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 1.8;
    chatter_publisher.publish(msg);

    return;
}

void MainWindow::on_sendButton_clicked()
{
    QString str = ui->sendEdit->text();//从LineEdit得到字符串
    str += crc16ForModbus(str);
    QByteArray sendData  = QByteArray::fromHex(str.toLocal8Bit());
    m_serial->write(sendData);//发送到串口
    str.clear();
    str = "Send:\t" +  sendData.toHex().toUpper();
    QListWidgetItem * listItem = new QListWidgetItem(str);
    ui->logWidget->addItem(listItem);
    ui->logWidget->setCurrentRow(ui->logWidget->count() - 1);
}

void MainWindow::readMyCom()//读取缓冲的数据
{
    QByteArray receiveData = m_serial->readAll();
    QString str = "Receive:\t" + receiveData.toHex().toUpper();
    QListWidgetItem * listItem = new QListWidgetItem(str);
    listItem->setBackground(Qt::green);
    ui->logWidget->addItem(listItem);
    ui->logWidget->setCurrentRow(ui->logWidget->count() - 1);
}
