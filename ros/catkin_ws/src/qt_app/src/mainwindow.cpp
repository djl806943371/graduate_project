#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "std_msgs/String.h"
#include <signal.h>
#include <iostream>
#include <QtGui>
#include <QMessageBox>
#include <qt_app/turtle.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

using namespace std;

MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    argc1(argc),
    argv1(argv)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
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


void MainWindow::on_pushButton_3_clicked()
{
    MyButton = new CustomButton(this);
    MyButton->show();
}
