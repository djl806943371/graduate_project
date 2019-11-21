#include "mainwindow.h"
#include <QApplication>
#include "settingsdialog.h"
#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <QObject>



int main(int argc, char **argv)
{
    QApplication a(argc, argv);
    MainWindow w(argc, argv);
    w.show();
    return a.exec();
}
