 #include "mainwindow.h"
 #include <QApplication>
 #include "settingsdialog.h"
 #include "ros/ros.h"
 #include <sstream>
 #include "std_msgs/String.h"


 int main(int argc, char **argv)
 {
     QApplication a(argc, argv);
     MainWindow w(argc, argv);
     w.show();
     return a.exec();
 }
