#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ros/ros.h"
#include "CustomButton.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char *argv[], QWidget *parent = 0);
    ~MainWindow();

private slots:

    void on_pushButton_clicked(bool checked);

    void on_pushButton_2_pressed();

    void on_pushButton_3_clicked();

private:
    Ui::MainWindow *ui;
    int argc1;
    char **argv1;
    ros::Publisher chatter_publisher;
    ros::Subscriber chatter_subscriber;
    CustomButton *MyButton;
};

#endif // MAINWINDOW_H
