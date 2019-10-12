#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ros/ros.h"
#include "settingsdialog.h"
#include <QSerialPort>
#include <QLabel>
#include "command.h"
#include "crc.h"
#include <QTimer>

//class QLabel;

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char *argv[], QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void openSerialPort();
    void closeSerialPort();
    void clearLog();
    void on_pushButton_clicked(bool checked);

    void on_pushButton_2_pressed();

    void on_sendButton_clicked();

    void on_goFront_pressed();

    void on_rpmSliderBar_valueChanged(int value);

    void on_goBack_pressed();

    void on_accelerationSliderBar_valueChanged(int value);

    void on_applyButton_clicked();

    void on_rocker_signalButtonMoved(int degree, int angle);

    void on_rocker_signalButtonReleased();
    void on_rocker_signalButtonClicked();

    void on_sendButton_2_clicked();

private:
    Ui::MainWindow *ui;
    int argc1;
    char **argv1;
    ros::Publisher chatter_publisher;
    ros::Subscriber chatter_subscriber;
    SettingsDialog *m_settings = nullptr;
    QSerialPort *m_serial = nullptr;
    QSerialPort *m_serial_2 = nullptr;
    QLabel *m_status = nullptr;
    QTimer *m_timer = nullptr;
    int m_degree, m_angle;

    virtual void closeEvent(QCloseEvent *event);

    void showStatusMessage(const QString &message);
    QByteArray QString2Hex(QString str);
    char ConvertHexChar(char ch);
    void readMyCom();
};

#endif // MAINWINDOW_H
