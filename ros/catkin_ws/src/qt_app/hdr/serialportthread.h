#ifndef SERIALPORTTHREAD_H
#define SERIALPORTTHREAD_H

#include <QObject>
#include <QDebug>
#include <QThread>
#include <QSerialPort>
#include <settingparameters.h>
#include <command.h>
#include <QByteArray>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class serialportThread : public QObject
{
    Q_OBJECT
public:
    serialportThread(int argc, char **argv);
public slots:
    void openSerialPort(Settings setting_1, Settings setting_2);
    void closeSerialPort();
    void customButtonMoved(double degree, double angle);
    void customButtonReleased();
    void periodReadWrite();
    void receiveArduino();
    void changeAcceleration(int acc);
    void clearFault();

signals:
    void openSerialPortSuccess();
    void openSerialPortFail(QString str);
    void arduinoReceived(QByteArray content);
    void setAccSuccess();
    void clearFaultSuccess();

private:
    QSerialPort *m_serial = nullptr;
    QSerialPort *m_serial_2 = nullptr;
    Settings p1, p2;
    double m_degree;
    double m_angle;
    bool isMoved;
    QVector<int> rpmVec;
    QVector<int> angleVec;
    QByteArray arduinoResponseData;
    int argc1;
    char **argv1;
    double x;
    double y;
    double th;
    double Vx;
    double Vy;
    double W;
    ros::Time current_time, last_time;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster *odom_broadcaster = nullptr;
};

#endif // SERIALPORTTHREAD_H
