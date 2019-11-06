#include "serialportthread.h"
#include "singleton.h"
#include "calculation.h"
#include "command.h"
#include <QTime>
#include <QVector>

serialportThread::serialportThread(int argc, char **argv) : m_serial(new QSerialPort(this)),
                                                            m_serial_2(new QSerialPort(this)),
                                                            m_degree(0),
                                                            m_angle(0),
                                                            isMoved(false),
                                                            argc1(argc),
                                                            argv1(argv),
                                                            x(0),
                                                            y(0),
                                                            th(0),
                                                            Vx(0),
                                                            Vy(0),
                                                            W(0)

{
    connect(m_serial_2, &QSerialPort::readyRead, this, &serialportThread::receiveArduino);

    ros::init(argc1, argv1, "odometry_publisher");
    if (!ros::master::check())
        qDebug() << 1;
    else
        qDebug() << 0;
    if (!ros::ok())
        qDebug() << 1;
    else
        qDebug() << 0;
    odom_broadcaster = new tf::TransformBroadcaster();
    ros::start();
    ros::NodeHandle n;
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
}

void serialportThread::openSerialPort(Settings setting_1, Settings setting_2)
{
    m_serial->setPortName(setting_1.name);
    m_serial->setBaudRate(setting_1.baudRate);
    m_serial->setDataBits(setting_1.dataBits);
    m_serial->setParity(setting_1.parity);
    m_serial->setStopBits(setting_1.stopBits);
    m_serial->setFlowControl(setting_1.flowControl);
    if (m_serial->open(QIODevice::ReadWrite))
    {
        Singleton<command>::GetInstance()->powerOn(m_serial);
    }
    else
    {
        emit openSerialPortFail(m_serial->errorString());
        return;
    }
    m_serial_2->setPortName(setting_2.name);
    m_serial_2->setBaudRate(setting_2.baudRate);
    m_serial_2->setDataBits(setting_2.dataBits);
    m_serial_2->setParity(setting_2.parity);
    m_serial_2->setStopBits(setting_2.stopBits);
    m_serial_2->setFlowControl(setting_2.flowControl);
    if (m_serial_2->open(QIODevice::ReadWrite))
    {
        m_serial_2->clear();
        emit openSerialPortSuccess();
    }
    else
    {
        emit openSerialPortFail(m_serial_2->errorString());
        return;
    }
    current_time = ros::Time::now();
    last_time = ros::Time::now();
}

void serialportThread::closeSerialPort()
{
    if (m_serial->isOpen())
        m_serial->close();
    if (m_serial_2->isOpen())
        m_serial_2->close();
}
void serialportThread::customButtonMoved(double degree, double angle)
{
    m_degree = degree;
    m_angle = angle;
    if (isMoved == false)
        isMoved = true;
}
void serialportThread::customButtonReleased()
{
    isMoved = false;
}

void serialportThread::periodReadWrite()
{
    if (isMoved)
    {
        rpmVec = Singleton<calculation>::GetInstance()->calVelocity(m_degree, m_angle);
        Singleton<command>::GetInstance()->ctlRpm(m_serial, rpmVec);
        angleVec = Singleton<calculation>::GetInstance()->calAngle(m_degree, m_angle);
        QString cmd = "{";
        cmd += QString::number(angleVec[0]) + ' ' + QString::number(angleVec[1]) + ' ' + QString::number(angleVec[2]) + ' ' + QString::number(angleVec[3]) + "}";
        Singleton<command>::GetInstance()->ctlAngle(m_serial_2, cmd);
    }
    else
    {
        Singleton<command>::GetInstance()->stopMove(m_serial, m_serial_2);
    }
    QVector<double> vels = Singleton<command>::GetInstance()->pollingSpeed(m_serial);


    current_time = ros::Time::now();
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (Vx * cos(th) - Vy * sin(th)) * dt;
    double delta_y = (Vx * sin(th) + Vy * cos(th)) * dt;
    double delta_th = W * dt;
    x += delta_x;
    y += delta_y;
    th += delta_th;
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    //send the transform
    odom_broadcaster->sendTransform(odom_trans);
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = Vx;
    odom.twist.twist.linear.y = Vy;
    odom.twist.twist.angular.z = W;
    //publish the message
    odom_pub.publish(odom);
    last_time = current_time;
}

void serialportThread::receiveArduino()
{
    const QByteArray tmp = m_serial_2->readAll();
    arduinoResponseData.append(tmp);
    if (arduinoResponseData.contains('#'))
    {
        QByteArray res = arduinoResponseData.split('#').at(0);
        arduinoResponseData = arduinoResponseData.right(arduinoResponseData.length() - arduinoResponseData.indexOf('#') - 1);
        emit arduinoReceived(res);
    }
}

void serialportThread::changeAcceleration(int acc)
{
    QTime time = QTime::currentTime().addMSecs(40);
    while (QTime::currentTime() < time)
        ;
    Singleton<command>::GetInstance()->ctlAcc(m_serial, acc);
    emit setAccSuccess();
}
