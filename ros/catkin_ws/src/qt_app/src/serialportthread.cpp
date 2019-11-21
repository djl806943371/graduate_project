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
                                                            W(0),
                                                            cnt(0)

{
    connect(m_serial_2, &QSerialPort::readyRead, this, &serialportThread::receiveArduino);

//    ros::init(argc1, argv1, "odometry_publisher");
//    odom_broadcaster = new tf::TransformBroadcaster();
//    ros::start();
//    ros::NodeHandle n;
//    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
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
        m_serial->clear();
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
//    current_time = ros::Time::now();
//    last_time = ros::Time::now();
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
        if(m_angle>0.5)
            qDebug() << m_angle;
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
    cnt++;
    if(cnt % 5 == 0){
        cnt = 0;
        Singleton<command>::GetInstance()->clearFault(m_serial);
    }
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
    while (QTime::currentTime() < time);
    Singleton<command>::GetInstance()->ctlAcc(m_serial, acc);
    emit setAccSuccess();
}

void serialportThread::clearFault(){
    Singleton<command>::GetInstance()->clearFault(m_serial);
    emit clearFaultSuccess();
}
