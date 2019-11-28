#include "command.h"
#include <QDebug>
#include "singleton.h"
#include <QTime>
#include <unistd.h>
#include <calculation.h>

command::command()
{
    arduinoResponseData = "";
}

QString command::powerOn(QSerialPort *m_serial)
{
    QString res = "";
    for(qint8 device = 1; device < 5; ++device)
    {
        QByteArray cmd(6, 0x00);
        cmd[0] = device;
        cmd[1] = 0x06;
        cmd[2] = 0x00;
        cmd[3] = 0x10;
        cmd[4] = 0x00;
        cmd[5] = 0x1F;
        QString temp = cmd.toHex() + Singleton<crc>::GetInstance()->getCrc16(cmd.toHex());
        cmd = (QByteArray::fromHex(temp.toLocal8Bit()));
        m_serial->write(cmd);
        m_serial->waitForBytesWritten(10);
        res += waitFor485Response(m_serial) + "||";
    }
    return res;
}

QString command::clearFault(QSerialPort *m_serial)
{
    QString res = "";
    for(qint8 device = 1; device < 5; ++device)
    {
        QByteArray cmd(6, 0x00);
        cmd[0] = device;
        cmd[1] = 0x06;
        cmd[2] = 0x00;
        cmd[3] = 0x15;
        cmd[4] = 0x00;
        cmd[5] = 0x7F;
        QString temp = cmd.toHex() + Singleton<crc>::GetInstance()->getCrc16(cmd.toHex());
        cmd = (QByteArray::fromHex(temp.toLocal8Bit()));
        m_serial->write(cmd);
        m_serial->waitForBytesWritten(10);
        waitFor485Response(m_serial);
    }
    powerOn(m_serial);
    return res;
}

QString command::stopMove(QSerialPort *m_serial, QSerialPort *m_serial_2)
{
    QVector<int> rpmVec(4, 0);
    ctlRpm(m_serial, rpmVec);
    QString angles = "{0 0 0 0}";
    ctlAngle(m_serial_2, angles);
    return "";
}

QString command::ctlRpm(QSerialPort *m_serial, QVector<int> rpmVec)
{
    for(qint8 i = 0; i < 4; i++)
    {
        rpm = rpmVec[i];
        QByteArray cmd(6, 0x00);
        cmd[0] = i + 1;
        cmd[1] = 0x06;
        cmd[2] = 0x00;
        cmd[3] = 0x11;
        bool isNeedReverse = false;
        if(rpm < 0)
        {
            isNeedReverse = true;
            rpm *= (-1);
        }
        QByteArray rpmHex = QByteArray::fromHex(QByteArray::number(rpm * 8192 / 3000, 16));
        if(rpmHex.length() == 1)
            cmd[5] = rpmHex[0];
        else
        {
            cmd[4] = rpmHex[0];
            cmd[5] = rpmHex[1];
        }
        if(isNeedReverse)
        {
            cmd[4] = (~cmd[4]) ;
            cmd[5] = (~cmd[5]) ;
        }
        QString temp = cmd.toHex() + Singleton<crc>::GetInstance()->getCrc16(cmd.toHex());
        cmd = QByteArray::fromHex(temp.toLocal8Bit());
        m_serial->write(cmd);
        m_serial->waitForBytesWritten(10);
        waitFor485Response(m_serial);
    }
    return "Success";
}

QString command::ctlAngle(QSerialPort *m_serial, QString angles)
{
    QByteArray tmp = angles.toLatin1();
    m_serial->write(tmp);
    m_serial->waitForBytesWritten(20);
    return "";
}

//
QString command::ctlAcc(QSerialPort *m_serial, int acc)
{
    QString res = "Driver:";
    for(qint8 device = 1; device < 5; ++device)
    {
        QByteArray cmd(6, 0x00);
        cmd[0] = device;
        cmd[1] = 0x06;
        cmd[2] = 0x00;
        cmd[3] = 0x13;
        QByteArray rpmHex = QByteArray::fromHex(QByteArray::number(acc, 16));
        cmd[4] = rpmHex[0];
        cmd[5] = rpmHex[0];
        QString temp = cmd.toHex() + Singleton<crc>::GetInstance()->getCrc16(cmd.toHex());
        cmd = (QByteArray::fromHex(temp.toLocal8Bit()));
        m_serial->write(cmd);
        m_serial->waitForBytesWritten(10);
        QString tmp = waitFor485Response(m_serial);
        qDebug() << "set acc:" << tmp;
        if(tmp == "waitForReadyRead timeout" || tmp == "waitForBytesWritten timeout")
        {
            res += QString("%1 ").arg(device);
        }
    }
    if(res != "Driver:")
        res += "set acceleration FAIL";
    else
        res = "set acceleration SUCCESS";
    return res;
}

QString command::waitFor485Response(QSerialPort *m_serial)
{
    if (m_serial->waitForReadyRead(10))
    {
        QByteArray response_data = m_serial->readAll();
        return response_data.toHex();
    }
    else
        return "waitForReadyRead timeout";
}

QVector<double> command::pollingSpeed(QSerialPort *m_serial){
    QVector<double> vels(4, 2000.0);
    for(qint8 device = 1; device <5; device++)
    {
        QByteArray cmd(6, 0x00);
        cmd[0] = device;
        cmd[1] = 0x03;
        cmd[2] = 0x00;
        cmd[3] = 0xF4;
        cmd[4] = 0x00;
        cmd[5] = 0x01;
        QString temp = cmd.toHex() + Singleton<crc>::GetInstance()->getCrc16(cmd.toHex());
        cmd = QByteArray::fromHex(temp.toLocal8Bit());
        m_serial->write(cmd);
        m_serial->waitForBytesWritten();
        temp = waitFor485Response(m_serial);
        temp = temp.mid(6, 4);
        if(temp != "rRea")
        {
            int rpm = temp.toInt(nullptr, 16);
            if(rpm > (1 << 15))
            {
                rpm = rpm - 65535 - 1;
            }
            rpm = rpm * 3000 / 8192;
            double vel = Singleton<calculation>::GetInstance()->rpmToVelocity(rpm);
            vels[device - 1] = vel;
        }
        if(device != 4)
        {
            QTime time = QTime::currentTime().addMSecs(40);
            while(QTime::currentTime() < time);
        }
    }
    vels[0] *= -1;
    vels[3] *= -1;
    return vels;
}

void command::pollingStatus(QSerialPort *m_serial){
    for(qint8 device = 0; device <4; device++)
    {
        QByteArray cmd(6, 0x00);
        cmd[0] = device;
        cmd[1] = 0x03;
        cmd[2] = 0x00;
        cmd[3] = 0xF1;
        cmd[4] = 0x00;
        cmd[5] = 0x01;
        QString temp = cmd.toHex() + Singleton<crc>::GetInstance()->getCrc16(cmd.toHex());
        cmd = QByteArray::fromHex(temp.toLocal8Bit());
        m_serial->write(cmd);
        m_serial->waitForBytesWritten();
        temp = waitFor485Response(m_serial);
        temp = temp.mid(6, 4);
        if(device != 4)
        {
            QTime time = QTime::currentTime().addMSecs(40);
            while(QTime::currentTime() < time);
        }
    }
}
