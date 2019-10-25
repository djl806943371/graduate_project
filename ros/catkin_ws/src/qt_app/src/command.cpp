#include "command.h"
#include <QDebug>
#include "singleton.h"
#include <QTime>

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
        res += waitFor485Response(m_serial) + "||";
    }
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
    for(qint8 i = 0; i <4; i++)
    {
        rpm = rpmVec[i];
        QByteArray cmd(6, 0x00);
        cmd[0] = i;
        cmd[1] = 0x06;
        cmd[2] = 0x00;
        cmd[3] = 0x11;
        bool isNeedReverse = false;
        if(rpm < 0)
        {
            isNeedReverse = true;
            rpm *= (-1);
        }
        QByteArray rpmHex = QByteArray::fromHex(QByteArray::number(rpm, 16));
        if(rpmHex.length() == 1)
            cmd[5] = rpmHex[0];
        else
        {
            cmd[4] = rpmHex[0];
            cmd[5] = rpmHex[1];
        }
        if(isNeedReverse)
        {
            cmd[4] = ~cmd[4];
            cmd[5] = ~cmd[5];
        }
        QString temp = cmd.toHex() + Singleton<crc>::GetInstance()->getCrc16(cmd.toHex());
        cmd = QByteArray::fromHex(temp.toLocal8Bit());
        m_serial->write(cmd);
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
        QString tmp = waitFor485Response(m_serial);
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
    if (m_serial->waitForBytesWritten(10))
    {
        if (m_serial->waitForReadyRead(10))
        {
            QByteArray response_data = m_serial->readAll();
//            while (m_serial->waitForReadyRead(2))
//            {
//                response_data += m_serial->readAll();
//            }
            return response_data.toHex();
    //                emit sigRecvResponse(response_data);
        }
        else
        {
    //                emit sigCatchException("waitForReadyRead timeout");
            return "waitForReadyRead timeout";
        }
    }
    else
    {
    //            emit sigCatchException("waitForBytesWritten timeout");
        return "waitForBytesWritten timeout";
    }
}
