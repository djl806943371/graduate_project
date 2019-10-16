#include "command.h"
#include <QDebug>
#include "singleton.h"

command::command()
{
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
        res += waitForResponse(m_serial) + "||";
    }
    return res;
}

QString command::ctlRpm(QSerialPort *m_serial, qint8 device, int rpm)
{
    QByteArray cmd(6, 0x00);
    cmd[0] = device;
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
    return waitForResponse(m_serial);
}

QString command::ctlAngle(QSerialPort *m_serial, qint8 device, int angle)
{
    //    QString temp = cmd.toHex() + Singleton<crc>::GetInstance()->getCrc16(cmd.toHex());
    //    cmd = QByteArray::fromHex(temp.toLocal8Bit());
    QByteArray tmp = QByteArray::number(angle);
    if(tmp.size() == 1)
        tmp.insert(0, "00");
    else if (tmp.size() == 2)
        tmp.insert(0, '0');
    m_serial->write(tmp);
    return waitForResponse(m_serial);
}

//控制加速度
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
        QString tmp = waitForResponse(m_serial);
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

QString command::waitForResponse(QSerialPort *m_serial)
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
