#include "command.h"
#include <QDebug>
#include "singleton.h"

command::command()
{
}

//控制转速
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
    cmd = (QByteArray::fromHex(temp.toLocal8Bit()));
    m_serial->write(cmd);
    return temp;
}

//控制加速度
QString command::ctlAcc(QSerialPort *m_serial, qint8 device, int acc)
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
    return temp;
}
