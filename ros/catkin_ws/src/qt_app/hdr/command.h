#ifndef COMMAND_H
#define COMMAND_H

#include <QSerialPort>
#include <QString>
#include "crc.h"

class command
{
public:
    command();

    QString powerOn(QSerialPort *m_serial);
    QString ctlRpm(QSerialPort *m_serial, qint8 device, int rpm);       //控制转速
    QString ctlAcc(QSerialPort *m_serial, int acc);       //控制加速度
    QString ctlAngle(QSerialPort *m_serial, qint8 device, int angle);   //控制角度
    QString waitForResponse(QSerialPort *m_serial);
};

#endif // COMMAND_H
