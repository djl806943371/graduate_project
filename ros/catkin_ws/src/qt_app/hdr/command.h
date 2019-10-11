#ifndef COMMAND_H
#define COMMAND_H

#include <QSerialPort>
#include <QString>
#include "crc.h"

class command
{
public:
    command();

    QString ctlRpm(QSerialPort *m_serial, qint8 device, int rpm);       //控制转速
    QString ctlAcc(QSerialPort *m_serial, qint8 device, int acc);       //控制加速度

};

#endif // COMMAND_H
