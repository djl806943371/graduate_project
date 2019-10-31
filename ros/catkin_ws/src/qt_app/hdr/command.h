#ifndef COMMAND_H
#define COMMAND_H

#include <QSerialPort>
#include <QString>
#include "crc.h"
#include <QVector>

class command
{
public:
    command();

    QString powerOn(QSerialPort *m_serial);
    QString ctlRpm(QSerialPort *m_serial, QVector<int> rpmVec);       //控制转速
    QString ctlAcc(QSerialPort *m_serial, int acc);       //控制加速度
    QString ctlAngle(QSerialPort *m_serial, QString angles);   //控制角度
    QString stopMove(QSerialPort *m_serial, QSerialPort *m_serial_2);
    QString waitFor485Response(QSerialPort *m_serial);
    QVector<double> pollingSpeed(QSerialPort *m_serial);
    QVector<double> pollingPose(QSerialPort *m_serial);

private:
    QByteArray arduinoResponseData;
    int rpm;
};

#endif // COMMAND_H
