#ifndef SETTINGPARAMETERS_H
#define SETTINGPARAMETERS_H

#include <QString>
#include <QSerialPort>

struct Settings
{
    QString name;
    qint32 baudRate;
    QString stringBaudRate;
    QSerialPort::DataBits dataBits;
    QString stringDataBits;
    QSerialPort::Parity parity;
    QString stringParity;
    QSerialPort::StopBits stopBits;
    QString stringStopBits;
    QSerialPort::FlowControl flowControl;
    QString stringFlowControl;
};

#endif // SETTINGPARAMETERS_H
