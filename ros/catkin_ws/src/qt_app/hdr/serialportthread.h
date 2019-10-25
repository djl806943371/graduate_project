#ifndef SERIALPORTTHREAD_H
#define SERIALPORTTHREAD_H

#include <QObject>
#include <QDebug>
#include <QThread>
#include <QSerialPort>
#include <settingparameters.h>
#include <command.h>
#include <QByteArray>

class serialportThread : public QObject
{
    Q_OBJECT
public:
    serialportThread();
public slots:
    void openSerialPort(Settings setting_1, Settings setting_2);
    void closeSerialPort();
    void customButtonMoved(double degree, double angle);
    void customButtonReleased();
    void periodReadWrite();
    void receiveArduino();
    void changeAcceleration(int acc);

signals:
    void openSerialPortSuccess();
    void openSerialPortFail(QString str);
    void arduinoReceived(QByteArray content);

private:
    QSerialPort *m_serial = nullptr;
    QSerialPort *m_serial_2 = nullptr;
    Settings p1, p2;
    double m_degree;
    double m_angle;
    bool isMoved;
    QVector<int> rpmVec;
    QByteArray arduinoResponseData;

};

#endif // SERIALPORTTHREAD_H
