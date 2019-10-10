#ifndef COMMAND_H
#define COMMAND_H

#include <QSerialPort>

class command
{
public:
    command();

private:
    void ctlRpm(QSerialPort *m_serial, int device, int rpm);
};

#endif // COMMAND_H
