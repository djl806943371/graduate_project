#include "command.h"

command::command()
{

}

void command::ctlRpm(QSerialPort *m_serial, qint8 device, int rpm)
{
    QByteArray cmd;
    cmd[0] = device;
}
