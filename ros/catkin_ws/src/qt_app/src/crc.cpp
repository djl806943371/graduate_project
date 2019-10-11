#include "crc.h"

QString crc::getCrc16(const QString &hexText)
{
    QByteArray data  = QByteArray::fromHex(hexText.toLocal8Bit());

    quint8 buf;
    quint16 crc16 = 0xFFFF;

    for ( auto i = 0; i < data.size(); ++i )
    {
        buf = static_cast<quint8>(data.at( i ) ^ crc16);
        crc16 >>= 8;
        crc16 ^= crc16Table[ buf ];
    }
    crc16 = static_cast<quint16>((crc16 >> 8) + (crc16 << 8));
    return  QString("%1").arg(crc16 , 4, 16, QLatin1Char('0'));   //拼凑成4个16进制字符，空位补0
}
