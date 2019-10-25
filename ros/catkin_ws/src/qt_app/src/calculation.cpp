#include "calculation.h"
#include "qmath.h"
#include <QDebug>

calculation::calculation()
{

}

QString calculation::calAngle(double v, double w)
{
    double Vx = v + w * WIDTH;
    double Vy = w * LENGTH;
    qDebug() << "Vx:" << Vx << "|Vy:" << Vy;
    int angle1, angle2, angle3, angle4;
    if(Vx == 0.0 && Vy > 0)
        angle1 = 90;
    else if(Vx == 0.0 && Vy < 0)
        angle1 = -90;
    else if(Vx == 0.0 && Vy == 0.0)
        angle1 = 0;
    else
        angle1 = static_cast<qint8>(atan(Vy / Vx) * 360 / (2 * acos(-1)));
    Vx = v - w * WIDTH;
    Vy = w * LENGTH;
    if(Vx == 0.0 && Vy > 0)
        angle2 = 90;
    else if(Vx == 0.0 && Vy < 0)
        angle2 = -90;
    else if(Vx == 0.0 && Vy == 0.0)
        angle2 = 0;
    else
        angle2 = static_cast<qint8>(atan(Vy / Vx) * 360 / (2 * acos(-1)));
    Vx = v - w * WIDTH;
    Vy = - w * LENGTH;
    if(Vx == 0.0 && Vy > 0)
        angle3 = 90;
    else if(Vx == 0.0 && Vy < 0)
        angle3 = -90;
    else if(Vx == 0.0 && Vy == 0.0)
        angle3 = 0;
    else
        angle3 = static_cast<qint8>(atan(Vy / Vx) * 360 / (2 * acos(-1)));
    Vx = v + w * WIDTH;
    Vy = - w * LENGTH;
    if(Vx == 0.0 && Vy > 0)
        angle4 = 90;
    else if(Vx == 0.0 && Vy < 0)
        angle4 = -90;
    else if(Vx == 0.0 && Vy == 0.0)
        angle4 = 0;
    else
        angle4 = static_cast<qint8>(atan(Vy / Vx) * 360 / (2 * acos(-1)));
    QString res = "{";
    res += QString::number(angle1) + ' ' + QString::number(angle2) + ' ' + QString::number(angle3) + ' ' + QString::number(angle4) + "}";
    return res;
}
QVector<int> calculation::calVelocity(double v, double w)
{
    QVector<int> res;
    double Vx = v + w * WIDTH;
    double Vy = w * LENGTH;
    double V = sqrt(Vx * Vx + Vy * Vy);
    int rpm = static_cast<int>(V / 2 / acos(-1) / RADIUS * 60);
    res.push_back(rpm);
    Vx = v - w * WIDTH;
    Vy = w * LENGTH;
    V = sqrt(Vx * Vx + Vy * Vy);
    rpm = static_cast<int>(V / 2 / acos(-1) / RADIUS * 60);
    res.push_back(-rpm);
    Vx = v - w * WIDTH;
    Vy = - w * LENGTH;
    V = sqrt(Vx * Vx + Vy * Vy);
    rpm = static_cast<int>(V / 2 / acos(-1) / RADIUS * 60);
    res.push_back(rpm);
    Vx = v + w * WIDTH;
    Vy = - w * LENGTH;
    V = sqrt(Vx * Vx + Vy * Vy);
    rpm = static_cast<int>(V / 2 / acos(-1) / RADIUS * 60);
    res.push_back(rpm);
    return res;
}
