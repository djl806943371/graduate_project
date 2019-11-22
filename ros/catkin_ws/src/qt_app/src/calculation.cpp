#include "calculation.h"
#include "qmath.h"
#include <QDebug>

calculation::calculation()
{

}

QVector<int> calculation::calAngle(double v, double w)
{
    QVector<int> res;
    double Vx = v + w * WIDTH;
    double Vy = w * LENGTH;
//    qDebug() << "Vx:" << Vx << "|Vy:" << Vy;
    int angle;
    if(Vx == 0.0 && Vy > 0)
        angle = 90;
    else if(Vx == 0.0 && Vy < 0)
        angle = -90;
    else if(Vx == 0.0 && Vy == 0.0)
        angle = 0;
    else
        angle = static_cast<qint8>(atan(Vy / Vx) * 360 / (2 * acos(-1)));
    res.push_back(angle);
    Vx = v - w * WIDTH;
    Vy = w * LENGTH;
    if(Vx == 0.0 && Vy > 0)
        angle = 90;
    else if(Vx == 0.0 && Vy < 0)
        angle = -90;
    else if(Vx == 0.0 && Vy == 0.0)
        angle = 0;
    else
        angle = static_cast<qint8>(atan(Vy / Vx) * 360 / (2 * acos(-1)));
    res.push_back(angle);
    Vx = v - w * WIDTH;
    Vy = - w * LENGTH;
    if(Vx == 0.0 && Vy > 0)
        angle = 90;
    else if(Vx == 0.0 && Vy < 0)
        angle = -90;
    else if(Vx == 0.0 && Vy == 0.0)
        angle = 0;
    else
        angle = static_cast<qint8>(atan(Vy / Vx) * 360 / (2 * acos(-1)));
    res.push_back(angle);
    Vx = v + w * WIDTH;
    Vy = - w * LENGTH;
    if(Vx == 0.0 && Vy > 0)
        angle = 90;
    else if(Vx == 0.0 && Vy < 0)
        angle = -90;
    else if(Vx == 0.0 && Vy == 0.0)
        angle = 0;
    else
        angle = static_cast<qint8>(atan(Vy / Vx) * 360 / (2 * acos(-1)));
    res.push_back(angle);
    return res;
}
QVector<int> calculation::calVelocity(double v, double w)
{
    QVector<int> res;
    double Vx = v + w * WIDTH;
    double Vy = w * LENGTH;
    double V = sqrt(Vx * Vx + Vy * Vy);
//    qDebug() << "ideal speed:" << V;
    if(Vx < 0)
        V *= -1;
    int rpm = static_cast<int>(V / 2 / acos(-1) / RADIUS * 60);
    res.push_back(-rpm);
    Vx = v - w * WIDTH;
    Vy = w * LENGTH;
    V = sqrt(Vx * Vx + Vy * Vy);
    if(Vx < 0)
        V *= -1;
    rpm = static_cast<int>(V / 2 / acos(-1) / RADIUS * 60);
    res.push_back(rpm);
    Vx = v - w * WIDTH;
    Vy = - w * LENGTH;
    V = sqrt(Vx * Vx + Vy * Vy);
    if(Vx < 0)
        V *= -1;
    rpm = static_cast<int>(V / 2 / acos(-1) / RADIUS * 60);
    res.push_back(rpm);
    Vx = v + w * WIDTH;
    Vy = - w * LENGTH;
    V = sqrt(Vx * Vx + Vy * Vy);
    if(Vx < 0)
        V *= -1;
    rpm = static_cast<int>(V / 2 / acos(-1) / RADIUS * 60);
    res.push_back(-rpm);
    return res;
}

double calculation::rpmToVelocity(int rpm)
{
    return 1.0 * rpm * 2 * acos(-1) * RADIUS / 60;
}
