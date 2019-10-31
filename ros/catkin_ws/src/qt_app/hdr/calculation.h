#ifndef CALCULATION_H
#define CALCULATION_H

#define WIDTH 0.225
#define LENGTH 0.3365
#define RADIUS 0.17

#include <QString>
#include <QVector>

class calculation
{
public:
    calculation();
    QString calAngle(double v, double w);
    QVector<int> calVelocity(double v, double w);
    double rpmToVelocity(int rpm);

};

#endif // CALCULATION_H
