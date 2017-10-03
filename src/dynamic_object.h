#ifndef DYNAMIC_OBJECT_H
#define DYNAMIC_OBJECT_H


#include "conversion_helpers.h"


struct sDynamicObject
{
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;

    sDynamicObject()
    {};

    sDynamicObject(
        int id,
        double x,
        double y,
        double vx,
        double vy,
        double s,
        double d)
        : id(id), x(x), y(y), 
        vx(mphToMs(vx)), vy(mphToMs(vy)),
        s(s), d(d)
    {
    };

    double GetSpeed() const
    {
        double speed = sqrt(pow(vx, 2) + pow(vy, 2));
        return speed;
    }
};

#endif // DYNAMIC_OBJECT_H