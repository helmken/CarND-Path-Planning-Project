#ifndef VEHICLE_H
#define VEHICLE_H

#include <math.h>

#include "constants.h"


struct sVehicle
{
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
    double v;
    double sDistanceEgo;

    sVehicle()
        : id(invalidVehicleId)
        , x(0.0), y(0.0)
        , vx(0.0), vy(0.0)
        , s(0.0), d(0.0)
        , v(0.0)
        , sDistanceEgo(0.0)
    {};

    sVehicle(
        int id,
        double x,
        double y,
        double vx,
        double vy,
        double s,
        double d)
        : id(id), x(x), y(y), 
        vx(vx), vy(vy),
        s(s), d(d),
        sDistanceEgo(0.0)
    {
        v = sqrt(pow(vx, 2) + pow(vy, 2));
    };

    sVehicle& operator=(const sVehicle& other)
    {
        id = other.id;
        x = other.x;
        y = other.y;
        vx = other.vx;
        vy = other.vy;
        s = other.s;
        d = other.d;
        v = other.v;
        sDistanceEgo = other.sDistanceEgo;
        return *this;
    }
};

#endif // VEHICLE_H
