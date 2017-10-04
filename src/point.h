#ifndef POINT_H
#define POINT_H


struct sPoint2D
{
    double x;
    double y;

    sPoint2D(double x, double y)
    : x(x), y(y)
    {};
};

struct s2DCoordFrenet
{
    double s;
    double d;

    s2DCoordFrenet(double s, double d)
        : s(s), d(d)
    {};
};

#endif // POINT_H