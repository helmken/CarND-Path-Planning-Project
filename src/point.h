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

struct sFrenetPt
{
    double s;
    double d;

    sFrenetPt(double s, double d)
        : s(s), d(d)
    {};

    void operator += (const sFrenetPt& other)
    {
        s += other.s;
        d += other.d;
    };
};

#endif // POINT_H