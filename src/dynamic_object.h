#ifndef DYNAMIC_OBJECT_H
#define DYNAMIC_OBJECT_H

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
        : id(id), x(x), y(y), vx(vx), vy(vy), s(s), d(d)
    {
    };
};

#endif // DYNAMIC_OBJECT_H