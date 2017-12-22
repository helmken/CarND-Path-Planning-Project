#include <cmath>
#include <iostream>

#include "constants.h"
#include "conversion_helpers.h"


double deg2rad(double x)
{
    return x * M_PI / 180;
}

double Distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

double Distance(const sPoint2D& pt0, const sPoint2D& pt1)
{
    return Distance(pt0.x, pt0.y, pt1.x, pt1.y);
}

double mphToMs(double mph)
{
    return mph * mphAsMs;
}
