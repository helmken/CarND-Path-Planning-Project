#ifndef CONVERSION_HELPERS_H
#define CONVERSION_HELPERS_H


#include <vector>
#include "point.h"


double deg2rad(double x);

double Distance(double x1, double y1, double x2, double y2);

double Distance(const sPoint2D& pt0, const sPoint2D& pt1);

// convert miles per hour to meters per second
double mphToMs(double mph);

#endif // CONVERSION_HELPERS_H
