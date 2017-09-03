#ifndef CONVERSION_HELPERS_H
#define CONVERSION_HELPERS_H

#include <vector>

// For converting back and forth between radians and degrees.
constexpr double pi();

double deg2rad(double x);

double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

/**
* returns waypoint that has the smallest distance
*/
int ClosestWaypoint(
    double x, double y,
    std::vector<double> maps_x, std::vector<double> maps_y);

/**
* returns waypoint that makes most sense considering the current orientation
*/
int NextWaypoint(
    double x, double y, double theta,
    std::vector<double> maps_x, std::vector<double> maps_y);


#endif // CONVERSION_HELPERS_H
