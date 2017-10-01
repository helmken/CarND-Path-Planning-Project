#ifndef CONVERSION_HELPERS_H
#define CONVERSION_HELPERS_H

#include <vector>


double deg2rad(double x);

double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

/**
* returns waypoint that has the smallest distance
*/
int FindClosestWaypointIdx(
    const double x, 
    const double y,
    const std::vector<double>& maps_x, 
    const std::vector<double>& maps_y);

/**
* returns waypoint that makes most sense considering the current orientation
*/
int FindNextWaypointIdx(
    const double x, 
    const double y, 
    const double theta,
    const std::vector<double>& maps_x, 
    const std::vector<double>& maps_y);

#endif // CONVERSION_HELPERS_H
