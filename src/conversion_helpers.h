#ifndef CONVERSION_HELPERS_H
#define CONVERSION_HELPERS_H

#include <vector>

// 50 MPH = 22.352 m/s
//        = 80.467 km/h
const double mphAsMs = 0.44704;

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

// convert miles per hour to meters per second
double mphToMs(double mph);

// convert meters per second to miles per hour
double msToMph(double ms);

#endif // CONVERSION_HELPERS_H
