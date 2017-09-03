#include <cmath>
#include <iostream>


#include "conversion_helpers.h"

// For converting back and forth between radians and degrees.
constexpr double pi()
{
    return M_PI;
}

double deg2rad(double x)
{
    return x * pi() / 180;
}

double rad2deg(double x)
{
    return x * 180 / pi();
}

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

/**
* returns waypoint that has the smallest distance
*/
int ClosestWaypoint(
    double x, double y,
    std::vector<double> maps_x, std::vector<double> maps_y)
{
    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (size_t i(0); i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x, y, map_x, map_y);
        if (dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}

/**
* returns waypoint that makes most sense considering the current orientation
*/
//int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
//{
//    int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);
//
//    double map_x = maps_x[closestWaypoint];
//    double map_y = maps_y[closestWaypoint];
//
//    double heading = atan2((map_y - y), (map_x - x));
//
//    double angle = abs(theta - heading);
//
//    //if (angle > pi() / 4)
//    //{
//    //    closestWaypoint++;
//    //}
//
//    // from slack: correct would be
//    if (angle > pi() / 2)
//    {
//        closestWaypoint++;
//    }
//
//    return closestWaypoint;
//}

// version from excodecowboy:
/**
* returns waypoint that makes most sense considering the current orientation
*/
int NextWaypoint(
    double x, double y, double theta,
    std::vector<double> maps_x, std::vector<double> maps_y)
{
    int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y - y), (map_x - x));

    double theta_pos = fmod(theta + (2 * pi()), 2 * pi());
    double heading_pos = fmod(heading + (2 * pi()), 2 * pi());
    double angle = abs(theta_pos - heading_pos);
    if (angle > pi()) {
        angle = (2 * pi()) - angle;
    }

    std::cout << "heading:" << heading << " diff:" << angle << std::endl;

    if (angle > pi() / 2)
    {
        closestWaypoint = (closestWaypoint + 1) % maps_x.size();
    }

    return closestWaypoint;
}
