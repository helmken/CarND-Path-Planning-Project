#include <cmath>
#include <iostream>


#include "conversion_helpers.h"


double deg2rad(double x)
{
    return x * M_PI / 180;
}

double rad2deg(double x)
{
    return x * 180 / M_PI;
}

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

/**
* returns waypoint that has the smallest distance
*/
int FindClosestWaypointIdx(
    const double x, 
    const double y,
    const std::vector<double>& maps_x, 
    const std::vector<double>& maps_y)
{
    double closestLen = 100000; //large number
    int closestWaypointIdx = 0;

    for (size_t idx(0); idx < maps_x.size(); idx++)
    {
        const double map_x = maps_x[idx];
        const double map_y = maps_y[idx];
        const double dist = distance(x, y, map_x, map_y);
        if (dist < closestLen)
        {
            closestLen = dist;
            closestWaypointIdx = idx;
        }
    }

    return closestWaypointIdx;
}

/**
* returns waypoint that makes most sense considering the current orientation
*/
//int FindNextWaypointIdx(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
//{
//    int closestWaypointIdx = FindClosestWaypointIdx(x, y, maps_x, maps_y);
//
//    double map_x = maps_x[closestWaypointIdx];
//    double map_y = maps_y[closestWaypointIdx];
//
//    double heading = atan2((map_y - y), (map_x - x));
//
//    double angle = abs(theta - heading);
//
//    //if (angle > M_PI / 4)
//    //{
//    //    closestWaypointIdx++;
//    //}
//
//    // from slack: correct would be
//    if (angle > M_PI / 2)
//    {
//        closestWaypointIdx++;
//    }
//
//    return closestWaypointIdx;
//}

// version from excodecowboy:
/**
* returns waypoint that makes most sense considering the current orientation
*/
int FindNextWaypointIdx(
    const double x, 
    const double y, 
    const double theta,
    const std::vector<double>& maps_x, 
    const std::vector<double>& maps_y)
{
    int closestWaypointIdx = FindClosestWaypointIdx(x, y, maps_x, maps_y);

    const double map_x = maps_x[closestWaypointIdx];
    const double map_y = maps_y[closestWaypointIdx];

    const double heading = atan2((map_y - y), (map_x - x));

    const double theta_pos = fmod(theta + (2 * M_PI), 2 * M_PI);
    const double heading_pos = fmod(heading + (2 * M_PI), 2 * M_PI);
    double angle = abs(theta_pos - heading_pos);
    if (angle > M_PI) 
    {
        angle = (2 * M_PI) - angle;
    }

    std::cout << "heading:" << heading << " diff:" << angle << std::endl;

    if (angle > M_PI / 2)
    {
        closestWaypointIdx = (closestWaypointIdx + 1) % maps_x.size();
    }

    return closestWaypointIdx;
}
