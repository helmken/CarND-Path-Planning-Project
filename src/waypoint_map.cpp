#include "waypoint_map.h"

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

#include "conversion_helpers.h"

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

sMap ReadMapFile()
{
    // Waypoint map to read from
    std::string map_file_ = "../data/highway_map.csv";

    // The max s value before wrapping around the track back to 0
    // double max_s = 6945.554; // TODO: unused?

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    sMap waypointMap;

    // d_x and d_y are normal components of a waypoint
    std::string line;
    while (getline(in_map_, line))
    {
        std::istringstream iss(line);
        double x;
        double y;
        double s;
        double d_x;
        double d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        waypointMap.map_waypoints_x.push_back(x);
        waypointMap.map_waypoints_y.push_back(y);
        waypointMap.map_waypoints_s.push_back(s);
        waypointMap.map_waypoints_dx.push_back(d_x);
        waypointMap.map_waypoints_dy.push_back(d_y);
    }

    return waypointMap;
}
