#ifndef WAYPOINT_MAP_H
#define WAYPOINT_MAP_H

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>


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

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(
    double x, double y, double theta,
    std::vector<double> maps_x,
    std::vector<double> maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
// TODO: according to slack getXY should not be used - instead splines should be used
std::vector<double> getXY(
    double s, double d,
    std::vector<double> maps_s,
    std::vector<double> maps_x,
    std::vector<double> maps_y);

struct sMap
{
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;
};

sMap ReadMapFile();

#endif // WAYPOINT_MAP_H
