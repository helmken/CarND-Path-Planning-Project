#ifndef WAYPOINT_MAP_H
#define WAYPOINT_MAP_H

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>


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
