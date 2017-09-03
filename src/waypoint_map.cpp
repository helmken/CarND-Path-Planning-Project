#include "waypoint_map.h"

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>


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
