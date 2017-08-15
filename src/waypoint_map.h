#ifndef WAYPOINT_MAP_H
#define WAYPOINT_MAP_H

#include <fstream>
#include <iostream>



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

    for (int i = 0; i < maps_x.size(); i++)
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

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(
    double x, double y, double theta, 
    std::vector<double> maps_x, 
    std::vector<double> maps_y)
{
    int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0)
    {
        prev_wp = maps_x.size() - 1;
    }

    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return { frenet_s, frenet_d };
}

// Transform from Frenet s,d coordinates to Cartesian x,y
// TODO: according to slack getXY should not be used - instead splines should be used
std::vector<double> getXY(
    double s, double d, 
    std::vector<double> maps_s, 
    std::vector<double> maps_x, 
    std::vector<double> maps_y)
{
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s*cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s*sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return { x, y };
}

struct sMap
{
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;
};

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

#endif // WAYPOINT_MAP_H
