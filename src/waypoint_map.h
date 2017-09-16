#ifndef WAYPOINT_MAP_H
#define WAYPOINT_MAP_H

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>


struct sWaypoint
{
    double x;
    double y;
    double s;
    double dx;
    double dy;

    sWaypoint()
        : x(0), y(0), s(0), dx(0), dy(0)
    {}

    sWaypoint(
        double x, double y,
        double s, double dx, double dy)
        : x(x), y(y), s(s), dx(dx), dy(dy)
    {}
};

class cWaypointMap
{
private:

    // Load up map values for waypoint's x, y, s and d normalized normal vectors

    std::vector<sWaypoint> m_waypoints;

    // TODO: these have to be removed after refactoring GetXY:
    std::vector<double> m_waypointsS;
    std::vector<double> m_waypointsX;
    std::vector<double> m_waypointsY;

    double m_minX;
    double m_maxX;
    double m_minY;
    double m_maxY;

public:
    void ReadMapFile();
    void GetMapBoundaries(
        double& left, double& right,
        double& bottom, double& top) const;
    const std::vector<sWaypoint>& GetWaypoints() const;

    // TODO: these have to be removed after refactoring GetXY:
    const std::vector<double>& GetMapPointsS() const;
    const std::vector<double>& GetMapPointsX() const;
    const std::vector<double>& GetMapPointsY() const;
};

#endif // WAYPOINT_MAP_H
