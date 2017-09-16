#include "waypoint_map.h"

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>


using namespace std;

void cWaypointMap::ReadMapFile()
{
    // Waypoint map to read from
    std::string mapFilename = "../data/highway_map.csv";

    // The max s value before wrapping around the track back to 0
    // double max_s = 6945.554; // TODO: unused?

    std::ifstream inFileStream(mapFilename.c_str(), std::ifstream::in);

    m_minX = std::numeric_limits<double>::max();
    m_maxX = std::numeric_limits<double>::min();

    m_minY = std::numeric_limits<double>::max();
    m_maxY = std::numeric_limits<double>::min();

    // d_x and d_y are normal components of a waypoint
    std::string line;
    while (getline(inFileStream, line))
    {
        std::istringstream iss(line);
        double x;
        double y;
        double s;
        double dx;
        double dy;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> dx;
        iss >> dy;

        m_waypoints.push_back(sWaypoint(x, y, s, dx, dy));

        // TODO: remove these:
        m_waypointsS.push_back(s);
        m_waypointsX.push_back(x);
        m_waypointsY.push_back(y);


        if (x < m_minX)
        {
            m_minX = x;
        }
        if (x > m_maxX)
        {
            m_maxX = x;
        }

        if (y < m_minY)
        {
            m_minY = y;
        }
        if (y > m_maxY)
        {
            m_maxY = y;
        }
    }
}

void cWaypointMap::GetMapBoundaries(
    double& left, double& right,
    double& bottom, double& top) const
{
    left = m_minX;
    right = m_maxX;
    bottom = m_minY;
    top = m_maxY;
}

const double cWaypointMap::GetMaxS() const
{
    // assuming that the last waypoint contains the maximum s value
    return m_waypoints[m_waypoints.size() - 1].s;
}

const std::vector<sWaypoint>& cWaypointMap::GetWaypoints() const
{
    return m_waypoints;
}

const std::vector<double>& cWaypointMap::GetMapPointsS() const
{
    return m_waypointsS;
}

const std::vector<double>& cWaypointMap::GetMapPointsX() const
{
    return m_waypointsX;
}

const std::vector<double>& cWaypointMap::GetMapPointsY() const
{
    return m_waypointsY;
}
