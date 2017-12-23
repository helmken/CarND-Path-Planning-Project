#include <fstream>
#include <iostream>
#include <cmath>
#include <string>
#include <sstream>

#include "constants.h"
#include "conversion_helpers.h"
#include "waypoint_map.h"


cWaypointMap::cWaypointMap()
    : 
    m_maxS(0.0),
    m_distLastFirstWp(0.0),
    m_trackLengthS(0.0),
    m_minX(doubleMax),
    m_maxX(doubleMin),
    m_minY(doubleMax),
    m_maxY(doubleMin)
{}

void cWaypointMap::ReadMapFile()
{
    // Waypoint map to read from
    auto mapFilename = "../data/highway_map.csv";

    std::ifstream inFileStream(mapFilename, std::ifstream::in);

    m_minX = doubleMax;
    m_maxX = doubleMin;

    m_minY = doubleMax;
    m_maxY = doubleMin;

    // map values for waypoint's x, y, s and d normalized normal vectors
    std::vector<double> waypointsS;
    std::vector<double> waypointsX;
    std::vector<double> waypointsY;

    std::vector<double> waypointsDx; // x-component of normal vector
    std::vector<double> waypointsDy; // y-component of normal vector

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

        waypointsS.push_back(s);
        waypointsX.push_back(x);
        waypointsY.push_back(y);

        waypointsDx.push_back(dx);
        waypointsDy.push_back(dy);

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

    if (!waypointsS.empty())
    {
        m_maxS = waypointsS.back();
        m_distLastFirstWp = Distance(
            waypointsX.back(), waypointsY.back(),
            waypointsX.front(), waypointsY.front());
        m_trackLengthS = m_maxS + m_distLastFirstWp;


        // copy points from beginning of track for smooth transition from
        // end of track to start of track
        waypointsS.push_back(m_trackLengthS);
        waypointsX.push_back(waypointsX[0]);
        waypointsY.push_back(waypointsY[0]);
        waypointsDx.push_back(waypointsDx[0]);
        waypointsDy.push_back(waypointsDy[0]);

        m_splineX.set_points(waypointsS, waypointsX); // interpolate x over s
        m_splineY.set_points(waypointsS, waypointsY); // interpolate y over s

        m_splineDx.set_points(waypointsS, waypointsDx); // interpolate normal x over s
        m_splineDy.set_points(waypointsS, waypointsDy); // interpolate normal y over s

        // check that interpolation works as expected
		//const auto pt0 = CartesianPosition(0.0, 0.0);
		//printf("%s: at s=0: (%.3f,%.3f), CartesianPos=(%.3f,%.3f)\n",
		//	__FUNCTION__, waypointsX[0], waypointsY[0], pt0.x, pt0.y);
		//const auto ptTrackLength = CartesianPosition(m_trackLengthS, 0.0);
		//printf("%s: at s=%.3f: (%.3f,%.3f), CartesianPos=(%.3f,%.3f)\n",
		//	__FUNCTION__, m_trackLengthS,
		//	waypointsX[waypointsX.size() - 1],
		//	waypointsY[waypointsX.size() - 1],
		//	ptTrackLength.x, ptTrackLength.y);
		//printf("%s: diff=%.3f\n", __FUNCTION__, Distance(pt0, ptTrackLength));
    }
}

sPoint2D cWaypointMap::CartesianPosition(const double s, const double d) const
{
    const double sNormalized = std::fmod(s, m_trackLengthS);
    //if (fabs(sNormalized - s) > 0.1)
    //{
    //    printf("sNormalized=%.3f, s=%.3f\n", sNormalized, s);
    //}

    const double x = m_splineX(sNormalized) + m_splineDx(sNormalized) * d;
    const double y = m_splineY(sNormalized) + m_splineDy(sNormalized) * d;
    
    return sPoint2D(x, y);
}

const double cWaypointMap::GetTrackLength() const
{
    return m_trackLengthS;
}
