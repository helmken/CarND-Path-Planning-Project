#ifndef WAYPOINT_MAP_H
#define WAYPOINT_MAP_H


#include <vector>

#include "spline.h"


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
    std::vector<sWaypoint> m_waypoints;

    double m_maxS; // maximum s value
    double m_distLastFirstWp; // distance between last and first waypoint
    double m_trackLengthS; // maximum s value + distLastFirstWp

    // interpolate track with splines
    tk::spline m_splineX;
    tk::spline m_splineY;

    // interpolate normal vectors with splines
    tk::spline m_splineDx;
    tk::spline m_splineDy;

    // boundaries of the track for visualization
    double m_minX;
    double m_maxX;
    double m_minY;
    double m_maxY;

public:
    cWaypointMap();

    void ReadMapFile();
    const double GetTrackLength() const;

    sPoint2D CartesianPosition(const double s, const double d) const;
};

#endif // WAYPOINT_MAP_H
