#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include "path.h"
#include "ego.h"
#include "waypoint_map.h"


struct s2DCoordCart
{
    double x;
    double y;

    s2DCoordCart(double x, double y)
        : x(x), y(y)
    {};
};

struct s2DCoordFrenet
{
    double s;
    double d;

    s2DCoordFrenet(double s, double d)
        : s(s), d(d)
    {};
};


/**
 * Generate trajectory considering
 * - target speed 
 * - avoiding collisions
 * - smooth: avoid longitudinal/lateral jerks
 * - safe: maintain safety distance
 */
class cTrajectoryPlanner
{
public:
    cTrajectoryPlanner();
    void Init(); // TODO: input
    void Execute(); // TODO: input

private:
};

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
s2DCoordFrenet getFrenet(
    double x, double y, double theta,
    std::vector<double> maps_x,
    std::vector<double> maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
// TODO: according to slack getXY should not be used - instead splines should be used
s2DCoordCart getXY(
    const double s, const double d,
    const std::vector<double>& maps_s,
    const std::vector<double>& maps_x,
    const std::vector<double>& maps_y);

s2DCoordCart FrenetToCartesian(
    const sWaypoint& wp0, const sWaypoint& wp1,
    const double s, const double d);

sPath GeneratePath(
    const sEgo& ego, 
    const cWaypointMap& waypointMap,
    const sPath& previousPath, 
    const int lane,
    const double referenceVelocity);


#endif // TRAJECTORY_PLANNER_H