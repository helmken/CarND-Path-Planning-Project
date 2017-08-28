#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include "path.h"
#include "ego.h"
#include "waypoint_map.h"


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

sPath GeneratePath(
    const sEgo& ego, 
    const sMap& waypointMap, 
    const sPath& previousPath, 
    const int lane,
    const double referenceVelocity);


#endif // TRAJECTORY_PLANNER_H