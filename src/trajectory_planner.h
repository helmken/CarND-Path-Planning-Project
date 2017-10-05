#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include "behavior_planner.h"
#include "ego.h"
#include "path.h"
#include "spline.h" // used to smooth out edgy path from waypoints
#include "waypoint_map.h"


// requirement from rubric: lane change should not take longer than 3 s
// requirement from rubric: max 10 m/s^2 total acceleration, max 10 m/s^3 jerk 

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

private:
};

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
s2DCoordFrenet getFrenet(
    double x, double y, double theta,
    std::vector<double> maps_x,
    std::vector<double> maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
// TODO: according to slack getXY should not be used - instead splines should be used
sPoint2D getXY(
    const double s, const double d,
    const cWaypointMap& waypointMap);

sPoint2D FrenetToCartesian(
    const sWaypoint& wp0, const sWaypoint& wp1,
    const double s, const double d);

/*
Generate path for given behavior
*/
sPath GeneratePath(
    const sBehavior& plannedBehavior,
    const sEgo& ego,
    const cWaypointMap& waypointMap,
    const sPath& previousPath);

void SamplePathSpline(
    tk::spline& pathSpline,
    const double plannedPathLength,
    const double speed0,
    const double speedTarget,
    const double deltaT,
    std::vector<sPoint2D>& generatedPathPointsLocal);

double SpeedAtEndOfPath(const std::vector<sPoint2D>& path);

std::vector<sPoint2D> TransformToLocalCoordinates(
    const sPoint2D& referencePoint,
    const double referenceYaw,
    const std::vector<sPoint2D>& points);

std::vector<sPoint2D> TransformToWorldCoordinates(
    const sPoint2D& referencePoint,
    const double referenceYaw,
    const std::vector<sPoint2D>& points);

void SetSplinePoints(tk::spline& pathSpline, const std::vector<sPoint2D>& pathPoints);

void PrintReferencePoints(
    const std::string& info,
    const std::vector<sPoint2D>& pathPoints);

#endif // TRAJECTORY_PLANNER_H