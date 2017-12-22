#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include "behavior.h"
#include "ego.h"
#include "path.h"
#include "spline.h"
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

    void Init(const cWaypointMap* waypointMap);

    sPath GenerateTrajectory(
        const sBehavior& behavior,
        const sEgo& ego,
        sPath& previousPath);

private:
    std::vector<sPoint2D> Startup(const sEgo& ego);

    std::vector<sPoint2D> KeepLane(
    	const sBehavior& behavior,
        const sEgo& ego,
        sPath& previousPath);

    std::vector<sPoint2D> AdaptSpeedToLeadingVehicle(
        const sBehavior& behavior,
        const sEgo& ego,
        sPath& previousPath);

    std::vector<sPoint2D> LaneChange(
        const sBehavior& behavior,
        const sEgo& ego,
        const sPath& previousPath);

    void AppendPlanningPoints(
        std::vector<sPoint2D>& referencePointsWorld,
        const double targetD,
        const std::vector<double>& sValues);

    const cWaypointMap* m_waypointMap;
};

void SampleSplineAccelerate(
    const std::vector<sPoint2D>& referencePointsLocal,
    std::vector<sPoint2D>& samplePoints,
    const double v0,
    const double vTarget,
    const double aMax,
    const size_t numPoints);

void SampleSplineDecelerate(
    const std::vector<sPoint2D>& referencePointsLocal,
    std::vector<sPoint2D>& samplePoints,
    const double v0,
    const double vTarget,
    const double aMax,
    const size_t numPoints);

std::vector<sPoint2D> TransformToLocalCoordinates(
    const sPoint2D& referencePoint,
    const double referenceYaw,
    const std::vector<sPoint2D>& points);

std::vector<sPoint2D> TransformToWorldCoordinates(
    const sPoint2D& referencePoint,
    const double referenceYaw,
    const std::vector<sFrenetPt>& frenetPoints);

std::vector<sPoint2D> TransformToWorldCoordinates(
    const sPoint2D& referencePoint,
    const double referenceYaw,
    const std::vector<sPoint2D>& points);

void SetSplinePoints(
    tk::spline& spline,
    const std::vector<sPoint2D>& points);

double PathLength(const std::vector<sPoint2D>& path);

#endif // TRAJECTORY_PLANNER_H
