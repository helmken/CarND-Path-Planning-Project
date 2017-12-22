#include <stdexcept>
#include <vector>

#include "conversion_helpers.h"
#include "lane_info.h"
#include "trajectory_planner.h"


using std::vector;


constexpr int numPathPoints(50);

constexpr size_t replanBufferLaneChange(10);
constexpr size_t replanBufferFollowVehicle(10);  
constexpr size_t replanBufferKeepLane(10);

constexpr double safetyDistance(5.0);

constexpr double minReusedPathLengthKeepLane(10.0);

constexpr double accelStartup(maxAccel * 0.9);
constexpr double accelAdaptSpeed(maxAccel * 0.8);
constexpr double accelKeepLane(maxAccel * 0.8);
constexpr double accelLaneChange(maxAccel * 0.7);


cTrajectoryPlanner::cTrajectoryPlanner()
    : m_waypointMap(nullptr)
{
}

void cTrajectoryPlanner::Init(const cWaypointMap* waypointMap)
{
    m_waypointMap = waypointMap;
}

sPath cTrajectoryPlanner::GenerateTrajectory(
    const sBehavior& behavior,
    const sEgo& ego,
    sPath& previousPath)
{
    sPath path;
    if (previousPath.points.empty())
    {
        //printf("%s: s=%.3f startup\n", __FUNCTION__, ego.s);
        path.points = Startup(ego);
        return path;
    }
    else if (ES_LANE_KEEP == behavior.currentState)
    {
        if (behavior.adaptSpeedToLeadingVehicle)
        {
            //printf("%s: s=%.3f adapt speed to leading vehicle\n", __FUNCTION__, ego.s);
            path.points = AdaptSpeedToLeadingVehicle(behavior, ego, previousPath);
            return path;
        }
        else
        {
            //printf("%s: s=%.3f keep lane\n", __FUNCTION__, ego.s);
            path.points = KeepLane(behavior, ego, previousPath);
            return path;
        }
    }
    else if (ES_LANE_KEEP != behavior.currentState)
    {
        //printf("%s: s=%.3f change lane\n", __FUNCTION__, ego.s);
        path.points = LaneChange(behavior, ego, previousPath);
        return path;
    }

    //printf("%s: reusing previous path\n", __FUNCTION__);
    return previousPath;
}

std::vector<sPoint2D> cTrajectoryPlanner::Startup(const sEgo& ego)
{
    //printf("%s: ego: s=%.3f, d=%.3f, x=%.3f, y=%.3f\n",
    //    __FUNCTION__, ego.s, ego.d, ego.x, ego.y);

    const auto refAngle = deg2rad(ego.yaw);

    const sPoint2D prevPos(
        ego.x - cos(refAngle),
        ego.y - sin(refAngle));
    const sPoint2D refPos(ego.x, ego.y);

    vector<sPoint2D> referencePointsWorld;
    referencePointsWorld.emplace_back(prevPos);
    referencePointsWorld.emplace_back(refPos);
    
    const double laneD = round(ego.d);
    AppendPlanningPoints(referencePointsWorld, laneD,
        { ego.s + 30, ego.s + 60, ego.s + 90 });
    
    const vector<sPoint2D> referencePointsLocal = TransformToLocalCoordinates(
        refPos, refAngle, referencePointsWorld);

    vector<sPoint2D> samplePointsLocal;
    SampleSplineAccelerate(referencePointsLocal, samplePointsLocal,
        ego.speed, maxSpeed, 
        accelStartup, numPathPoints);

    const vector<sPoint2D> samplePointsWorld = TransformToWorldCoordinates(
        refPos, refAngle, samplePointsLocal);

    return samplePointsWorld;
}

std::vector<sPoint2D> cTrajectoryPlanner::KeepLane(
    const sBehavior& behavior,
    const sEgo& ego,
    sPath& previousPath)
{
    const auto prevPathSize = previousPath.points.size();
    const auto prevPathLength = PathLength(previousPath.points);

    vector<sPoint2D> referencePointsWorld;
    vector<sPoint2D> reusedPath;
 
    auto& pt0 = previousPath.points[prevPathSize - 2];
    auto& refPt = previousPath.points[prevPathSize - 1];
    auto refAngle = atan2(refPt.y - pt0.y, refPt.x - pt0.x);

    if (prevPathLength < minReusedPathLengthKeepLane)
    {
        // reuse whole previous path if path length is small,
        // e.g. at low speed

        //printf("%s: previous path: size=%i, length=%.3f\n",
        //    __FUNCTION__,
        //    prevPathSize,
        //    prevPathLength);

        reusedPath = previousPath.points;
        
        referencePointsWorld.emplace_back(reusedPath[0]);
        referencePointsWorld.emplace_back(reusedPath[prevPathSize - 1]);

        const auto pathEndS = ego.s + prevPathLength;
        AppendPlanningPoints(referencePointsWorld, 
            LaneNameToD(behavior.targetLane),
            { pathEndS + 30, pathEndS + 60, pathEndS + 90 });
    }
    else
    {
        // at higher speeds reuse only a small portion to correct
        // drifts from the center of the lane

        reusedPath = std::vector<sPoint2D>(
            previousPath.points.begin(),
            previousPath.points.begin() + replanBufferKeepLane);
        const auto reusedPathLength = PathLength(reusedPath);

        pt0 = reusedPath[replanBufferKeepLane - 2];
        refPt = reusedPath[replanBufferKeepLane - 1];
        refAngle = atan2(refPt.y - pt0.y, refPt.x - pt0.x);

        referencePointsWorld.emplace_back(reusedPath[0]);
        referencePointsWorld.emplace_back(refPt);
        AppendPlanningPoints(referencePointsWorld,
            LaneNameToD(behavior.targetLane),
            { ego.s + reusedPathLength + 30 });
    }

    const vector<sPoint2D> referencePointsLocal
        = TransformToLocalCoordinates(
        refPt, refAngle, referencePointsWorld);

    const auto v = Distance(refPt, pt0) / cycleTime;
    vector<sPoint2D> samplePoints;
    const auto numMissingPathPoints = numPathPoints - reusedPath.size();
    SampleSplineAccelerate(referencePointsLocal, samplePoints, 
        v, maxSpeed, accelKeepLane, numMissingPathPoints);

    const auto samplePointsWorld = TransformToWorldCoordinates(
        refPt, refAngle, samplePoints);

    for (auto i = 0; i < numMissingPathPoints; ++i)
    {
        reusedPath.emplace_back(samplePointsWorld[i]);
    }

    return reusedPath;
}

std::vector<sPoint2D> cTrajectoryPlanner::AdaptSpeedToLeadingVehicle(
    const sBehavior& behavior,
    const sEgo& ego,
    sPath& previousPath)
{
    //printf("%s: leading vehicle: distance=%.3f, speed=%.3f, ego speed=%.3f\n", 
    //    __FUNCTION__, 
    //    behavior.leadingVehicleDistance, 
    //    behavior.leadingVehicleSpeed,
    //    ego.speed);

    assert(previousPath.points.size() >= replanBufferFollowVehicle);

    std::vector<sPoint2D> reusedPath(
        previousPath.points.begin(),
        previousPath.points.begin() + replanBufferFollowVehicle);

    const auto& pt0 = reusedPath[replanBufferFollowVehicle - 2];
    const auto& refPt = reusedPath[replanBufferFollowVehicle - 1];
    const auto refAngle = atan2(refPt.y - pt0.y, refPt.x - pt0.x);
    const auto reusedPathLength = PathLength(reusedPath);

    vector<sPoint2D> referencePointsWorld;
    referencePointsWorld.emplace_back(reusedPath[0]);
    referencePointsWorld.emplace_back(
        reusedPath[replanBufferFollowVehicle - 1]);
    
    const auto pathEnd = ego.s + reusedPathLength;
    AppendPlanningPoints(referencePointsWorld,
        LaneNameToD(behavior.targetLane),
        { pathEnd + 30, pathEnd + 60, pathEnd + 90 });

    const auto referencePointsLocal = TransformToLocalCoordinates(
        refPt, refAngle, referencePointsWorld);

    const auto v0 = Distance(refPt, pt0) / cycleTime;
    auto targetV = behavior.targetSpeed;

    // check safety distance or fall back
    if (behavior.leadingVehicle.sDistanceEgo < safetyDistance)
    {
        targetV -= (2.0 * cycleTime * accelAdaptSpeed);
		//printf("%s: reduce speed to maintain safety distance: targetV=%.3f\n",
		//	__FUNCTION__, targetV);
    }

    const auto deltaV = targetV - v0;
    vector<sPoint2D> samplePointsLocal;
    const auto missingPathPoints = numPathPoints - replanBufferFollowVehicle;
    if (deltaV >= 0)
    {
        SampleSplineAccelerate(referencePointsLocal, samplePointsLocal,
            v0, targetV, accelAdaptSpeed, missingPathPoints);
    }
    else
    {
        SampleSplineDecelerate(referencePointsLocal, samplePointsLocal,
            v0, targetV, accelAdaptSpeed, missingPathPoints);
    }

    const auto samplePointsWorld = TransformToWorldCoordinates(
        refPt, refAngle, samplePointsLocal);

    const auto numSamplePoints = samplePointsWorld.size();
    for (auto i = 0; i < numSamplePoints; ++i)
    {
        reusedPath.emplace_back(samplePointsWorld[i]);
    }

    return reusedPath;
}

std::vector<sPoint2D> cTrajectoryPlanner::LaneChange(
    const sBehavior& behavior,
    const sEgo& ego,
    const sPath& previousPath)
{
    assert(previousPath.points.size() >= replanBufferLaneChange);

    std::vector<sPoint2D> reusedPath(
        previousPath.points.begin(),
        previousPath.points.begin() + replanBufferLaneChange);

    const auto& pt0 = reusedPath[replanBufferLaneChange - 2];
    const auto& refPt = reusedPath[replanBufferLaneChange - 1];
    const auto refAngle = atan2(refPt.y - pt0.y, refPt.x - pt0.x);
    const auto reusedPathLength = PathLength(reusedPath);

    vector<sPoint2D> referencePointsWorld;
    referencePointsWorld.emplace_back(reusedPath[0]);
    referencePointsWorld.emplace_back(refPt);

    const auto pathEnd = ego.s + reusedPathLength;
    AppendPlanningPoints(referencePointsWorld,
        LaneNameToD(behavior.targetLane),
        { pathEnd + 37, pathEnd + 74 });

    const auto referencePointsLocal = TransformToLocalCoordinates(
        refPt, refAngle, referencePointsWorld);

    tk::spline splineLocal;
    SetSplinePoints(splineLocal, referencePointsLocal);

    const auto v0 = Distance(refPt, pt0) / cycleTime;
    const auto deltaV = behavior.targetSpeed - v0;
    vector<sPoint2D> samplePointsLocal;
    const auto numMissingPathPoints = numPathPoints - replanBufferLaneChange;

    if (deltaV >= 0.0)
    {
        SampleSplineAccelerate(referencePointsLocal, samplePointsLocal,
            v0, behavior.targetSpeed, accelLaneChange, numMissingPathPoints);
    }
    else
    {
        SampleSplineDecelerate(referencePointsLocal, samplePointsLocal,
            v0, behavior.targetSpeed, accelLaneChange, numMissingPathPoints);
    }
    
    const auto samplePointsWorld = TransformToWorldCoordinates(
        refPt, refAngle, samplePointsLocal);

    const auto numSamplePoints = samplePointsWorld.size();
    for (auto i = 0; i < numSamplePoints; ++i)
    {
        reusedPath.emplace_back(samplePointsWorld[i]);
    }

    return reusedPath;
}

void cTrajectoryPlanner::AppendPlanningPoints(
    std::vector<sPoint2D>& referencePointsWorld,
    const double targetD,
    const std::vector<double>& sValues)
{
    for (const auto s : sValues)
    {
        referencePointsWorld.emplace_back(
            m_waypointMap->CartesianPosition(s, targetD));
    }
}

void SampleSplineAccelerate(
    const vector<sPoint2D>& referencePointsLocal,
    vector<sPoint2D>& samplePoints,
    const double v0,
    const double vTarget,
    const double aMax,
    const size_t numPoints)
{
    tk::spline splineLocal;
    SetSplinePoints(splineLocal, referencePointsLocal);

    auto v = v0;
    auto x = 0.0;
    auto y = 0.0;
    for (auto i = 0; i < numPoints; ++i)
    {
        if (v < vTarget)
        {
            v += cycleTime * aMax;
        }

        v = std::min(v, vTarget);
        x += cycleTime * v;
        y = splineLocal(x);
        samplePoints.emplace_back(sPoint2D(x, y));
    }
}

void SampleSplineDecelerate(
    const vector<sPoint2D>& referencePointsLocal,
    vector<sPoint2D>& samplePoints,
    const double v0,
    const double vTarget,
    const double aMax,
    const size_t numPoints)
{
    tk::spline splineLocal;
    SetSplinePoints(splineLocal, referencePointsLocal);

    auto v = v0;
    auto x = 0.0;
    auto y = 0.0;
    for (auto i = 0; i < numPoints; ++i)
    {
        if (v > vTarget)
        {
            v -= cycleTime * aMax;
        }

        v = std::max(v, vTarget);
        x += cycleTime * v;
        y = splineLocal(x);
        samplePoints.emplace_back(sPoint2D(x, y));
    }
}

vector<sPoint2D> TransformToLocalCoordinates(
    const sPoint2D& referencePoint,
    const double referenceYaw,
    const vector<sPoint2D>& points)
{
    vector<sPoint2D> transformed;

    for (size_t i(0); i < points.size(); ++i)
    {
        // 1) shift to origin
        const auto shiftX = points[i].x - referencePoint.x;
        const auto shiftY = points[i].y - referencePoint.y;

        // 2) rotate around yaw
        const auto rotX = (shiftX * cos(-referenceYaw) - shiftY * sin(-referenceYaw));
        const auto rotY = (shiftX * sin(-referenceYaw) + shiftY * cos(-referenceYaw));

        transformed.push_back(sPoint2D(rotX, rotY));
    }

    return transformed;
}

vector<sPoint2D> TransformToWorldCoordinates(
    const sPoint2D& referencePoint,
    const double referenceYaw,
    const vector<sFrenetPt>& frenetPoints)
{
    vector<sPoint2D> transformed;

    for (const auto& frenet : frenetPoints)
    {
        // rotate back to initial orientation to revert earlier rotation
        auto x = (frenet.s * cos(referenceYaw) - frenet.d * sin(referenceYaw));
        auto y = (frenet.s * sin(referenceYaw) + frenet.d * cos(referenceYaw));

        x += referencePoint.x;
        y += referencePoint.y;

        transformed.emplace_back(sPoint2D(x, y));
    }

    return transformed;
}

vector<sPoint2D> TransformToWorldCoordinates(
    const sPoint2D& referencePoint,
    const double referenceYaw,
    const vector<sPoint2D>& points)
{
    vector<sPoint2D> transformed;

    for (size_t i(0); i < points.size(); ++i)
    {
        // rotate back to initial orientation to revert earlier rotation
        auto x = (points[i].x * cos(referenceYaw) - points[i].y * sin(referenceYaw));
        auto y = (points[i].x * sin(referenceYaw) + points[i].y * cos(referenceYaw));

        x += referencePoint.x;
        y += referencePoint.y;

        transformed.push_back(sPoint2D(x, y));
    }

    return transformed;
}

void SetSplinePoints(
    tk::spline& spline,
    const std::vector<sPoint2D>& points)
{
    vector<double> coordsX;
    vector<double> coordsY;

    for (auto pt : points)
    {
        coordsX.push_back(pt.x);
        coordsY.push_back(pt.y);
    }

    spline.set_points(coordsX, coordsY);
}

double PathLength(const std::vector<sPoint2D>& path)
{
    auto length = 0.0;
    
    if (path.empty())
    {
        return length;
    }

    auto lastX = path[0].x;
    auto lastY = path[0].y;

    for (auto pt = path.begin() + 1; pt != path.end(); ++pt)
    {
        length += Distance((*pt).x, (*pt).y, lastX, lastY);

        lastX = (*pt).x;
        lastY = (*pt).y;
    }

    return length;
}
