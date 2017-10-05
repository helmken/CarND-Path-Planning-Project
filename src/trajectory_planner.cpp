#include <limits>
#include <vector>

#include "conversion_helpers.h"
#include "trajectory_planner.h"


using namespace std;


cTrajectoryPlanner::cTrajectoryPlanner()
{
}

void cTrajectoryPlanner::Init()
{
}

// using fixed time horizon to realize planned behavior
// according to rubric, a lane change should not take longer than 3 s
const double timeHorizon(2.0);

// reusing portion of previous path to avoid jerks
const double timeToFollowPreviousPath(0.5);

const double avgDeltaT = 0.02; // 20 milli seconds as seconds

sPath GeneratePath(
    const sBehavior& plannedBehavior,
    const sEgo& ego,
    const cWaypointMap& waypointMap,
    const sPath& previousPath)
{
    // TODO: use code from lesson 5.24
    
    // reuse waypoints from previous path
    const double desiredPathPortionLength = ego.speed * timeToFollowPreviousPath;
    double reusedPathPointsLength(0);
    vector<sPoint2D> reusedPathPoints = previousPath.PathPortion(
        desiredPathPortionLength,
        reusedPathPointsLength);

    const double deltaD = LaneNameToD(plannedBehavior.targetLane) - ego.d;

    const double plannedPathLength = 50; // in meters TODO: ego.speed * timeHorizon;

    vector<sPoint2D> referencePoints;
    double referenceYaw(0.0);
    sPoint2D referencePoint(0.0, 0.0);
    
    if (ego.speed < 0.1)
    {
        // generate path points based on ego position
        
        referenceYaw = deg2rad(ego.yaw);
        referencePoints.push_back(
            sPoint2D(
                ego.x - cos(referenceYaw),
                ego.y - sin(referenceYaw)));

        referencePoint = sPoint2D(ego.x, ego.y);
        referencePoints.push_back(referencePoint);

        sPoint2D wpEnd = getXY(
            ego.s + plannedPathLength,
            ego.d + deltaD,
            waypointMap);
        referencePoints.push_back(wpEnd);
    }
    else
    {
        // reuse portion of previous path

        const sPoint2D& pt0 = reusedPathPoints[reusedPathPoints.size() - 2];
        const sPoint2D& pt1 = reusedPathPoints[reusedPathPoints.size() - 1];
        referenceYaw = atan2(pt1.y - pt0.y, pt1.x - pt0.x);
        referencePoint = pt1;

        referencePoints.push_back(pt0);
        referencePoints.push_back(pt1);

        sPoint2D wpMiddle = getXY(
            ego.s + plannedPathLength / 2.0,
            ego.d + deltaD / 2.0,
            waypointMap);
        referencePoints.push_back(wpMiddle);

        sPoint2D wpEnd = getXY(
            ego.s + plannedPathLength,
            ego.d + deltaD,
            waypointMap);
        referencePoints.push_back(wpEnd);

        sPoint2D wpPastEnd = getXY(
            ego.s + plannedPathLength + desiredPathPortionLength,
            ego.d + deltaD,
            waypointMap);
        referencePoints.push_back(wpPastEnd);
    }

    //PrintReferencePoints("reference points in world coordinates", referencePoints);

    vector<sPoint2D> referencePointsLocalCoords = TransformToLocalCoordinates(
        referencePoint, referenceYaw, referencePoints);
    //PrintReferencePoints("path points in local coordinates", referencePointsLocalCoords);



    // create a spline
    tk::spline pathSpline;
    SetSplinePoints(pathSpline, referencePointsLocalCoords);

    // sample spline 
    vector<sPoint2D> generatedPathPointsLocal;
    
    const double speed = reusedPathPoints.size() > 2 ?
        SpeedAtEndOfPath(reusedPathPoints) : ego.speed;
    const double deltaT = timeHorizon - timeToFollowPreviousPath;

    SamplePathSpline(
        pathSpline, 
        plannedPathLength, 
        speed, 
        plannedBehavior.speedAtTargetPosition,
        deltaT,
        generatedPathPointsLocal);

    vector<sPoint2D> generatedPathPointsWorld = TransformToWorldCoordinates(
        referencePoint, referenceYaw, generatedPathPointsLocal);

    sPath plannedPath;
    for (size_t i(0); i < reusedPathPoints.size(); ++i)
    {
        plannedPath.points.push_back(reusedPathPoints[i]);
    }

    for (size_t i(0); i < generatedPathPointsWorld.size(); ++i)
    {
        plannedPath.points.push_back(generatedPathPointsWorld[i]);
    }

    return plannedPath;
}

double SpeedAtEndOfPath(const std::vector<sPoint2D>& path)
{
    if (path.size() < 2)
    {
        return 0;
    }

    // assuming time between last points is avgDeltaT
    const sPoint2D& pt0 = path[path.size() - 2];
    const sPoint2D& pt1 = path[path.size() - 1];
    
    const double dist = distance(pt0.x, pt0.y, pt1.x, pt1.y);
    return dist / avgDeltaT;
}

void SamplePathSpline(
    tk::spline& pathSpline,
    const double plannedPathLength,
    const double speed0,
    const double speedTarget,
    const double deltaT,
    vector<sPoint2D>& generatedPathPointsLocal)
{
    // spline starts at penultimate point of reused previous path

    const double splineTargetX = plannedPathLength;
    const double splineTargetY = pathSpline(splineTargetX);
    const double splineTargetDistance = sqrt(pow(splineTargetX, 2) + pow(splineTargetY, 2));

    const double deltaSpeed = speedTarget - speed0;
    const double acc = deltaSpeed / deltaT;

    double t(0.0);
    double speed = speed0;
    double x = 0.0;
    while (t < deltaT)
    {
        t += avgDeltaT;
        speed += acc * avgDeltaT;
        x += speed * avgDeltaT;
        double y = pathSpline(x);
        generatedPathPointsLocal.push_back(sPoint2D(x, y));
    }

    //const double stepSize = avgDeltaT * targetSpeed;
    //double splineSamplePos = 0;
    //while (splineSamplePos < splineTargetDistance)
    //{
    //    double ptX = splineSamplePos + stepSize;
    //    double ptY = pathSpline(ptX);
    //    generatedPathPointsLocal.push_back(sPoint2D(ptX, ptY));
    //    splineSamplePos = ptX;
    //}
}

void SetSplinePoints(tk::spline& pathSpline, const vector<sPoint2D>& pathPoints)
{
    vector<double> ptsx(pathPoints.size());
    vector<double> ptsy(pathPoints.size());

    for (size_t i(0); i < pathPoints.size(); ++i)
    {
        ptsx[i] = pathPoints[i].x;
        ptsy[i] = pathPoints[i].y;
    }

    pathSpline.set_points(ptsx, ptsy);
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
        const double shiftX = points[i].x - referencePoint.x;
        const double shiftY = points[i].y - referencePoint.y;

        // 2) rotate around yaw
        const double rotX = (shiftX * cos(-referenceYaw) - shiftY * sin(-referenceYaw));
        const double rotY = (shiftX * sin(-referenceYaw) + shiftY * cos(-referenceYaw));

        transformed.push_back(sPoint2D(rotX, rotY));
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
        double x = (points[i].x * cos(referenceYaw) - points[i].y * sin(referenceYaw));
        double y = (points[i].x * sin(referenceYaw) + points[i].y * cos(referenceYaw));

        x += referencePoint.x;
        y += referencePoint.y;

        transformed.push_back(sPoint2D(x, y));
    }

    return transformed;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
s2DCoordFrenet getFrenet(
    double x, double y, double theta,
    std::vector<double> maps_x,
    std::vector<double> maps_y)
{
    int next_wp = FindNextWaypointIdx(x, y, theta, maps_x, maps_y);

    int prev_wp = next_wp - 1;
    if (next_wp == 0)
    {
        prev_wp = maps_x.size() - 1;
    }

    double mapDeltaX = maps_x[next_wp] - maps_x[prev_wp];
    double mapDeltaY = maps_y[next_wp] - maps_y[prev_wp];
    double dx = x - maps_x[prev_wp];
    double dy = y - maps_y[prev_wp];

    // find the projection of x onto normal from map
    double projNorm =   (dx * mapDeltaX + dy * mapDeltaY) 
                       / (mapDeltaX * mapDeltaX + mapDeltaY * mapDeltaY);
    double projX = projNorm * mapDeltaX;
    double projY = projNorm * mapDeltaY;

    double frenet_d = distance(dx, dy, projX, projY);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, dx, dy);
    double centerToRef = distance(center_x, center_y, projX, projY);

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

    frenet_s += distance(0, 0, projX, projY);

    return s2DCoordFrenet(frenet_s, frenet_d);
}

// Transform from Frenet s,d coordinate to Cartesian x,y coordinate
// TODO: according to slack getXY should not be used - instead splines should be used
sPoint2D getXY(
    const double s, const double d,
    const cWaypointMap& waypointMap)
{
    const std::vector<double>& maps_s = waypointMap.GetMapPointsS();
    const std::vector<double>& maps_x = waypointMap.GetMapPointsX();
    const std::vector<double>& maps_y = waypointMap.GetMapPointsY();

    //int waypointIdx0 = -1;

    //// the maximum value of s in the standard track is 6914.1492576599103
    //// when reaching this point, the app crashes!
    //const double maxS = maps_s[maps_s.size() - 1];
    //
    //if (s > maxS)
    //{
    //    s -= maxS;
    //}
    //
    //while (s > maps_s[waypointIdx0 + 1] && (waypointIdx0 < (int)(maps_s.size() - 1)))
    //{
    //    waypointIdx0++;
    //}

    //int waypointIdx1 = (waypointIdx0 + 1) % maps_x.size();

    int waypointIdx1 = waypointMap.FindWaypointIdxForS(s);
    int waypointIdx0 = waypointIdx1 > 0 ? 
        waypointIdx1 - 1 : maps_s.size() - 1;

    double heading = atan2(
        (maps_y[waypointIdx1] - maps_y[waypointIdx0]), 
        (maps_x[waypointIdx1] - maps_x[waypointIdx0]));
    
    // the x, y, s along the segment
    const double deltaS = (s - maps_s[waypointIdx0]);

    const double deltaX = maps_x[waypointIdx0] + deltaS * cos(heading);
    const double deltaY = maps_y[waypointIdx0] + deltaS * sin(heading);

    const double headingPerp = heading - M_PI / 2;

    const double x = deltaX + d * cos(headingPerp);
    const double y = deltaY + d * sin(headingPerp);

    return sPoint2D(x, y);
}

sPoint2D FrenetToCartesian(
    const sWaypoint& wp0, const sWaypoint& wp1,
    const double s, const double d)
{
    const double heading = atan2(
        wp1.y - wp0.y,
        wp1.x - wp0.x);

    // the x,y,s along the segment
    const double deltaS = (s - wp0.s);

    const double deltaX = wp0.x + deltaS * cos(heading);
    const double deltaY = wp0.y + deltaS * sin(heading);

    const double headingPerp = heading - M_PI / 2;

    const double x = deltaX + d * cos(headingPerp);
    const double y = deltaY + d * sin(headingPerp);

    return sPoint2D(x, y);
}

void PrintReferencePoints(
    const std::string& info,
    const vector<sPoint2D>& pathPoints)
{
    printf("%s\n", info.c_str());
    for (size_t i(0); i < pathPoints.size(); ++i)
    {
        if (0 == i)
        {
            printf("%i: (%.3f,%.3f)\n", i, pathPoints[i].x, pathPoints[i].y);
        }
        else
        {
            const double dist = distance(
                pathPoints[i].x, pathPoints[i].y,
                pathPoints[i - 1].x, pathPoints[i - 1].y);
            printf("%i: (%.3f,%.3f), dist=%.3f\n", i, 
                pathPoints[i].x, pathPoints[i].y, dist);
        }
    }
}

void PrintPath(
    const std::string& info,
    const sPath& path)
{
    printf("%s\n", info.c_str());
    for (size_t i(0); i < path.points.size(); ++i)
    {
        printf("%i: (%.3f,%.3f)\n", i, path.points[i].x, path.points[i].y);
    }
}
