#include <vector>

#include "conversion_helpers.h"
#include "trajectory_planner.h"
#include "spline.h" // used to smooth out edgy path from waypoints


using namespace std;


cTrajectoryPlanner::cTrajectoryPlanner()
{
}

void cTrajectoryPlanner::Init()
{
}

sPath GeneratePath(
    const sBehavior& plannedBehavior,
    const sEgo& ego,
    const cWaypointMap& waypointMap,
    const sPath& previousPath)
{
    // TODO: use code from lesson 5.24
    
    sPath plannedPath;
    // TODO: implement real path planning for given situation



    return plannedPath;
}

void PrintPathPoints(
    const std::string& info,
    const vector<s2DPtCart>& pathPoints)
{
    printf("%s\n", info.c_str());
    for (size_t i(0); i < pathPoints.size(); ++i)
    {
        printf("%i: (%.3f,%.3f)\n", i, pathPoints[i].x, pathPoints[i].y);
    }
}

void PrintPath(
    const std::string& info,
    const sPath& path)
{
    printf("%s\n", info.c_str());
    for (size_t i(0); i < path.coordsX.size(); ++i)
    {
        printf("%i: (%.3f,%.3f)\n", i, path.coordsX[i], path.coordsY[i]);
    }
}


// reference (x, y, yaw) state
// either we will reference the starting point as where the car is or at
// the previous paths end point
void GetFirstTwoReferencePoints(
    const sEgo& ego,
    const sPath& previousPath,
    s2DPtCart& referencePoint,
    s2DPtCart& previousPoint,
    double& referenceYaw)
{
    const size_t prevPathSize = previousPath.coordsX.size();

    // if previous size is almost empty, use the car as starting reference
    if (prevPathSize < 2)
    {
        // happens at the start of simulation...
        // use two points that make the path tangent to the car

        referencePoint.x = ego.x;
        referencePoint.y = ego.y;
        referenceYaw = deg2rad(ego.yaw);

        previousPoint.x = ego.x - cos(ego.yaw);
        previousPoint.y = ego.y - sin(ego.yaw);
    }
    else
    {
        // use the previous path's end point as starting reference

        // redefine reference state as previous path end point
        // use two points that make the path tangent to the previous path's end point
        referencePoint.x = previousPath.coordsX[prevPathSize - 1];
        referencePoint.y = previousPath.coordsY[prevPathSize - 1];

        previousPoint.x = previousPath.coordsX[prevPathSize - 2];
        previousPoint.y = previousPath.coordsY[prevPathSize - 2];
        referenceYaw = atan2(
            referencePoint.y - previousPoint.y, 
            referencePoint.x - previousPoint.x);
    }
}

// create 5 reference points so that 
// point 0 is the second last point on the path
// point 1 is the last point on the path
// points 2..4 are 30, 60, 90 meters ahead on the target lane
void CreateFiveReferencePoints(
    const sEgo& ego,
    const cWaypointMap& waypointMap,
    const sPath& previousPath,
    const int targetLane,
    vector<s2DPtCart>& referencePoints,
    double& referenceYaw)
{
    // reference (x, y, yaw) state
    // either we will reference the starting point as where the car is or at
    // the previous paths end point
    s2DPtCart referencePoint(0.0, 0.0);
    s2DPtCart previousPoint(0.0, 0.0);
    GetFirstTwoReferencePoints(
        ego, previousPath,
        referencePoint, previousPoint, referenceYaw);

    referencePoints.push_back(previousPoint);
    referencePoints.push_back(referencePoint);

    const double waypointDist = previousPath.Length() > 30.0 ?
        previousPath.Length() + 2 : 30.0;
    
    // in frenet frame add evenly 30m spaced points ahead of the starting reference
    const double frenetDCoord(2 + 4 * targetLane);
    s2DPtCart nextWp30 = getXY(
        ego.s + waypointDist, // + 30; 
        frenetDCoord,
        waypointMap.GetMapPointsS(),
        waypointMap.GetMapPointsX(),
        waypointMap.GetMapPointsY());
    referencePoints.push_back(nextWp30);

    s2DPtCart nextWp60 = getXY(
        ego.s + 60,
        frenetDCoord,
        waypointMap.GetMapPointsS(),
        waypointMap.GetMapPointsX(),
        waypointMap.GetMapPointsY());
    referencePoints.push_back(nextWp60);

    s2DPtCart nextWp90 = getXY(
        ego.s + 90,
        frenetDCoord,
        waypointMap.GetMapPointsS(),
        waypointMap.GetMapPointsX(),
        waypointMap.GetMapPointsY());
    referencePoints.push_back(nextWp90);
}

void SetSplinePoints(tk::spline& pathSpline, const vector<s2DPtCart>& pathPoints)
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

vector<s2DPtCart> TransformToLocalCoordinates(
    const s2DPtCart& referencePoint,
    const double referenceYaw,
    const vector<s2DPtCart>& points)
{
    vector<s2DPtCart> transformed;

    for (size_t i(0); i < points.size(); ++i)
    {
        // 1) shift to origin
        const double shiftX = points[i].x - referencePoint.x;
        const double shiftY = points[i].y - referencePoint.y;

        // 2) rotate around yaw
        const double rotX = (shiftX * cos(-referenceYaw) - shiftY * sin(-referenceYaw));
        const double rotY = (shiftX * sin(-referenceYaw) + shiftY * cos(-referenceYaw));

        transformed.push_back(s2DPtCart(rotX, rotY));
    }

    return transformed;
}

vector<s2DPtCart> TransformToWorldCoordinates(
    const s2DPtCart& referencePoint,
    const double referenceYaw,
    const vector<s2DPtCart>& points)
{
    vector<s2DPtCart> transformed;

    for (size_t i(0); i < points.size(); ++i)
    {
        // rotate back to initial orientation to revert earlier rotation
        double x = (points[i].x * cos(referenceYaw) - points[i].y * sin(referenceYaw));
        double y = (points[i].x * sin(referenceYaw) + points[i].y * cos(referenceYaw));

        x += referencePoint.x;
        y += referencePoint.y;

        transformed.push_back(s2DPtCart(x, y));
    }

    return transformed;
}

const unsigned int numOfPathPoints(50);
const double avgDeltaT = 0.02; // 20 milli seconds as seconds

sPath GeneratePath(
    const sEgo& ego, 
    const cWaypointMap& waypointMap,
    const sPath& previousPath, 
    const int targetLane,
    const double referenceVelocity)
{
    // create a list of widely spread (x, y) waypoints, evenly spaced at 30 m
    // later we will interpolate these waypoints with a spline and fill it in
    // with more points that control speed

    vector<s2DPtCart> referencePoints;
    double referenceYaw(0.0);

    CreateFiveReferencePoints(ego, waypointMap, previousPath, targetLane,
        referencePoints, referenceYaw);
    
    // copy reference point to avoid overwriting during transformation
    const s2DPtCart referencePoint = referencePoints[1];

    //printf("ego position: %s\n", ToString(ego).c_str());
    //PrintPath("previous path", previousPath);
    //PrintPathPoints("reference points", referencePoints);

    vector<s2DPtCart> localCoords = TransformToLocalCoordinates(
        referencePoint, referenceYaw, referencePoints);

    //PrintPathPoints("reference points in local coordinates", localCoords);

    // create a spline
    tk::spline pathSpline;

    // set (x, y) points to the spline
    SetSplinePoints(pathSpline, localCoords);

    // define the actual (x, y) points we will use for the planner
    // start with all of the previous path points from last time
    sPath newPath = previousPath;

    // calculate how to break up spline points so that we travel
    // at our desired reference velocity
    double splineTargetX = 30.0;
    double splineTargetY = pathSpline(splineTargetX);
    double splineTargetDistance = sqrt(pow(splineTargetX, 2) + pow(splineTargetY, 2));

    double splineSamplePos = 0;

    const double stepSize = avgDeltaT * referenceVelocity;

    vector<s2DPtCart> additionalPathPoints;

    // fill up the rest of our path planner after filling it with previous points,
    // here we will always output 50 points
    const unsigned int prevPathSize = previousPath.coordsX.size();
    for (size_t i(1); i <= numOfPathPoints - prevPathSize; ++i)
    {
        double ptX = splineSamplePos + stepSize;
        double ptY = pathSpline(ptX);

        splineSamplePos = ptX;

        additionalPathPoints.push_back(s2DPtCart(ptX, ptY));
    }

    additionalPathPoints = TransformToWorldCoordinates(
        referencePoint, referenceYaw, additionalPathPoints);

    for (size_t i(0); i < additionalPathPoints.size(); ++i)
    {
        newPath.coordsX.push_back(additionalPathPoints[i].x);
        newPath.coordsY.push_back(additionalPathPoints[i].y);

        //printf("adding point %i (%.3f,%.3f), numOfSteps=%.3f, stepSize=%.3f\n", 
        //    i + prevPathSize, additionalPathPoints[i].x, additionalPathPoints[i].y, numOfSteps, stepSize);
    }

    //printf("new path length: %.3f\n", newPath.Length());

    return newPath;
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
s2DPtCart getXY(
    const double s, const double d,
    const std::vector<double>& maps_s,
    const std::vector<double>& maps_x,
    const std::vector<double>& maps_y)
{
    int waypointIdx0 = -1;

    // the maximum value of s in the standard track is 6914.1492576599103
    // when reaching this point, the app crashes!
    const double maxS = maps_s[maps_s.size() - 1];

    while (s > maps_s[waypointIdx0 + 1] && (waypointIdx0 < (int)(maps_s.size() - 1)))
    {
        waypointIdx0++;
    }

    int waypointIdx1 = (waypointIdx0 + 1) % maps_x.size();

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

    return s2DPtCart(x, y);
}

s2DPtCart FrenetToCartesian(
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

    return s2DPtCart(x, y);
}