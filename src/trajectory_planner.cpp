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

sPath GeneratePath(
    const sEgo& ego, 
    const cWaypointMap& waypointMap,
    const sPath& previousPath, 
    const int lane,
    const double referenceVelocity)
{
    // create a list of widely spread (x, y) waypoints, evenly spaced at 30 m
    // later we will interpolate these waypoints with a spline and fill it in
    // with more points that control speed

    vector<double> ptsx;
    vector<double> ptsy;

    // reference x, y, yaw states
    // either we will reference the starting point as where the car is or at
    // the previous paths end point
    double ref_x = ego.x;
    double ref_y = ego.y;
    double ref_yaw = deg2rad(ego.yaw);

    size_t prevPathSize = previousPath.coordsX.size();

    // if previous size is almost empty, use the car as starting reference
    if (prevPathSize < 2)
    {
        // happens at the start of simulation...
        // use two points that make the path tangent to the car
        double prev_car_x = ego.x - cos(ego.yaw);
        double prev_car_y = ego.y - sin(ego.yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(ego.x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(ego.y);
    }
    else
    {
        // use the previous path's end point as starting reference

        // redefine reference state as previous path end point
        ref_x = previousPath.coordsX[prevPathSize - 1];
        ref_y = previousPath.coordsY[prevPathSize - 1];

        double ref_x_prev = previousPath.coordsX[prevPathSize - 2];
        double ref_y_prev = previousPath.coordsY[prevPathSize - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // use two points that make the path tangent to the previous path's end point
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // in frenet frame add evenly 30m spaced points ahead of the starting reference
    const double frenetDCoord(2 + 4 * lane);
    s2DCoordCart next_wp0 = getXY(
        ego.s + 30,
        frenetDCoord,
        waypointMap.GetMapPointsS(),
        waypointMap.GetMapPointsX(),
        waypointMap.GetMapPointsY());

    s2DCoordCart next_wp1 = getXY(
        ego.s + 60,
        frenetDCoord,
        waypointMap.GetMapPointsS(),
        waypointMap.GetMapPointsX(),
        waypointMap.GetMapPointsY());

    s2DCoordCart next_wp2 = getXY(
        ego.s + 90,
        frenetDCoord,
        waypointMap.GetMapPointsS(),
        waypointMap.GetMapPointsX(),
        waypointMap.GetMapPointsY());

    ptsx.push_back(next_wp0.x);
    ptsx.push_back(next_wp1.x);
    ptsx.push_back(next_wp2.x);

    ptsy.push_back(next_wp0.y);
    ptsy.push_back(next_wp1.y);
    ptsy.push_back(next_wp2.y);

    // transformation to local car coordinates
    for (size_t i(0); i < ptsx.size(); ++i)
    {
        // shift car reference angle to 0 degrees (as in MPC project)
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        // rotate
        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    // create a spline
    tk::spline pathSpline;

    // set (x, y) points to the spline
    pathSpline.set_points(ptsx, ptsy);

    // define the actual (x, y) points we will use for the planner
    // start with all of the previous path points from last time
    sPath newPath = previousPath;

    // calculate how to break up spline points so that we travel
    // at our desired reference velocity
    double target_x = 30.0;
    double target_y = pathSpline(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;

    // fill up the rest of our path planner after filling it with previous points,
    // here we will always output 50 points
    for (size_t i(1); i <= 50 - previousPath.coordsX.size(); ++i)
    {
        double N = (target_dist / (0.2 * referenceVelocity / 2.24)); // 2.24 mph -> m/s
        double x_point = x_add_on + target_x / N;
        double y_point = pathSpline(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotate back to normal after rotating it earlier
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        newPath.coordsX.push_back(x_point);
        newPath.coordsY.push_back(y_point);
    }

    return newPath;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
s2DCoordFrenet getFrenet(
    double x, double y, double theta,
    std::vector<double> maps_x,
    std::vector<double> maps_y)
{
    int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0)
    {
        prev_wp = maps_x.size() - 1;
    }

    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

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

    frenet_s += distance(0, 0, proj_x, proj_y);

    return s2DCoordFrenet(frenet_s, frenet_d);
    //return { frenet_s, frenet_d };
}

// Transform from Frenet s,d coordinate to Cartesian x,y coordinate
// TODO: according to slack getXY should not be used - instead splines should be used
s2DCoordCart getXY(
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
    
    // the x,y,s along the segment
    const double deltaS = (s - maps_s[waypointIdx0]);

    const double deltaX = maps_x[waypointIdx0] + deltaS * cos(heading);
    const double deltaY = maps_y[waypointIdx0] + deltaS * sin(heading);

    const double headingPerp = heading - M_PI / 2;

    const double x = deltaX + d * cos(headingPerp);
    const double y = deltaY + d * sin(headingPerp);

    return s2DCoordCart(x, y);
}

s2DCoordCart FrenetToCartesian(
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

    return s2DCoordCart(x, y);
}