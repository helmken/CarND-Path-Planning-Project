#include <vector>

#include "conversion_helpers.h"
#include "path_generator.h"
#include "spline.h" // used to smooth out edgy path from waypoints


using namespace std;


sPath GeneratePath(
    const sEgo& ego, 
    const sMap& waypointMap, 
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

    size_t prevPathSize = previousPath.x.size();

    // if previous size is almost empty, use the car as starting reference
    if (prevPathSize < 2)
    {
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
        ref_x = previousPath.x[prevPathSize - 1];
        ref_y = previousPath.y[prevPathSize - 1];

        double ref_x_prev = previousPath.x[prevPathSize - 2];
        double ref_y_prev = previousPath.y[prevPathSize - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // use two points that make the path tangent to the previous path's end point
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // in frenet frame add evenly 30m spaced points ahead of the starting reference
    vector<double> next_wp0 = getXY(
        ego.s + 30,
        (2 + 4 * lane),
        waypointMap.map_waypoints_s,
        waypointMap.map_waypoints_x,
        waypointMap.map_waypoints_y);

    vector<double> next_wp1 = getXY(
        ego.s + 60,
        (2 + 4 * lane),
        waypointMap.map_waypoints_s,
        waypointMap.map_waypoints_x,
        waypointMap.map_waypoints_y);

    vector<double> next_wp2 = getXY(
        ego.s + 90,
        (2 + 4 * lane),
        waypointMap.map_waypoints_s,
        waypointMap.map_waypoints_x,
        waypointMap.map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

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
    tk::spline s;

    // set (x, y) points to the spline
    s.set_points(ptsx, ptsy);

    // define the actual (x, y) points we will use for the planner
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // start with all of the previous path points from last time
    for (size_t i(0); i < previousPath.x.size(); ++i)
    {
        next_x_vals.push_back(previousPath.x[i]);
        next_y_vals.push_back(previousPath.y[i]);
    }

    // calculate how to break up spline points so that we travel
    // at our desired reference velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;

    // fill up the rest of our path planner after filling it with previous points,
    // here we will always output 50 points
    for (size_t i(1); i <= 50 - previousPath.x.size(); ++i)
    {
        double N = (target_dist / (0.2 * referenceVelocity / 2.24)); // 2.24 mph -> m/s
        double x_point = x_add_on + target_x / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotate back to normal after rotating it earlier
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }


    //vector<double> next_x_vals;
    //vector<double> next_y_vals;


    //// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
    //// code from walkthrough video
    ////double dist_inc(0.5); // related to speed 50 mph
    //double dist_inc(0.3); // reducing this value results in lower average speed
    //for (int i(0); i < 50; ++i)
    //{
    //    // use frenet to stay in the lane
    //    double next_s = ego.s + (i + 1) * dist_inc;
    //    double next_d = 6; // related to the width of the road and the position of waypoints
    //    vector<double> xy = getXY(next_s, next_d,
    //        waypointMap.map_waypoints_s, waypointMap.map_waypoints_x, waypointMap.map_waypoints_y);

    //    // straight path
    //    //next_x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)));
    //    //next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));

    //    // use frenet - with this, the car stays in its lane
    //    next_x_vals.push_back(xy[0]);
    //    next_y_vals.push_back(xy[1]);

    //}
    //// ... end of code from walkthrough video



    sPath newPath;
    newPath.x = next_x_vals;
    newPath.y = next_y_vals;

    return newPath;
}