#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <thread>
#include <vector>
#include <map>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h" // used to smooth out edgy path from waypoints

#include "ego.h"
#include "conversion_helpers.h"
#include "simulator_message_reader.h"
#include "waypoint_map.h"


using namespace std;

// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) 
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) 
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos) 
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

int main() 
{
    uWS::Hub uwsHub;

    sMap waypointMap = ReadMapFile();

    // lane 0 is left, 1 is middle, 2 is right lane
    // start in middle lane:
    int lane = 1;

    // have a reference velocity to target
    // double ref_vel = 49.5; // mph <- the high speed value causes a big jerk at initialization
    double ref_vel = 0.0; // mph <- start slow and increase velocity

    uwsHub.onMessage(
        [&waypointMap, &lane, &ref_vel]
        (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') 
        {
            auto jsonString = hasData(data);

            if (jsonString != "") 
            {
                auto simMessage = json::parse(jsonString);
                //cout << simMessage << "\n";

                string simEventType = simMessage[0].get<string>();
                if (simEventType == "telemetry")
                {
                    json telemetry = simMessage[1];

                    // Main car's localization Data
                    sEgo ego = ReadEgoFromJson(telemetry);

                    // Previous path data given to the Planner
                    sPath previousPath = ReadPathFromJson(telemetry);

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    json sensorFusion = telemetry["sensor_fusion"];

                    vector<sDynamicObject> dynamicObjects = ReadDynamicObjects(sensorFusion);

                    // code from walkthrough video
                    size_t prev_size = previousPath.x.size();

                    if (prev_size > 0)
                    {
                        ego.s = previousPath.endS;
                    }

                    bool too_close = false;

                    // sensorFusion: list of other cars on the highway?!?
                    // to avoid hitting other cars: go through sensorFusion list and check if 
                    // another car is in our lane
                    // if yes: check how close
                    
                    // find rev_v to use
                    // i is index of other car on the road 
                    for (size_t i(0); i < sensorFusion.size(); ++i)
                    {
                        // car is in my lane
                        // d is position of i-th car on the road
                        // d tells us on what lane the other car is
                        double d = sensorFusion[i][6]; 

                        // lane is our lane 
                        if (d < (2 + 4 * lane + 2) && d >(2 + 4 * lane - 2))
                        {
                            // so the car is in our lane
                            double vx = sensorFusion[i][3];
                            double vy = sensorFusion[i][4];
                            double check_speed = sqrt(vx * vx + vy * vy);
                            double check_car_s = sensorFusion[i][5];

                            // check_car_s can help us to predict where that car is in the future  
                            check_car_s += ((double)prev_size * 0.2 * check_speed); // if using previous points can project s value out
                            
                            // check s values greater than mine and s gap
                            if ((check_car_s > ego.s) && (check_car_s - ego.s) < 30)
                            {
                                // check if our car is close to the other car -> if so, need to take action

                                // do some logic here, lower reference velocity so we dont crash into the car in front of us,
                                // could also flag to try to change lanes
                                
                                // ref_vel = 29.5; // mph

                                // lines below consider this flag and reduce speed
                                too_close = true;
                                if (lane > 0)
                                {
                                    lane = 0;
                                }
                            }
                        }
                    }


                    if (too_close)
                    {
                        ref_vel -= 0.224; // this is somehow related to decelerating with 5 m/s^2
                    }
                    else if (ref_vel < 49.5)
                    {
                        ref_vel += 0.224;
                    }


                    // create a list of widely spred (x, y) waypoints, evenly spaced at 30 m
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

                    // if previous size is almost empty, use the car as starting reference
                    if (prev_size < 2)
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
                        ref_x = previousPath.x[prev_size - 1];
                        ref_y = previousPath.y[prev_size - 1];

                        double ref_x_prev = previousPath.x[prev_size - 2];
                        double ref_y_prev = previousPath.y[prev_size - 2];
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                        // use two points that make the path tangent to the previous path's end point
                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                    }

                    // in frenet frame add evenly 30m spaced points ahead of the starting reference
                    vector<double> next_wp0 = getXY(ego.s + 30, (2 + 4 * lane),
                        waypointMap.map_waypoints_s, waypointMap.map_waypoints_x, waypointMap.map_waypoints_y);
                    vector<double> next_wp1 = getXY(ego.s + 60, (2 + 4 * lane),
                        waypointMap.map_waypoints_s, waypointMap.map_waypoints_x, waypointMap.map_waypoints_y);
                    vector<double> next_wp2 = getXY(ego.s + 90, (2 + 4 * lane),
                        waypointMap.map_waypoints_s, waypointMap.map_waypoints_x, waypointMap.map_waypoints_y);

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
                        double N = (target_dist / (0.2 * ref_vel / 2.24)); // 2.24 mph -> m/s
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


                    json msgJson;

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


                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
            else 
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    uwsHub.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
        size_t, size_t) 
    {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) 
        {
            res->end(s.data(), s.length());
        }
        else 
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    uwsHub.onConnection([&uwsHub](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        std::cout << "Connected!!!" << std::endl;
    });

    uwsHub.onDisconnection([&uwsHub](uWS::WebSocket<uWS::SERVER> ws, int code,
        char *message, size_t length) 
    {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (uwsHub.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else 
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    uwsHub.run();
}
