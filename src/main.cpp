#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <thread>
#include <vector>
#include <map>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "ego.h"
#include "path_planner.h"
#include "simulator_message_reader.h"
#include "visualization.h"


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

    cWaypointMap waypointMap;
    waypointMap.ReadMapFile();

    cPathPlanner pathPlanner(waypointMap);

    cVisualization visualization(waypointMap);
    visualization.SetupGL();

    try
    {
        uwsHub.onMessage(
            [&pathPlanner, &visualization]
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

                        // Sensor Fusion Data, a list of all other cars on the same side of the road.
                        json sensorFusion = telemetry["sensor_fusion"];

                        vector<sDynamicObject> dynamicObjects = ReadDynamicObjectsFromJson(sensorFusion);

                        // Previous path data given to the Planner
                        sPath previousPath = ReadPathFromJson(telemetry);

                        sPath newPath = pathPlanner.Execute(
                            ego,
                            dynamicObjects,
                            previousPath);

                        json msgJson;
                        msgJson["next_x"] = newPath.coordsX;
                        msgJson["next_y"] = newPath.coordsY;

                        auto msg = "42[\"control\"," + msgJson.dump() + "]";

                        //this_thread::sleep_for(chrono::milliseconds(1000));
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                        visualization.Draw(
                            ego,
                            dynamicObjects,
                            previousPath,
                            newPath);
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
    }
    catch (std::exception& ex)
    {
        std::cerr << "exception: " << ex.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "exception thrown" << std::endl;
    }

    // We don't need this since we're not using HTTP but if it's removed the
    // program doesn't compile :-(
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
    //if (uwsHub.listen(port))
    if (uwsHub.listen("127.0.0.1", port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else 
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    uwsHub.run();

    visualization.ShutdownGL();
}
