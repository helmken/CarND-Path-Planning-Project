#ifndef SIMULATOR_MESSAGE_READER_H
#define SIMULATOR_MESSAGE_READER_H

#include "vehicle.h"
#include "ego.h"
#include "json.hpp"
#include "path.h"

sEgo JsonReadEgo(const nlohmann::json& telemetry)
{
    sEgo ego(
        telemetry["x"], 
        telemetry["y"],
        telemetry["s"],
        telemetry["d"],
        telemetry["yaw"],
        telemetry["speed"]);
    
    return ego;
}

sPath JsonReadPath(const nlohmann::json& telemetry)
{
    sPath path(
        telemetry["previous_path_x"],
        telemetry["previous_path_y"]);

    return path;
}

sVehicle ReadVehicle(const nlohmann::json& sensorFusion)
{
    sVehicle vehicle(
            sensorFusion[0],
            sensorFusion[1],
            sensorFusion[2],
            sensorFusion[3],
            sensorFusion[4],
            sensorFusion[5],
            sensorFusion[6]);

    return vehicle;
}

std::vector<sVehicle> JsonReadRoadUsers(const nlohmann::json& sensorFusion)
{
    std::vector<sVehicle> vehicles;
    for (auto i(0); i < sensorFusion.size(); ++i)
    {
        vehicles.push_back(ReadVehicle(sensorFusion[i]));
    }

    return vehicles;
}

#endif // SIMULATOR_MESSAGE_READER_H
