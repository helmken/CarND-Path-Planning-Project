#ifndef SIMULATOR_MESSAGE_READER_H
#define SIMULATOR_MESSAGE_READER_H

#include "dynamic_object.h"
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
        telemetry["previous_path_y"],
        telemetry["end_path_s"],
        telemetry["end_path_d"]);

    return path;
}

sDynamicObject ReadDynamicObject(const nlohmann::json& sensorFusion)
{
    sDynamicObject dynamicObj(
            sensorFusion[0],
            sensorFusion[1],
            sensorFusion[2],
            sensorFusion[3],
            sensorFusion[4],
            sensorFusion[5],
            sensorFusion[6]);

    return dynamicObj;
}

std::vector<sDynamicObject> JsonReadDynamicObjects(const nlohmann::json& sensorFusion)
{
    std::vector<sDynamicObject> dynamicObjects;
    for (size_t i(0); i < sensorFusion.size(); ++i)
    {
        dynamicObjects.push_back(ReadDynamicObject(sensorFusion[i]));
    }

    return dynamicObjects;
}

#endif // SIMULATOR_MESSAGE_READER_H