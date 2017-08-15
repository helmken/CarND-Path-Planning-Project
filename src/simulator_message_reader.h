#ifndef SIMULATOR_MESSAGE_READER_H
#define SIMULATOR_MESSAGE_READER_H

#include "ego.h"
#include "json.hpp"
#include "path.h"

sEgo ReadEgoFromJson(const nlohmann::json& telemetry)
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

sPath ReadPathFromJson(const nlohmann::json& telemetry)
{
    sPath path(
        telemetry["previous_path_x"],
        telemetry["previous_path_y"],
        telemetry["end_path_s"],
        telemetry["end_path_d"]);

    return path;
}

#endif // SIMULATOR_MESSAGE_READER_H