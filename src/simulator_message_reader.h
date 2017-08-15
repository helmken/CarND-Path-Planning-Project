#ifndef SIMULATOR_MESSAGE_READER_H
#define SIMULATOR_MESSAGE_READER_H

#include "ego.h"
#include "json.hpp"

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

#endif // SIMULATOR_MESSAGE_READER_H