#include <iostream>
#include <sstream>

#include "behavior.h"
#include "ego.h"


sBehavior::sBehavior()
    : currentState(ES_LANE_KEEP)
	, currentLane(LN_UNDEFINED)
    , adaptSpeedToLeadingVehicle(false)
    , targetSpeed(0.0)
    , targetLane(LN_UNDEFINED)
    , laneIsBlockedLeft(false)
    , laneIsBlockedRight(false)
{};

bool sBehavior::LeadingVehicleAhead() const
{
    if (invalidVehicleId != leadingVehicle.id)
    {
        return true;
    }
    return false;
}

bool sBehavior::LeadingVehicleLeft() const
{
    if (invalidVehicleId != leadingVehicleLeft.id)
    {
        return true;
    }
    return false;
}

bool sBehavior::LeadingVehicleRight() const
{
    if (invalidVehicleId != leadingVehicleRight.id)
    {
        return true;
    }
    return false;
}

void sBehavior::Dump() const
{

    auto stateLaneLeft = laneIsBlockedLeft ? "BLOCKED" : "FREE   ";
    auto stateLaneRight = laneIsBlockedRight ? "BLOCKED" : "FREE   ";

    std::stringstream sstream;
    sstream
		<< "Behavior:\n"
        << "state:        " << ToString(currentState).c_str() << "\n"
        << "current lane: " << ToString(currentLane).c_str() << "\n"
        << "target lane:  " << ToString(targetLane).c_str() << "\n"
		<< "target speed: " << targetSpeed << " m/s, "
							<< targetSpeed / mphAsMs << " miles/h\n"
        << "left lane:    " << stateLaneLeft << " "
        << "right lane:   " << stateLaneRight << "\n";

    printf("%s", sstream.str().c_str());
}
