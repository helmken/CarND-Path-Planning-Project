#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <string>

#include "constants.h"
#include "vehicle.h"


struct sBehavior
{
    // current state: keep lane, change left/right
    eEgoState currentState;
    eLaneName currentLane;

    // properties of current state
    bool adaptSpeedToLeadingVehicle;
    double targetSpeed;
    eLaneName targetLane;

    sVehicle leadingVehicle;
    sVehicle leadingVehicleLeft;
    sVehicle leadingVehicleRight;

    sVehicle followingVehicle;
    sVehicle followingVehicleLeft;
    sVehicle followingVehicleRight;

    // left lane is blocked by another vehicle
    bool laneIsBlockedLeft;

    // right lane is blocked by another vehicle
    bool laneIsBlockedRight;

    sBehavior();

    bool LeadingVehicleAhead() const;
    bool LeadingVehicleLeft() const;
    bool LeadingVehicleRight() const;

    void Dump() const;
};

#endif // BEHAVIOR_H
