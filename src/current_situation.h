#ifndef CURRENT_SITUATION_H
#define CURRENT_SITUATION_H

#include <vector>

#include "dynamic_object.h"
#include "ego.h"

struct sLaneInfo
{
    eLaneName laneName;

    double boundaryLeft;
    double boundaryRight;

    bool leadingVehicleAhead;
    sDynamicObject leadingDynamicObject;
    double distanceToLeadingVehicle;

    std::vector<sDynamicObject> dynamicObjects;

    sLaneInfo::sLaneInfo(const eLaneName laneName)
        : laneName(laneName)
        , leadingVehicleAhead(false)
    {
        switch (laneName)
        {
        case LN_LANE_LEFT:
            boundaryLeft = 0.0;
            boundaryRight = boundaryLeft + laneWidth;
            break;
        case LN_LANE_MIDDLE:
            boundaryLeft = laneWidth;
            boundaryRight = boundaryLeft + laneWidth;
            break;
        case LN_LANE_RIGHT:
            boundaryLeft = 2.0 * laneWidth;
            boundaryRight = boundaryLeft + laneWidth;
            break;
        }
    };

    bool IsWithinLaneBoundaries(const double d)
    {
        if (d >= boundaryLeft && d < boundaryRight)
        {
            return true;
        }
        return false;
    };
};

struct sCurrentSituation
{
    sLaneInfo laneLeft;
    sLaneInfo laneMiddle;
    sLaneInfo laneRight;

    sCurrentSituation()
        : laneLeft(LN_LANE_LEFT)
        , laneMiddle(LN_LANE_MIDDLE)
        , laneRight(LN_LANE_RIGHT)
    {};
};

#endif // CURRENT_SITUATION_H