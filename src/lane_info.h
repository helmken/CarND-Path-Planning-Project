#ifndef LANE_INFO_H
#define LANE_INFO_H


#include <vector>
#include "dynamic_object.h"


const double laneWidth(4.0);

enum eLaneName
{
    LN_UNDEFINED = -1,
    LN_LANE_LEFT,
    LN_LANE_MIDDLE,
    LN_LANE_RIGHT
};

enum eLaneChangeDirection
{
    LCD_LEFT,
    LCD_STRAIGHT,
    LCD_RIGHT
};

class sLaneInfo
{
    eLaneName laneName;

    double boundaryLeft;
    double boundaryRight;

    bool leadingVehicleAhead;
    sDynamicObject leadingVehicle;
    double distanceToLeadingVehicle;

    std::vector<sDynamicObject> dynamicObjects;

public:
    sLaneInfo(const eLaneName laneName);

    bool IsWithinLaneBoundaries(const double d);

    void FindLeadingVehicleInLane(
        const double egoS);

    bool IsLeadingVehicleAhead() const;

    bool IsDistanceToLeadingVehicleLarger(const double distance) const;

    double GetDistanceToLeadingVehicle() const;

    eLaneName GetLaneName() const;

    double GetSpeedOfLeadingVehicle() const;

    int GetLeadingVehicleId() const;

    void AddVehicle(const sDynamicObject& vehicle);
};

#endif // LANE_INFO_H
