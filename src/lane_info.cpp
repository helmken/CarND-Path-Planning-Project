#include <limits>

#include "lane_info.h"


using namespace std;


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

bool sLaneInfo::IsWithinLaneBoundaries(const double d)
{
    if (d >= boundaryLeft && d < boundaryRight)
    {
        return true;
    }
    return false;
};

void sLaneInfo::AddVehicle(const sDynamicObject& vehicle)
{
    dynamicObjects.push_back(vehicle);
}

void sLaneInfo::FindLeadingVehicleInLane(
    const double egoS)
{
    double closestS = numeric_limits<double>::max();

    size_t closestIdx = dynamicObjects.size() + 1;

    for (size_t i(0); i < dynamicObjects.size(); ++i)
    {
        double objS = dynamicObjects[i].s;
        if (objS > egoS && objS < closestS)
        {
            closestS = objS;
            closestIdx = i;
        }
    }

    if (closestIdx < dynamicObjects.size())
    {
        leadingVehicleAhead = true;
        leadingVehicle = dynamicObjects[closestIdx];
        distanceToLeadingVehicle = closestS - egoS;
    }
    else
    {
        leadingVehicleAhead = false;
    }
}

bool sLaneInfo::IsLeadingVehicleAhead() const
{
    return leadingVehicleAhead;
}

bool sLaneInfo::IsDistanceToLeadingVehicleLarger(const double distance) const
{
    if (!leadingVehicleAhead)
    {
        return true;
    }
    if (distanceToLeadingVehicle > distance)
    {
        return true;
    }

    return false;
}

double sLaneInfo::GetDistanceToLeadingVehicle() const
{
    if (!leadingVehicleAhead)
    {
        throw std::invalid_argument("distance to leading vehicle is invalid");
    }

    return distanceToLeadingVehicle;
}

eLaneName sLaneInfo::GetLaneName() const
{
    return laneName;
}

double sLaneInfo::GetSpeedOfLeadingVehicle() const
{
    if (leadingVehicleAhead)
    {
        return leadingVehicle.GetSpeed();
    }

    throw std::logic_error("not leading vehicle - not possible to determine speed");
}

int sLaneInfo::GetLeadingVehicleId() const
{
    if (leadingVehicleAhead)
    {
        return leadingVehicle.id;
    }

    throw std::logic_error("not leading vehicle - not possible to determine ID");
}
