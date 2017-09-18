#include <limits>
#include <stdexcept>
#include <vector>

#include "road_situation.h"
#include "dynamic_object.h"
#include "ego.h"


using namespace std;


cRoadSituation::cRoadSituation()
    : laneLeft(LN_LANE_LEFT)
    , laneMiddle(LN_LANE_MIDDLE)
    , laneRight(LN_LANE_RIGHT)
{};

const sLaneInfo& cRoadSituation::GetLaneInfo(eLaneName laneName) const
{
    switch (laneName)
    {
    case LN_LANE_LEFT:
    {
        return laneLeft;
    }
    case LN_LANE_MIDDLE:
    {
        return laneMiddle;
    }
    case LN_LANE_RIGHT:
    {
        return laneRight;
    }
    default:
    {
        throw std::invalid_argument("unexpected lane name");
    }
    }
}

void cRoadSituation::AnalyzeRoadSituation(
    const std::vector<sDynamicObject>& vehicles,
    const double egoS)
{
    SortVehiclesByLane(vehicles);
    FindLeadingVehiclesInLanes(egoS);
}

void cRoadSituation::SortVehiclesByLane(
    const std::vector<sDynamicObject>& dynamicObjects)
{
    for (vector<sDynamicObject>::const_iterator iter = dynamicObjects.begin();
        iter != dynamicObjects.end(); ++iter)
    {
        const double d = (*iter).d;
        if (laneLeft.IsWithinLaneBoundaries(d))
        {
            laneLeft.AddVehicle(*iter);
        }
        else if (laneMiddle.IsWithinLaneBoundaries(d))
        {
            laneMiddle.AddVehicle(*iter);
        }
        if (laneRight.IsWithinLaneBoundaries(d))
        {
            laneRight.AddVehicle(*iter);
        }
    }
}

void cRoadSituation::FindLeadingVehiclesInLanes(const double egoS)
{
    laneLeft.FindLeadingVehicleInLane(egoS);
    laneMiddle.FindLeadingVehicleInLane(egoS);
    laneRight.FindLeadingVehicleInLane(egoS);
}

eLaneName cRoadSituation::GetOptimalLaneForLaneChange(
    const sEgo& ego) const
{
    // prefer ego lane in case of equal distance
    const eLaneName egoLane = ego.GetCurrentLaneName();
    eLaneName resultLane = egoLane;
    double distanceLeadingVehicle = GetLaneInfo(ego.GetCurrentLaneName())
        .GetDistanceToLeadingVehicle();

    if (    LN_LANE_LEFT != egoLane 
        &&  laneLeft.GetDistanceToLeadingVehicle() > distanceLeadingVehicle)
    {
        distanceLeadingVehicle = laneLeft.GetDistanceToLeadingVehicle();
        resultLane = LN_LANE_LEFT;
    }

    if (    LN_LANE_MIDDLE != egoLane
        &&  laneMiddle.GetDistanceToLeadingVehicle() > distanceLeadingVehicle)
    {
        distanceLeadingVehicle = laneMiddle.GetDistanceToLeadingVehicle();
        resultLane = LN_LANE_MIDDLE;
    }

    if (    LN_LANE_RIGHT != egoLane
        &&  laneRight.GetDistanceToLeadingVehicle() > distanceLeadingVehicle)
    {
        distanceLeadingVehicle = laneRight.GetDistanceToLeadingVehicle();
        resultLane = LN_LANE_RIGHT;
    }

    return resultLane;
}

//eLaneChangeDirection cRoadSituation::SelectLaneChangeDirection(
//    const sEgo& ego)
//{
//    const eLaneName egoLane = ego.GetCurrentLaneName();
//
//    switch (egoLane)
//    {
//    case LN_LANE_LEFT:
//    {
//        if (!roadInfo.laneMiddle.leadingVehicleAhead
//            || (roadInfo.laneMiddle.distanceToLeadingVehicle
//                 >  roadInfo.laneLeft.distanceToLeadingVehicle))
//        {
//            return LCD_RIGHT;
//        }
//    }
//    break;
//    case LN_LANE_MIDDLE:
//    {
//        if (!roadInfo.laneLeft.leadingVehicleAhead)
//        {
//            return LCD_LEFT;
//        }
//        else if (!roadInfo.laneRight.leadingVehicleAhead)
//        {
//            return LCD_RIGHT;
//        }
//        else if (roadInfo.laneLeft.distanceToLeadingVehicle
//                 >  roadInfo.laneRight.distanceToLeadingVehicle)
//        {
//            return LCD_LEFT;
//        }
//        else if (roadInfo.laneRight.distanceToLeadingVehicle
//                 >  roadInfo.laneLeft.distanceToLeadingVehicle)
//        {
//            return LCD_RIGHT;
//        }
//    }
//    break;
//    case LN_LANE_RIGHT:
//    {
//        if (!roadInfo.laneMiddle.leadingVehicleAhead
//            || (roadInfo.laneMiddle.distanceToLeadingVehicle
//                 >  roadInfo.laneRight.distanceToLeadingVehicle))
//        {
//            return LCD_LEFT;
//        }
//    }
//    break;
//    }
//
//    return LCD_STRAIGHT;
//}
