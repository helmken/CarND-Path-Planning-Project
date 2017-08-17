#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <vector>

#include "dynamic_object.h"
#include "ego.h"


enum eLaneName 
{
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

// number of Lanes 
const int numLanes(3);

const double laneWidth(4.0);

struct sLaneInfo
{
    eLaneName laneName;

    double boundaryLeft;
    double boundaryRight;
    
    bool leadingDynamicObjectAhead;
    sDynamicObject leadingDynamicObject;
    double leadingDynamicObjectDistance;
    
    std::vector<sDynamicObject> dynamicObjects;

    sLaneInfo(const eLaneName laneName);

    bool IsWithinLaneBoundaries(const double d);
};

struct sRoadInfo
{
    sLaneInfo laneLeft;
    sLaneInfo laneMiddle;
    sLaneInfo laneRight;

    sRoadInfo()
        : laneLeft(LN_LANE_LEFT)
        , laneMiddle(LN_LANE_MIDDLE)
        , laneRight(LN_LANE_RIGHT)
    {};
};

double CalculateReferenceSpeed(
    const std::vector<sDynamicObject>& dynamicObjects,
    int& egoLane,
    const sEgo& ego,
    std::size_t prevPathSize);

void AnalyseRoadSituation(
    const std::vector<sDynamicObject>& dynamicObjects,
    const double egoS,
    sRoadInfo& roadInfo);

void SortDynamicObjectsByLane(
    const std::vector<sDynamicObject>& dynamicObjects,
    sRoadInfo& roadInfo);

void FindLeadingDynamicObjectInLane(
    const double egoS,
    sLaneInfo& laneInfo);

bool StayOnCurrentLane(
    const sRoadInfo& roadInfo, 
    const sEgo& ego);

eLaneChangeDirection SelectLaneChangeDirection(
    const sRoadInfo& roadInfo,
    const sEgo& ego);

eLaneName DToLaneName(const double d);

int GetLaneIdxFromLaneChangeDirection(
    const eLaneChangeDirection laneChangeDir,
    const sEgo& ego);

int LaneNameToLaneIdx(eLaneName laneName);

int GetLeftLaneIdx(eLaneName laneName);

int GetRightLaneIdx(eLaneName laneName);

#endif // BEHAVIOR_PLANNER_H