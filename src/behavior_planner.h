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


// general tasks that will be handled by trajectory planner 
// - drive at target speed 
// - maintain safety distance
// - avoid collisions
enum eEgoState
{
    ES_LANE_KEEP, // stay close to lane center, drive at target speed
    ES_PREPARE_LANE_CHANGE_LEFT, // adapt to speed and position on left lane
    ES_LANE_CHANGE_LEFT, // move to left lane
    ES_PREPARE_LANE_CHANGE_RIGHT, // adapt to speed and position on right lane
    ES_LANE_CHANGE_RIGHT // move to right lane
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

class cTrajectoryPlanner;

class cBehaviorPlanner
{
public:
    cBehaviorPlanner();

    void Init(cTrajectoryPlanner* trajectoryPlanner);

    /**
     * Generate trajectories for each possible next state and select
     * the best possible next state based on a cost function.
     */
    void Execute(); // TODO: input params: map, route, predictions

private:
    eEgoState m_egoState;
    cTrajectoryPlanner* m_trajectoryPlanner;
};

double CalculateReferenceSpeed(
    const std::vector<sDynamicObject>& dynamicObjects,
    int& egoLane,
    const sEgo& ego);

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