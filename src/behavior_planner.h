#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H


#include <vector>

#include "current_situation.h"
#include "dynamic_object.h"
#include "ego.h"
#include "path.h"


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


// maximum allowed speed
//const double maxSpeed(49.9);  // this is in miles per hour
const double maxSpeed(5.0); // meters per second

const double distanceKeepLane(30.0);

const int invalidVehicleId(-1);


/*
Behavior description as suggested in Lesson 4.2 
"Behavior Planning - Understanding Output" 
*/
struct sBehavior
{
    eLaneName targetLane;

    /*
    ID of vehicle to follow: 
    - if ID is valid, ego speed has to be adapted to target vehicle speed
    - if ID is invalid, ego can accelerate to maximum speed
    */
    int targetLeadingVehicleId;

    /*
    if target vehicle ID is invalid, adapt ego speed to target speed
    */
    double targetSpeed;

    /*
    duration to reach desired behavior
    */
    double secondsToReachTarget;

    sBehavior()
        : targetLane(LN_UNDEFINED)
        , targetLeadingVehicleId(-1)
        , targetSpeed(-0.0)
        , secondsToReachTarget(-1.0)
    {};

    sBehavior(
        eLaneName targetLane,
        int targetLeadingVehicleId,
        double targetSpeed,
        double secondsToReachTarget)
        : targetLane(targetLane)
        , targetLeadingVehicleId(targetLeadingVehicleId)
        , targetSpeed(targetSpeed)
        , secondsToReachTarget(secondsToReachTarget)
    {};
};

std::string ToString(const sBehavior& behavior);


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
    sBehavior Execute(
        const sEgo& ego,
        const std::vector<sDynamicObject>& vehicles);

private:
    eEgoState m_egoState;
    cTrajectoryPlanner* m_trajectoryPlanner;
};

double CalculateReferenceSpeed(
    const std::vector<sDynamicObject>& dynamicObjects,
    int& egoLane,
    const sEgo& ego);

void AnalyzeRoadSituation(
    const std::vector<sDynamicObject>& vehicles,
    const double egoS,
    sCurrentSituation& currentSituation);

void SortDynamicObjectsByLane(
    const std::vector<sDynamicObject>& dynamicObjects,
    sCurrentSituation& roadInfo);

void FindLeadingDynamicObjectInLane(
    const double egoS,
    sLaneInfo& laneInfo);

bool StayOnCurrentLaneAndAccelerateToMaxSpeed(
    const sCurrentSituation& roadInfo,
    const sEgo& ego,
    sBehavior& plannedBehavior);

bool StayOnCurrentLaneAndAccelerateToMaxSpeed(
    const sLaneInfo& laneInfo,
    sBehavior& plannedBehavior);

bool StayOnCurrentLaneAndAdaptSpeed(
    const sCurrentSituation& roadInfo,
    const sEgo& ego,
    sBehavior& plannedBehavior);

void AdaptToLeadingVehicleInLane(
    const sLaneInfo& currentLane, 
    sBehavior& behavior);

eLaneChangeDirection SelectLaneChangeDirection(
    const sCurrentSituation& roadInfo,
    const sEgo& ego);

int GetLaneIdxFromLaneChangeDirection(
    const eLaneChangeDirection laneChangeDir,
    const sEgo& ego);

eLaneName GetLaneNameFromLaneChangeDirection(
    const eLaneChangeDirection laneChangeDir,
    const sEgo& ego);

int LaneNameToLaneIdx(eLaneName laneName);

int GetLeftLaneIdx(eLaneName laneName);

eLaneName GetLeftLaneName(eLaneName laneName);

int GetRightLaneIdx(eLaneName laneName);

eLaneName GetRightLaneName(eLaneName laneName);

#endif // BEHAVIOR_PLANNER_H