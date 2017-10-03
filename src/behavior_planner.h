#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H


#include <vector>

#include "road_situation.h"
#include "dynamic_object.h"
#include "ego.h"
#include "path.h"


// general tasks that will be handled by path planner 
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

// 50 MPH = 22.352 m/s
//        = 80.467 km/h
const double maxSpeed(22.3); // meters per second

const double thresholdKeepLane(30.0);

const int invalidVehicleId(-1);


/*
Behavior description as suggested in Lesson 4.2 
"Behavior Planning - Understanding Output" 
*/
struct sBehavior
{
    eLaneName targetLane;

    /*
    ID of vehicle to follow: if ID is valid, ego speed has to be adapted to
    target vehicle speed, otherwise ego can accelerate to maximum speed
    */
    int leadingVehicleId;

    /*
    desired speed at target position
    */
    double speedAtTargetPosition;

    /*
    distance to target position
    */
    double distanceToTargetPosition;

    /*
    duration to reach desired position
    */
    double timeToTargetPosition;

    sBehavior()
        : targetLane(LN_UNDEFINED)
        , leadingVehicleId(-1)
        , speedAtTargetPosition(-1.0)
        , distanceToTargetPosition(-1.0)
        , timeToTargetPosition(-1.0)
    {};

    sBehavior(
        eLaneName lane,
        int id,
        double speed,
        double distance,
        double time)
        : targetLane(lane)
        , leadingVehicleId(id)
        , speedAtTargetPosition(speed)
        , distanceToTargetPosition(distance)
        , timeToTargetPosition(time)
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

bool StayOnCurrentLane(
    const sEgo& ego,
    const cRoadSituation& roadInfo
);

bool AccelerateToMaxSpeed(
    const sLaneInfo& laneInfo,
    sBehavior& plannedBehavior);

void AdaptSpeedToLeadingVehicle(
    const sLaneInfo& laneInfo,
    sBehavior& plannedBehavior);

int LaneNameToLaneIdx(eLaneName laneName);

int GetLeftLaneIdx(eLaneName laneName);

eLaneName GetLeftLaneName(eLaneName laneName);

int GetRightLaneIdx(eLaneName laneName);

eLaneName GetRightLaneName(eLaneName laneName);

#endif // BEHAVIOR_PLANNER_H