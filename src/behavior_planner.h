#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <chrono>
#include <memory>
#include <vector>

#include "behavior.h"
#include "ego.h"


// general tasks that will be handled by path planner 
// - drive at target speed 
// - maintain safety distance
// - avoid collisions


class cSensorFusion;
struct sVehicle;

class cBehaviorPlanner
{
public:
    cBehaviorPlanner();

    void Init(std::shared_ptr<cSensorFusion>& sensorFusion);

    /*
     * Select suitable behavior based on input from sensor fusion.
     */
    const sBehavior& Execute(const sEgo& ego);

private:
    void CheckNeighborLanes(const sEgo& ego, std::vector<sBehavior>& behaviors);
    const std::vector<eEgoState>& PossibleSuccessorStates(
        const sEgo& ego) const;
    bool IncreaseDistanceToLeadingVehicle(const sEgo& ego);

    std::shared_ptr<cSensorFusion> m_sensorFusion;

    sBehavior m_currentBehavior;

    std::vector<eEgoState> m_successorStatesLaneLeft;
    std::vector<eEgoState> m_successorStatesLaneMiddle;
    std::vector<eEgoState> m_successorStatesLaneRight;
    std::vector<eEgoState> m_successorStatesEmpty;
};

void DetermineTargetSpeed(sBehavior& behavior);

std::vector<sBehavior> PossibleBehaviors(
    const sEgo& ego,
    const std::shared_ptr<cSensorFusion>& roadSituation,
    const std::vector<eEgoState>& successorStates);

sBehavior LowestCostBehavior(
    const sEgo& ego, 
    const std::vector<sBehavior>& possibleBehaviors);

double PenaltyLaneChange(
    const sBehavior& behavior);

double PenaltyNotOnMiddleLane(
    const sBehavior& behavior);

double PenaltySpeed(
    const sBehavior& behavior);

double PenaltyDistance(
    const sBehavior& behavior);

double PenaltyCollision(const sBehavior& behavior);


#endif // BEHAVIOR_PLANNER_H
