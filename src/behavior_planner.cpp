#include <algorithm>
#include <limits>
#include <math.h>
#include <vector>

#include "behavior_planner.h"
#include "lane_info.h"
#include "sensor_fusion.h"
#include "vehicle.h"


using std::chrono::duration;
using std::chrono::system_clock;
using std::milli;
using std::vector;


// adapt speed if distance to leading vehicle is below this threshold
constexpr double adaptSpeedDistanceThreshold(12.0);

constexpr double maxSpeedLaneChange(49 * mphAsMs); // meters per second

constexpr double penaltyLaneChange = 10;
constexpr double penaltyNotOnMiddleLane = 20; // prefer middle lane

// penalty factors for speed or distance are only applied for
// vehicles within distance threshold
constexpr double penaltyDistanceThreshold = 100;
constexpr double penaltyFactorSpeed = 2.0; // speed diff to max speed
constexpr double penaltyFactorDistance = 2.0;

constexpr double penaltyCollision = 1000.0;

constexpr double criticalDistance = 5.0; // used for debug output

cBehaviorPlanner::cBehaviorPlanner()
    : m_sensorFusion(nullptr)
{
    m_currentBehavior.currentState = ES_LANE_KEEP;
    m_successorStatesLaneLeft = 
        { ES_LANE_KEEP, ES_LANE_CHANGE_RIGHT };
    m_successorStatesLaneMiddle = 
        { ES_LANE_KEEP, ES_LANE_CHANGE_LEFT, ES_LANE_CHANGE_RIGHT };
    m_successorStatesLaneRight =
        { ES_LANE_KEEP, ES_LANE_CHANGE_LEFT};
}

void cBehaviorPlanner::Init(std::shared_ptr<cSensorFusion>& sensorFusion)
{
    m_sensorFusion = sensorFusion;
}

const sBehavior& cBehaviorPlanner::Execute(const sEgo& ego)
{
    const auto& successorStates = PossibleSuccessorStates(ego);
    auto possibleBehaviors = PossibleBehaviors(ego, m_sensorFusion, successorStates);
    CheckNeighborLanes(ego, possibleBehaviors);
    auto plannedBehavior = LowestCostBehavior(ego, possibleBehaviors);
    DetermineTargetSpeed(plannedBehavior);

    if (m_currentBehavior.currentState != plannedBehavior.currentState)
    {
        if (    ES_LANE_CHANGE_LEFT == plannedBehavior.currentState
            ||  ES_LANE_CHANGE_RIGHT == plannedBehavior.currentState)
        {
			//printf("%s: initializing %s\n",
			//	__FUNCTION__, ToString(plannedBehavior.currentState).c_str());
        }
        else if (ES_LANE_KEEP == plannedBehavior.currentState)
        {
			//printf("%s: finished %s, initializing %s\n",
			//	__FUNCTION__,
			//	ToString(m_currentBehavior.currentState).c_str(),
			//	ToString(plannedBehavior.currentState).c_str());
        }
    }

    if (    plannedBehavior.LeadingVehicleAhead()
        &&  plannedBehavior.leadingVehicle.sDistanceEgo < criticalDistance)
    {
		//printf("%s: distance to leading vehicle=%.3f - dumping sensor fusion state\n",
		//	__FUNCTION__, plannedBehavior.leadingVehicle.sDistanceEgo);
		//m_sensorFusion->DumpState();
		//plannedBehavior.Dump();
    }

    m_currentBehavior = plannedBehavior;

    if (    m_currentBehavior.adaptSpeedToLeadingVehicle
        &&  IncreaseDistanceToLeadingVehicle(ego))
    {
        m_currentBehavior.targetSpeed -= maxAccel * cycleTime;
		//printf("%s: increasing distance to leading vehicle, targetV=%.3f\n",
		//	__FUNCTION__, m_currentBehavior.targetSpeed);
    }

    return m_currentBehavior;
}

bool cBehaviorPlanner::IncreaseDistanceToLeadingVehicle(const sEgo& ego)
{
	//switch (ego.LaneName())
	//{
	//case LN_LANE_LEFT:
	//	break;
	//case LN_LANE_MIDDLE:
	//	break;
	//case LN_LANE_RIGHT:
	//	break;
	//}

    return false;
}

void DetermineTargetSpeed(sBehavior& behavior)
{
    if (ES_LANE_KEEP == behavior.currentState)
    {
        if (    behavior.LeadingVehicleAhead() 
            &&  behavior.leadingVehicle.sDistanceEgo < adaptSpeedDistanceThreshold)
        {
            //printf("%s: adapt to leading vehicle speed=%.3f\n",
            //    __FUNCTION__, behavior.leadingVehicle.v);
            behavior.targetSpeed = behavior.leadingVehicle.v;
            behavior.adaptSpeedToLeadingVehicle = true;
        }
        else
        {
            behavior.targetSpeed = maxSpeed;
            behavior.adaptSpeedToLeadingVehicle = false;
        }
    }
    else if (ES_LANE_CHANGE_LEFT == behavior.currentState)
    {
        // leadingVehicle refers to target lane
        // leadingVehicleRight refers to original lane
        auto v = maxSpeedLaneChange;
        if (    behavior.LeadingVehicleAhead()
            &&  behavior.leadingVehicle.sDistanceEgo < adaptSpeedDistanceThreshold)
        {
            v = std::min(v, behavior.leadingVehicle.v);
            //printf("%s: ES_LANE_CHANGE_LEFT: adapted speed to left lane: %.3f\n",
            //    __FUNCTION__, v);
        }
        if (    behavior.LeadingVehicleRight()
            &&  behavior.leadingVehicleRight.sDistanceEgo < adaptSpeedDistanceThreshold)
        {
            v = std::min(v, behavior.leadingVehicleRight.v);
            //printf("%s: ES_LANE_CHANGE_LEFT: adapted speed to original lane: %.3f\n",
            //    __FUNCTION__, v);
        }
        behavior.targetSpeed = v;
    }
    else if (ES_LANE_CHANGE_RIGHT == behavior.currentState)
    {
        // leadingVehicle refers to target lane
        // leadingVehicleLeft refers to original lane
        auto v = maxSpeedLaneChange;
        if (    behavior.LeadingVehicleAhead()
            &&  behavior.leadingVehicle.sDistanceEgo < adaptSpeedDistanceThreshold)
        {
            v = std::min(v, behavior.leadingVehicle.v);
            //printf("%s: ES_LANE_CHANGE_RIGHT: adapted speed to right lane: %.3f\n",
            //    __FUNCTION__, v);
        }
        if (    behavior.LeadingVehicleLeft()
            &&  behavior.leadingVehicleLeft.sDistanceEgo < adaptSpeedDistanceThreshold)
        {
            v = std::min(v, behavior.leadingVehicleLeft.v);
            //printf("%s: ES_LANE_CHANGE_LEFT: adapted speed to original lane: %.3f\n",
            //    __FUNCTION__, v);
        }
        behavior.targetSpeed = v;
    }
    else
    {
        //printf("%s: unable to determine target speed!\n", __FUNCTION__);
    }
}

void cBehaviorPlanner::CheckNeighborLanes(const sEgo& ego, std::vector<sBehavior>& behaviors)
{
    for (auto& behavior : behaviors)
    {
        m_sensorFusion->CheckNeighborLanes(ego, behavior);
    }
}

const std::vector<eEgoState>& cBehaviorPlanner::PossibleSuccessorStates(
    const sEgo& ego) const
{
    switch (ego.LaneName())
    {
    case LN_LANE_LEFT:
        return m_successorStatesLaneLeft;
        break;
    case LN_LANE_MIDDLE:
        return m_successorStatesLaneMiddle;
        break;
    case LN_LANE_RIGHT:
        return m_successorStatesLaneRight;
        break;
    case LN_UNDEFINED:
    	return m_successorStatesEmpty;
    	break;
    }

    return m_successorStatesEmpty;
}

std::vector<sBehavior> PossibleBehaviors(
    const sEgo& ego,
    const std::shared_ptr<cSensorFusion>& sensorFusion,
    const std::vector<eEgoState>& successorStates)
{
    std::vector<sBehavior> behaviors;

    for (const auto state : successorStates)
    {
        auto targetLane = ego.LaneName();

        switch (state)
        {
            case ES_LANE_KEEP:
            {
                targetLane = ego.LaneName();
                break;
            }
            case ES_LANE_CHANGE_LEFT:
            {
                targetLane = LeftLaneOf(ego.LaneName());
                break;
            }
            case ES_LANE_CHANGE_RIGHT:
            {
                targetLane = RightLaneOf(ego.LaneName());
                break;
            }
        }

        sBehavior behavior;
        behavior.currentState = state;
        behavior.currentLane = ego.LaneName();
        sensorFusion->InitBehavior(targetLane, behavior);
        behaviors.push_back(behavior);
    }

    return behaviors;
}

sBehavior LowestCostBehavior(
    const sEgo& ego, 
    const std::vector<sBehavior>& possibleBehaviors)
{
    if (possibleBehaviors.empty())
    {
        throw std::invalid_argument("list of possible behaviors is empty!");
    }

    vector<double> costs;
    for (const auto behavior : possibleBehaviors)
    {
        auto cost = 0.0;
        cost += PenaltyLaneChange(behavior);
        cost += PenaltyNotOnMiddleLane(behavior);
        cost += PenaltySpeed(behavior);
        cost += PenaltyDistance(behavior);
        cost += PenaltyCollision(behavior);
        costs.push_back(cost);

        //printf("%s: cost=%.3f, behavior=%s\n", 
        //    __FUNCTION__, cost, ToString(behavior).c_str());
    }

    double minCost = doubleMax;
    size_t minCostIdx = 0;
    for (size_t i = 0; i < costs.size(); ++i)
    {
        if (costs[i] < minCost)
        {
            minCost = costs[i];
            minCostIdx = i;
        }
    }

    //printf("%s: selected: cost=%.3f, behavior=%s\n", __FUNCTION__, 
    //    costs[minCostIdx], ToString(possibleBehaviors[minCostIdx]).c_str());
    return possibleBehaviors[minCostIdx];
}

double PenaltyLaneChange(const sBehavior& behavior)
{
    if (ES_LANE_KEEP == behavior.currentState)
    {
        return 0;
    }

    //printf("%s\n", __FUNCTION__);
    return penaltyLaneChange;
}

double PenaltyNotOnMiddleLane(const sBehavior& behavior)
{
    if (LN_LANE_MIDDLE == behavior.targetLane)
    {
        return 0.0;
    }

    //printf("%s\n", __FUNCTION__);
    return penaltyNotOnMiddleLane;
}

double PenaltySpeed(const sBehavior& behavior)
{
    if (    !behavior.LeadingVehicleAhead()
        ||  penaltyDistanceThreshold < behavior.leadingVehicle.sDistanceEgo)
    {
        return 0.0;
    }

    auto penalty = penaltyFactorSpeed * (maxSpeed - behavior.leadingVehicle.v);
    //printf("%s: %.3f\n", __FUNCTION__, penalty);

    return penalty;
}

double PenaltyDistance(const sBehavior& behavior)
{
    if (    !behavior.LeadingVehicleAhead()
        ||  penaltyDistanceThreshold < behavior.leadingVehicle.sDistanceEgo)
    {
        return 0.0;
    }

    auto penalty = penaltyFactorDistance 
        * (penaltyDistanceThreshold - behavior.leadingVehicle.sDistanceEgo);
    //printf("%s: %.3f\n", __FUNCTION__, penalty);

    return penalty;
}

double PenaltyCollision(const sBehavior& behavior)
{
    if (        ((ES_LANE_CHANGE_LEFT == behavior.currentState)
            &&  behavior.laneIsBlockedLeft)
        ||      ((ES_LANE_CHANGE_RIGHT == behavior.currentState)
            &&  behavior.laneIsBlockedRight))
    {
        //printf("%s: adding collision penalty\n", __FUNCTION__);

        return penaltyCollision;
    }

    return 0.0;
}
