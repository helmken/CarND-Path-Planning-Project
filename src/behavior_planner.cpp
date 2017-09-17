#include <math.h>
#include <limits>
#include <vector>

#include "behavior_planner.h"
#include "trajectory_planner.h"


using namespace std;


// maximum allowed speed
const double maxSpeed(49.9);


const double distanceKeepLane(30.0);  


cBehaviorPlanner::cBehaviorPlanner()
    : m_egoState(ES_LANE_KEEP)
    , m_trajectoryPlanner(nullptr)
{

}

void cBehaviorPlanner::Init(cTrajectoryPlanner* trajectoryPlanner)
{
    m_trajectoryPlanner = trajectoryPlanner;
}

void cBehaviorPlanner::Execute() // TODO: input params: map, route, predictions
{
    //def transition_function(predictions, current_fsm_state, current_pose, cost_functions, weights) :    //    # only consider states which can be reached from current FSM state.    //    possible_successor_states = successor_states(current_fsm_state)            //    # keep track of the total cost of each state.    //    costs = []    //    for state in possible_successor_states :    //      # generate a rough idea of what trajectory we would    //      # follow IF we chose this state.    //      trajectory_for_state = generate_trajectory(state, current_pose, predictions)    //          //      # calculate the "cost" associated with that trajectory.    //      cost_for_state = 0    //      for i in range(len(cost_functions)) :    //        # apply each cost function to the generated trajectory    //        cost_function = cost_functions[i]    //        cost_for_cost_function = cost_function(trajectory_for_state, predictions)                            //    //        # multiply the cost by the associated weight    //        weight = weights[i]    //        cost_for_state += weight * cost_for_cost_function    //        costs.append({ 'state' : state, 'cost' : cost_for_state })            //    //        # Find the minimum cost state.    //        best_next_state = None    //        min_cost = 9999999    //        for i in range(len(possible_successor_states)) :    //            state = possible_successor_states[i]    //            cost = costs[i]    //            if cost < min_cost :    //                min_cost = cost    //                best_next_state = state     
    //  return best_next_state

    // TODO: use behavior planner from lesson 4.16
}

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
}


double CalculateReferenceSpeed(
    const std::vector<sDynamicObject>& dynamicObjects, 
    int& targetLane,
    const sEgo& ego)
{
    bool too_close = false;

    // to avoid hitting other cars: go through dynamic objects list and check
    // if another car is in our lane
    // if yes: check how close

    double referenceSpeed = 1.0; // mph

    sCurrentSituation roadInfo;
    AnalyseRoadSituation(
        dynamicObjects,
        ego.s,
        roadInfo);

    targetLane = 1;

    if (StayOnCurrentLane(roadInfo, ego))
    {
        // stay on lane and accelerate to max speed

        referenceSpeed = 5.0; // 5.0 results in a speed of approx. 50 mph
    }
    else
    {
        referenceSpeed = 5.0; // 5.0 results in a speed of approx. 50 mph

        const eLaneChangeDirection changeDir =
            SelectLaneChangeDirection(roadInfo, ego);

        targetLane = GetLaneIdxFromLaneChangeDirection(changeDir, ego);
    }

    //// code from walkthrough
    //// find reference speed to use
    //// i is index of other car on the road 
    //for (size_t i(0); i < dynamicObjects.size(); ++i)
    //{
    //    const sDynamicObject& dynObj = dynamicObjects[i];

    //    // check if car is in ego lane:
    //    // d is position of dynamic object on the road -> find out which lane
    //    double d = dynObj.d;

    //    // lane is our lane 
    //    if (d < (2 + laneWidth * egoLane + 2) && d >(2 + laneWidth * egoLane - 2))
    //    {
    //        // so the car is in our lane
    //        double vx = dynObj.vx;
    //        double vy = dynObj.vy;

    //        double dynObjSpeed = sqrt(vx * vx + vy * vy);
    //        double check_car_s = dynObj.s;

    //        // check_car_s can help us to predict where that car is in the future  
    //        // if using previous points can project s value out
    //        check_car_s += ((double)prevPathSize * 0.2 * dynObjSpeed); 

    //        // check s values greater than mine and s gap
    //        if ((check_car_s > ego.s) && (check_car_s - ego.s) < 30)
    //        {
    //            // check if our car is close to the other car -> if so, need to take action

    //            // do some logic here, lower reference velocity so we dont crash into the car in front of us,
    //            // could also flag to try to change lanes

    //            referenceSpeed = 29.5; // mph

    //            // lines below consider this flag and reduce speed
    //            too_close = true;
    //            if (egoLane > 0)
    //            {
    //                egoLane = 0; // set left lane as target lane
    //            }
    //        }
    //    }
    //}

    //if (too_close)
    //{
    //    referenceSpeed -= 0.224; // this is somehow related to decelerating with 5 m/s^2
    //}
    //else if (referenceSpeed < 49.5)
    //{
    //    referenceSpeed += 0.224;
    //}
    //// end of code from walkthrough

    return referenceSpeed;
}

void AnalyseRoadSituation(
    const std::vector<sDynamicObject>& dynamicObjects,
    const double egoS,
    sCurrentSituation& roadInfo)
{
    SortDynamicObjectsByLane(dynamicObjects, roadInfo);
    FindLeadingDynamicObjectInLane(egoS, roadInfo.laneLeft);
    FindLeadingDynamicObjectInLane(egoS, roadInfo.laneMiddle);
    FindLeadingDynamicObjectInLane(egoS, roadInfo.laneRight);
}

void SortDynamicObjectsByLane(
    const std::vector<sDynamicObject>& dynamicObjects,
    sCurrentSituation& roadInfo)
{
    for (vector<sDynamicObject>::const_iterator iter = dynamicObjects.begin();
        iter != dynamicObjects.end(); ++iter)
    {
        double d = (*iter).d;
        if (roadInfo.laneLeft.IsWithinLaneBoundaries(d))
        {
            roadInfo.laneLeft.dynamicObjects.push_back(*iter);
        }
        else if (roadInfo.laneMiddle.IsWithinLaneBoundaries(d))
        {
            roadInfo.laneMiddle.dynamicObjects.push_back(*iter);
        }
        if (roadInfo.laneRight.IsWithinLaneBoundaries(d))
        {
            roadInfo.laneRight.dynamicObjects.push_back(*iter);
        }
    }
}

void FindLeadingDynamicObjectInLane(
    const double egoS,
    sLaneInfo& laneInfo)
{
    double closestS = numeric_limits<double>::max();

    const vector<sDynamicObject>& dynamicObjects = laneInfo.dynamicObjects;
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
        laneInfo.leadingVehicleAhead = true;
        laneInfo.leadingDynamicObject = dynamicObjects[closestIdx];
        laneInfo.distanceToLeadingVehicle = closestS - egoS;
    }
    else
    {
        laneInfo.leadingVehicleAhead = false;
    }
}

bool StayOnCurrentLane(
    const sCurrentSituation& roadInfo,
    const sEgo& ego)
{
    const eLaneName egoLane = DToLaneName(ego.d);

    switch (egoLane)
    {
    case LN_LANE_LEFT:
        {
            if (    !roadInfo.laneLeft.leadingVehicleAhead
                ||  roadInfo.laneLeft.distanceToLeadingVehicle > distanceKeepLane)
            {
                return true;
            }
        }
        break;
    case LN_LANE_MIDDLE:
        if (!roadInfo.laneMiddle.leadingVehicleAhead
            || roadInfo.laneMiddle.distanceToLeadingVehicle > distanceKeepLane)
        {
            return true;
        }
        break;
    case LN_LANE_RIGHT:
        if (!roadInfo.laneMiddle.leadingVehicleAhead
            || roadInfo.laneMiddle.distanceToLeadingVehicle > distanceKeepLane)
        {
            return true;
        }
        break;
    }

    return false;
}

eLaneChangeDirection SelectLaneChangeDirection(
    const sCurrentSituation& roadInfo,
    const sEgo& ego)
{
    const eLaneName egoLane = DToLaneName(ego.d);

    switch (egoLane)
    {
    case LN_LANE_LEFT:
        {
            if (    !roadInfo.laneMiddle.leadingVehicleAhead
                ||      roadInfo.laneMiddle.distanceToLeadingVehicle
                    >   roadInfo.laneLeft.distanceToLeadingVehicle)
            {
                return LCD_RIGHT;
            }
        }
        break;
    case LN_LANE_MIDDLE:
        {
            if (!roadInfo.laneLeft.leadingVehicleAhead)
            {
                return LCD_LEFT;
            }
            else if (!roadInfo.laneRight.leadingVehicleAhead)
            {
                return LCD_RIGHT;
            }
            else if (   roadInfo.laneLeft.distanceToLeadingVehicle
                      > roadInfo.laneRight.distanceToLeadingVehicle)
            {
                return LCD_LEFT;
            }
            else if (   roadInfo.laneRight.distanceToLeadingVehicle
                     >  roadInfo.laneLeft.distanceToLeadingVehicle)
            {
                return LCD_RIGHT;
            }
        }
        break;
    case LN_LANE_RIGHT:
        {
            if (    !roadInfo.laneMiddle.leadingVehicleAhead
                ||      roadInfo.laneMiddle.distanceToLeadingVehicle
                    >   roadInfo.laneRight.distanceToLeadingVehicle)
            {
                return LCD_LEFT;
            }
        }
        break;
    }

    return LCD_STRAIGHT;
}

eLaneName DToLaneName(const double d)
{
    if (d >= 0.0 && d < laneWidth)
    {
        return LN_LANE_LEFT;
    }
    else if (d >= laneWidth && d < 2.0 * laneWidth)
    {
        return LN_LANE_MIDDLE;
    }
    if (d >= 2.0 * laneWidth && d < 3.0 * laneWidth)
    {
        return LN_LANE_LEFT;
    }

    return LN_LANE_MIDDLE;
}

int GetLaneIdxFromLaneChangeDirection(
    const eLaneChangeDirection laneChangeDir,
    const sEgo& ego)
{
    const eLaneName egoLane = DToLaneName(ego.d);
    if (LCD_STRAIGHT == laneChangeDir)
    {
        return LaneNameToLaneIdx(egoLane);
    }

    switch (laneChangeDir)
    {
    case LCD_LEFT:
        return GetLeftLaneIdx(egoLane);
        break;
    case LCD_RIGHT:
        return GetRightLaneIdx(egoLane);
        break;
    }

    return 1;
}

int LaneNameToLaneIdx(eLaneName laneName)
{
    switch (laneName)
    {
    case LN_LANE_LEFT:
        return 0;
        break;
    case LN_LANE_MIDDLE:
        return 1;
        break;
    case LN_LANE_RIGHT:
        return 2;
        break;
    }

    return 1;
}

int GetLeftLaneIdx(eLaneName laneName)
{
    if (LN_LANE_MIDDLE == laneName)
    {
        return 0;
    }
    else if (LN_LANE_RIGHT == laneName)
    {
        return 1;
    }

    return 1;
}

int GetRightLaneIdx(eLaneName laneName)
{
    if (LN_LANE_MIDDLE == laneName)
    {
        return 2;
    }
    else if (LN_LANE_LEFT == laneName)
    {
        return 1;
    }

    return 1;
}
