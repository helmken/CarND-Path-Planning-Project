#include <math.h>
#include <limits>
#include <vector>

#include "behavior_planner.h"
#include "trajectory_planner.h"


using namespace std;


cBehaviorPlanner::cBehaviorPlanner()
    : m_egoState(ES_LANE_KEEP)
    , m_trajectoryPlanner(nullptr)
{
}

void cBehaviorPlanner::Init(cTrajectoryPlanner* trajectoryPlanner)
{
    m_trajectoryPlanner = trajectoryPlanner;
}

sBehavior cBehaviorPlanner::Execute(
    const sEgo& ego,
    const std::vector<sDynamicObject>& vehicles)
{
    //def transition_function(predictions, current_fsm_state, current_pose, cost_functions, weights) :    //    # only consider states which can be reached from current FSM state.    //    possible_successor_states = successor_states(current_fsm_state)            //    # keep track of the total cost of each state.    //    costs = []    //    for state in possible_successor_states :    //      # generate a rough idea of what trajectory we would    //      # follow IF we chose this state.    //      trajectory_for_state = generate_trajectory(state, current_pose, predictions)    //          //      # calculate the "cost" associated with that trajectory.    //      cost_for_state = 0    //      for i in range(len(cost_functions)) :    //        # apply each cost function to the generated trajectory    //        cost_function = cost_functions[i]    //        cost_for_cost_function = cost_function(trajectory_for_state, predictions)                            //    //        # multiply the cost by the associated weight    //        weight = weights[i]    //        cost_for_state += weight * cost_for_cost_function    //        costs.append({ 'state' : state, 'cost' : cost_for_state })            //    //        # Find the minimum cost state.    //        best_next_state = None    //        min_cost = 9999999    //        for i in range(len(possible_successor_states)) :    //            state = possible_successor_states[i]    //            cost = costs[i]    //            if cost < min_cost :    //                min_cost = cost    //                best_next_state = state     
    //  return best_next_state

    // TODO: use behavior planner from lesson 4.16

    cRoadSituation roadSituation;
    roadSituation.AnalyzeRoadSituation(vehicles, ego.s);

    sBehavior plannedBehavior;
    if (StayOnCurrentLane(ego, roadSituation))
    {
        //printf("StayOnCurrentLane\n");
        const sLaneInfo& laneInfo = roadSituation.GetLaneInfo(ego.GetCurrentLaneName());

        if (AccelerateToMaxSpeed(laneInfo, plannedBehavior))
        {
            return plannedBehavior;
        }

        AdaptSpeedToLeadingVehicle(laneInfo, plannedBehavior);

        return plannedBehavior;
    }

    eLaneName targetLane = roadSituation.GetOptimalLaneForLaneChange(ego);
    const sLaneInfo& targetLaneInfo = roadSituation.GetLaneInfo(targetLane);

    if (AccelerateToMaxSpeed(targetLaneInfo, plannedBehavior))
    {
        return plannedBehavior;
    }

    AdaptSpeedToLeadingVehicle(targetLaneInfo, plannedBehavior);

    return plannedBehavior;
}

bool AccelerateToMaxSpeed(
    const sLaneInfo& laneInfo,
    sBehavior& plannedBehavior)
{
    if (!laneInfo.IsLeadingVehicleAhead()
        || laneInfo.IsDistanceToLeadingVehicleLarger(thresholdKeepLane))
    {
        plannedBehavior.targetLane = laneInfo.GetLaneName();
        plannedBehavior.targetSpeed = maxSpeed;
        plannedBehavior.secondsToReachTarget = 0.0;
        plannedBehavior.targetLeadingVehicleId = invalidVehicleId;

        return true;
    }

    return false;
}

void AdaptSpeedToLeadingVehicle(
    const sLaneInfo& laneInfo,
    sBehavior& plannedBehavior)
{
    plannedBehavior.targetLane = laneInfo.GetLaneName();
    plannedBehavior.targetSpeed = laneInfo.GetSpeedOfLeadingVehicle();
    plannedBehavior.secondsToReachTarget = 0.0;
    plannedBehavior.targetLeadingVehicleId = laneInfo.GetLeadingVehicleId();
}

//double CalculateReferenceSpeed(
//    const std::vector<sDynamicObject>& dynamicObjects,
//    int& targetLane,
//    const sEgo& ego)
//{
//    bool too_close = false;
//
//    // to avoid hitting other cars: go through dynamic objects list and check
//    // if another car is in our lane
//    // if yes: check how close
//
//    double referenceSpeed = 1.0; // mph
//
//    cRoadSituation roadInfo;
//    AnalyzeRoadSituation(
//        dynamicObjects,
//        ego.s,
//        roadInfo);
//
//    targetLane = 1;
//
//    sBehavior plannedBehavior;
//    if (StayOnCurrentLaneAndAccelerateToMaxSpeed(roadInfo, ego, plannedBehavior))
//    {
//        // stay on lane and accelerate to max speed
//
//        referenceSpeed = 5.0; // 5.0 results in a speed of approx. 50 mph
//    }
//    else
//    {
//        referenceSpeed = 5.0; // 5.0 results in a speed of approx. 50 mph
//
//        const eLaneChangeDirection changeDir =
//            SelectLaneChangeDirection(roadInfo, ego);
//
//        targetLane = GetLaneIdxFromLaneChangeDirection(changeDir, ego);
//    }
//
//    //// code from walkthrough
//    //// find reference speed to use
//    //// i is index of other car on the road 
//    //for (size_t i(0); i < dynamicObjects.size(); ++i)
//    //{
//    //    const sDynamicObject& dynObj = dynamicObjects[i];
//
//    //    // check if car is in ego lane:
//    //    // d is position of dynamic object on the road -> find out which lane
//    //    double d = dynObj.d;
//
//    //    // lane is our lane 
//    //    if (d < (2 + laneWidth * egoLane + 2) && d >(2 + laneWidth * egoLane - 2))
//    //    {
//    //        // so the car is in our lane
//    //        double vx = dynObj.vx;
//    //        double vy = dynObj.vy;
//
//    //        double dynObjSpeed = sqrt(vx * vx + vy * vy);
//    //        double check_car_s = dynObj.s;
//
//    //        // check_car_s can help us to predict where that car is in the future  
//    //        // if using previous points can project s value out
//    //        check_car_s += ((double)prevPathSize * 0.2 * dynObjSpeed); 
//
//    //        // check s values greater than mine and s gap
//    //        if ((check_car_s > ego.s) && (check_car_s - ego.s) < 30)
//    //        {
//    //            // check if our car is close to the other car -> if so, need to take action
//
//    //            // do some logic here, lower reference velocity so we dont crash into the car in front of us,
//    //            // could also flag to try to change lanes
//
//    //            referenceSpeed = 29.5; // mph
//
//    //            // lines below consider this flag and reduce speed
//    //            too_close = true;
//    //            if (egoLane > 0)
//    //            {
//    //                egoLane = 0; // set left lane as target lane
//    //            }
//    //        }
//    //    }
//    //}
//
//    //if (too_close)
//    //{
//    //    referenceSpeed -= 0.224; // this is somehow related to decelerating with 5 m/s^2
//    //}
//    //else if (referenceSpeed < 49.5)
//    //{
//    //    referenceSpeed += 0.224;
//    //}
//    //// end of code from walkthrough
//
//    return referenceSpeed;
//}



bool StayOnCurrentLane(
    const sEgo& ego,
    const cRoadSituation& roadInfo)
{
    const sLaneInfo& laneInfo = roadInfo.GetLaneInfo(ego.GetCurrentLaneName());

    // no leading vehicle in ego lane or distance to leading vehicle is larger
    // than threshold
    if (    !laneInfo.IsLeadingVehicleAhead()
        ||  laneInfo.IsDistanceToLeadingVehicleLarger(thresholdKeepLane))
    {
        return true;
    }

    // distance to leading vehicle is the largest in ego lane

    return false;
}

//bool StayOnCurrentLaneAndAccelerateToMaxSpeed(
//    const cRoadSituation& roadInfo,
//    const sEgo& ego,
//    sBehavior& plannedBehavior)
//{
//    const eLaneName egoLane = ego.GetCurrentLaneName();
//
//    switch (egoLane)
//    {
//    case LN_LANE_LEFT:
//    {
//        return StayOnCurrentLaneAndAccelerateToMaxSpeed(
//            roadInfo.laneLeft, plannedBehavior);
//    }
//    break;
//    case LN_LANE_MIDDLE:
//    {
//        return StayOnCurrentLaneAndAccelerateToMaxSpeed(
//            roadInfo.laneMiddle, plannedBehavior);
//    }
//    break;
//    case LN_LANE_RIGHT:
//    {
//        return StayOnCurrentLaneAndAccelerateToMaxSpeed(
//            roadInfo.laneRight, plannedBehavior);
//    }
//    break;
//    }
//
//    return false;
//}

//bool StayOnCurrentLaneAndAccelerateToMaxSpeed(
//    const sLaneInfo& laneInfo,
//    sBehavior& plannedBehavior)
//{
//    if (    !laneInfo.leadingVehicleAhead
//        ||  laneInfo.distanceToLeadingVehicle > thresholdKeepLane)
//    {
//        plannedBehavior = sBehavior(
//            laneInfo.laneName,
//            invalidVehicleId,
//            maxSpeed,
//            0.0);
//
//        return true;
//    }
//
//    return false;
//}


//bool StayOnCurrentLaneAndAdaptSpeed(
//    const cRoadSituation& roadInfo,
//    const sEgo& ego,
//    sBehavior& plannedBehavior)
//{
//    const eLaneName egoLane = ego.GetCurrentLaneName();
//
//    switch (egoLane)
//    {
//    case LN_LANE_LEFT:
//    {
//        if (    roadInfo.laneMiddle.leadingVehicleAhead
//            &&  (   roadInfo.laneMiddle.distanceToLeadingVehicle 
//                 <  roadInfo.laneLeft.distanceToLeadingVehicle))
//        {
//            AdaptToLeadingVehicleInLane(roadInfo.laneLeft, plannedBehavior);
//            return true;
//        }
//    }
//    break;
//    case LN_LANE_MIDDLE:
//    {
//        double distanceLeadingVeh = roadInfo.laneMiddle.distanceToLeadingVehicle;
//
//        if (    roadInfo.laneLeft.leadingVehicleAhead
//            && (roadInfo.laneLeft.distanceToLeadingVehicle < distanceLeadingVeh)
//            &&  roadInfo.laneRight.leadingVehicleAhead
//            && (roadInfo.laneRight.distanceToLeadingVehicle < distanceLeadingVeh))
//        {
//            AdaptToLeadingVehicleInLane(roadInfo.laneMiddle, plannedBehavior);
//            return true;
//        }
//    }
//    break;
//    case LN_LANE_RIGHT:
//    {
//        if (roadInfo.laneMiddle.leadingVehicleAhead
//            && (roadInfo.laneMiddle.distanceToLeadingVehicle
//                <  roadInfo.laneRight.distanceToLeadingVehicle))
//        {
//            AdaptToLeadingVehicleInLane(roadInfo.laneRight, plannedBehavior);
//            return true;
//        }
//    }
//    break;
//    }
//
//    return false;
//}

//void AdaptToLeadingVehicleInLane(const sLaneInfo& currentLane, sBehavior& behavior)
//{
//    const sDynamicObject& leadingVeh = currentLane.leadingVehicle;
//    double speed = sqrt(pow(leadingVeh.vx, 2) + pow(leadingVeh.vy, 2));
//    behavior = sBehavior(
//        LN_LANE_RIGHT,
//        leadingVeh.id,
//        speed,
//        0.0);
//}


int GetLaneIdxFromLaneChangeDirection(
    const eLaneChangeDirection laneChangeDir,
    const sEgo& ego)
{
    const eLaneName egoLane = ego.GetCurrentLaneName();

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

eLaneName GetLaneNameFromLaneChangeDirection(
    const eLaneChangeDirection laneChangeDir,
    const sEgo& ego)
{
    const eLaneName egoLane = ego.GetCurrentLaneName();

    if (LCD_STRAIGHT == laneChangeDir)
    {
        return egoLane;
    }

    switch (laneChangeDir)
    {
    case LCD_LEFT:
        return GetLeftLaneName(egoLane);
        break;
    case LCD_RIGHT:
        return GetRightLaneName(egoLane);
        break;
    }

    return LN_LANE_MIDDLE;
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

eLaneName GetLeftLaneName(eLaneName laneName)
{
    if (LN_LANE_MIDDLE == laneName)
    {
        return LN_LANE_LEFT;
    }
    else if (LN_LANE_RIGHT == laneName)
    {
        return LN_LANE_MIDDLE;
    }

    return LN_LANE_MIDDLE;
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

eLaneName GetRightLaneName(eLaneName laneName)
{
    if (LN_LANE_MIDDLE == laneName)
    {
        return LN_LANE_RIGHT;
    }
    else if (LN_LANE_LEFT == laneName)
    {
        return LN_LANE_MIDDLE;
    }

    return LN_LANE_MIDDLE;
}

std::string ToString(const sBehavior& behavior)
{
    stringstream sstream;
    sstream
        << "target lane:" << behavior.targetLane
        << ", leading vehicle ID: " << behavior.targetLeadingVehicleId
        << ", target speed: " << behavior.targetSpeed
        << ", seconds to reach target: " << behavior.secondsToReachTarget
        << "\n";
    return sstream.str();
}