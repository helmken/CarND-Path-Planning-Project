#include <math.h>
#include <limits>
#include <vector>

#include "behavior_planner.h"


using namespace std;


// maximum allowed speed
const double maxSpeed(49.9);


const double distanceKeepLane(30.0);  


sLaneInfo::sLaneInfo(const eLaneName laneName)
    : laneName(laneName)
    , leadingDynamicObjectAhead(false)
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
    int& egoLane,
    const sEgo& ego,
    std::size_t prevPathSize)
{
    bool too_close = false;

    // to avoid hitting other cars: go through dynamic objects list and check
    // if another car is in our lane
    // if yes: check how close

    double referenceSpeed = 1.0; // mph

    sRoadInfo roadInfo;
    AnalyseRoadSituation(
        dynamicObjects,
        ego.s,
        roadInfo);


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

        egoLane = GetLaneIdxFromLaneChangeDirection(changeDir, ego);
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
    sRoadInfo& roadInfo)
{
    SortDynamicObjectsByLane(dynamicObjects, roadInfo);
    FindLeadingDynamicObjectInLane(egoS, roadInfo.laneLeft);
    FindLeadingDynamicObjectInLane(egoS, roadInfo.laneMiddle);
    FindLeadingDynamicObjectInLane(egoS, roadInfo.laneRight);
}

void SortDynamicObjectsByLane(
    const std::vector<sDynamicObject>& dynamicObjects,
    sRoadInfo& roadInfo)
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
        laneInfo.leadingDynamicObjectAhead = true;
        laneInfo.leadingDynamicObject = dynamicObjects[closestIdx];
        laneInfo.leadingDynamicObjectDistance = closestS - egoS;
    }
    else
    {
        laneInfo.leadingDynamicObjectAhead = false;
    }
}

bool StayOnCurrentLane(
    const sRoadInfo& roadInfo,
    const sEgo& ego)
{
    const eLaneName egoLane = DToLaneName(ego.d);

    switch (egoLane)
    {
    case LN_LANE_LEFT:
        {
            if (    !roadInfo.laneLeft.leadingDynamicObjectAhead
                ||  roadInfo.laneLeft.leadingDynamicObjectDistance > distanceKeepLane)
            {
                return true;
            }
        }
        break;
    case LN_LANE_MIDDLE:
        if (!roadInfo.laneMiddle.leadingDynamicObjectAhead
            || roadInfo.laneMiddle.leadingDynamicObjectDistance > distanceKeepLane)
        {
            return true;
        }
        break;
    case LN_LANE_RIGHT:
        if (!roadInfo.laneMiddle.leadingDynamicObjectAhead
            || roadInfo.laneMiddle.leadingDynamicObjectDistance > distanceKeepLane)
        {
            return true;
        }
        break;
    }

    return false;
}

eLaneChangeDirection SelectLaneChangeDirection(
    const sRoadInfo& roadInfo,
    const sEgo& ego)
{
    const eLaneName egoLane = DToLaneName(ego.d);

    switch (egoLane)
    {
    case LN_LANE_LEFT:
        {
            if (    !roadInfo.laneMiddle.leadingDynamicObjectAhead
                ||      roadInfo.laneMiddle.leadingDynamicObjectDistance
                    >   roadInfo.laneLeft.leadingDynamicObjectDistance)
            {
                return LCD_RIGHT;
            }
        }
        break;
    case LN_LANE_MIDDLE:
        {
            if (!roadInfo.laneLeft.leadingDynamicObjectAhead)
            {
                return LCD_LEFT;
            }
            else if (!roadInfo.laneRight.leadingDynamicObjectAhead)
            {
                return LCD_RIGHT;
            }
            else if (   roadInfo.laneLeft.leadingDynamicObjectDistance
                      > roadInfo.laneRight.leadingDynamicObjectDistance)
            {
                return LCD_LEFT;
            }
            else if (   roadInfo.laneRight.leadingDynamicObjectDistance
                     >  roadInfo.laneLeft.leadingDynamicObjectDistance)
            {
                return LCD_RIGHT;
            }
        }
        break;
    case LN_LANE_RIGHT:
        {
            if (    !roadInfo.laneMiddle.leadingDynamicObjectAhead
                ||      roadInfo.laneMiddle.leadingDynamicObjectDistance
                    >   roadInfo.laneRight.leadingDynamicObjectDistance)
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
