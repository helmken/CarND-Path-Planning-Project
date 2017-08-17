#include <math.h>
#include <limits>
#include <vector>

#include "behavior_planner.h"


using namespace std;

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

    vector<sDynamicObject> objsSortedByLane[3];

    SortDynamicObjectsByLane(
        dynamicObjects,
        objsSortedByLane[LANE_LEFT],
        objsSortedByLane[LANE_MIDDLE],
        objsSortedByLane[LANE_RIGHT]);

    size_t dynObjIndices[3];
    bool lanesEmpty[3];
    for (int laneIdx(0); laneIdx < 3; ++laneIdx)
    {
        lanesEmpty[laneIdx] = !FindNextDynamicObjectInLane(
            objsSortedByLane[laneIdx], ego.s, 
            dynObjIndices[laneIdx]);
    }

    if (lanesEmpty[egoLane])
    {
        // ego lane is empty -> set max speed
        referenceSpeed = 49.9; // mph
    }
    else
    {

    }

    // find reference speed to use
    // i is index of other car on the road 
    for (size_t i(0); i < dynamicObjects.size(); ++i)
    {
        const sDynamicObject& dynObj = dynamicObjects[i];

        // check if car is in ego lane:
        // d is position of dynamic object on the road -> find out which lane
        double d = dynObj.d;

        // lane is our lane 
        if (d < (2 + laneWidth * egoLane + 2) && d >(2 + laneWidth * egoLane - 2))
        {
            // so the car is in our lane
            double vx = dynObj.vx;
            double vy = dynObj.vy;

            double dynObjSpeed = sqrt(vx * vx + vy * vy);
            double check_car_s = dynObj.s;

            // check_car_s can help us to predict where that car is in the future  
            // if using previous points can project s value out
            check_car_s += ((double)prevPathSize * 0.2 * dynObjSpeed); 

            // check s values greater than mine and s gap
            if ((check_car_s > ego.s) && (check_car_s - ego.s) < 30)
            {
                // check if our car is close to the other car -> if so, need to take action

                // do some logic here, lower reference velocity so we dont crash into the car in front of us,
                // could also flag to try to change lanes

                referenceSpeed = 29.5; // mph

                // lines below consider this flag and reduce speed
                too_close = true;
                if (egoLane > 0)
                {
                    egoLane = 0; // set left lane as target lane
                }
            }
        }
    }

    if (too_close)
    {
        referenceSpeed -= 0.224; // this is somehow related to decelerating with 5 m/s^2
    }
    else if (referenceSpeed < 49.5)
    {
        referenceSpeed += 0.224;
    }

    return referenceSpeed;
}

void SortDynamicObjectsByLane(
    const std::vector<sDynamicObject>& dynamicObjects,
    std::vector<sDynamicObject>& laneLeft,
    std::vector<sDynamicObject>& laneMiddle,
    std::vector<sDynamicObject>& laneRight)
{
    for (vector<sDynamicObject>::const_iterator iter = dynamicObjects.begin();
        iter != dynamicObjects.end(); ++iter)
    {
        double d = (*iter).d;
        if (d >= 0.0 && d < laneWidth)
        {
            laneLeft.push_back(*iter);
        }
        else if (d >= laneWidth && d < 2 * laneWidth)
        {
            laneMiddle.push_back(*iter);
        }
        else if (d >= laneWidth && d < 2 * laneWidth)
        {
            laneMiddle.push_back(*iter);
        }
        else if (d >= 2 * laneWidth && d < 3 * laneWidth)
        {
            laneRight.push_back(*iter);
        }
    }
}

bool FindNextDynamicObjectInLane(
    const std::vector<sDynamicObject>& dynamicObjects,
    const double egoS,
    std::size_t& idx)
{
    double closestS = numeric_limits<double>::max();
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
        idx = closestIdx;
        return true;
    }

    return false;
}
