#ifndef SPEED_PLANNER_H
#define SPEED_PLANNER_H

#include <vector>

#include "dynamic_object.h"
#include "ego.h"

double CalculateReferenceSpeed(
    const std::vector<sDynamicObject>& dynamicObjects, 
    int& egoLane,
    const sEgo& ego,
    size_t prevPathSize)
{
    bool too_close = false;

    // to avoid hitting other cars: go through dynamic objects list and check
    // if another car is in our lane
    // if yes: check how close

    double referenceSpeed = 1.0; // mph

    // find reference speed to use
    // i is index of other car on the road 
    for (size_t i(0); i < dynamicObjects.size(); ++i)
    {
        const sDynamicObject& dynObj = dynamicObjects[i];

        // check if car is in ego lane:
        // d is position of dynamic object on the road -> find out which lane
        double d = dynObj.d;

        // lane is our lane 
        if (d < (2 + 4 * egoLane + 2) && d >(2 + 4 * egoLane - 2))
        {
            // so the car is in our lane
            double vx = dynObj.vx;
            double vy = dynObj.vy;

            double dynObjSpeed = sqrt(vx * vx + vy * vy);
            double check_car_s = dynObj.s;

            // check_car_s can help us to predict where that car is in the future  
            check_car_s += ((double)prevPathSize * 0.2 * dynObjSpeed); // if using previous points can project s value out

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

#endif // SPEED_PLANNER_H