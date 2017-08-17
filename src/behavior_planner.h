#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <vector>

#include "dynamic_object.h"
#include "ego.h"

const int LANE_LEFT(0);
const int LANE_MIDDLE(1);
const int LANE_RIGHT(2);

const double laneWidth(4.0);

double CalculateReferenceSpeed(
    const std::vector<sDynamicObject>& dynamicObjects,
    int& egoLane,
    const sEgo& ego,
    std::size_t prevPathSize);

void SortDynamicObjectsByLane(
    const std::vector<sDynamicObject>& dynamicObjects,
    std::vector<sDynamicObject>& laneLeft,
    std::vector<sDynamicObject>& laneMiddle,
    std::vector<sDynamicObject>& laneRight);

bool FindNextDynamicObjectInLane(
    const std::vector<sDynamicObject>& dynamicObjects,
    const double egoS,
    std::size_t& idx);

#endif // BEHAVIOR_PLANNER_H