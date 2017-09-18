#ifndef ROAD_SITUATION_H
#define ROAD_SITUATION_H


#include <stdexcept>
#include <vector>

#include "ego.h"
#include "lane_info.h"


class cRoadSituation
{
    sLaneInfo laneLeft;
    sLaneInfo laneMiddle;
    sLaneInfo laneRight;

public:
    cRoadSituation();

    const sLaneInfo& GetLaneInfo(eLaneName laneName) const;

    void AnalyzeRoadSituation(
        const std::vector<sDynamicObject>& vehicles,
        const double egoS);

    void SortVehiclesByLane(
        const std::vector<sDynamicObject>& dynamicObjects);

    void FindLeadingVehiclesInLanes(const double egoS);

    eLaneName GetOptimalLaneForLaneChange(
        const sEgo& ego) const;

    bool IsLaneOptimal(const eLaneName lane) const;
};

#endif // ROAD_SITUATION_H