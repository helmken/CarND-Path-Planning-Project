#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H


#include <map>
#include <vector>

#include "lane_info.h"

struct sEgo;

class cSensorFusion
{
public:
    cSensorFusion();

    void Execute(const std::vector<sVehicle>& vehicles,
    			 const sEgo& ego);

    void InitBehavior(const eLaneName targetLane, sBehavior& behavior);

    const cLaneInfo& GetLaneInfo(eLaneName laneName) const;
    bool LaneIsBlockedLeft(const sEgo& ego);
    bool LaneIsBlockedMiddle(const sEgo& ego);
    bool LaneIsBlockedRight(const sEgo& ego);

    // check neighbor lanes for lateral neighbor vehicles
    bool CheckNeighborLanes(const sEgo& ego, sBehavior& behavior);

    void DumpState() const;

    eLaneName DToLaneName(const double d) const;

private:
    void Reset();

    void SortVehiclesByLane(
        const std::vector<sVehicle>& vehicles,
        const double egoS);

    void SortVehiclesByDistanceToEgo();

    cLaneInfo m_laneLeft;
    cLaneInfo m_laneMiddle;
    cLaneInfo m_laneRight;

    // maintain map to track vehicles across update cycles
    std::map<int, sVehicle> m_vehicles;
};

#endif // SENSOR_FUSION_H
