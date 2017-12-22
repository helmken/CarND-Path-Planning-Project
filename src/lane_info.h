#ifndef LANE_INFO_H
#define LANE_INFO_H


#include <vector>

#include "constants.h"
#include "vehicle.h"


struct sEgo;
struct sVehicle;

class cLaneInfo
{
public:
    cLaneInfo(const eLaneName laneName);

    void InitClosestVehicles(
        sVehicle& leadingVehicle,
        sVehicle& followingVehicle);

    bool IsWithinLaneBoundaries(const double d) const;

    bool IsLeadingVehicleAhead() const;

    double LeadingVehicleDistance() const;

    eLaneName LaneName() const;

    void AddVehicle(const sVehicle& vehicle);

    void Reset();

    // check if a vehicle occupies the lane laterally close to ego vehicle
    bool IsLaneBlocked(const sEgo& ego) const;

    const sVehicle& GetLeadingVehicle() const;
    const sVehicle& GetFollowingVehicle() const;

    void SortVehiclesByDistanceToEgo();
    std::vector<double> GetVehicleDistances() const;
    const std::vector<sVehicle>& GetVehicles() const;

private:
    const eLaneName m_laneName;

    double m_boundaryLeft;
    double m_boundaryRight;

    sVehicle m_leadingVehicle;
    sVehicle m_followingVehicle;

    std::vector<sVehicle> m_vehicles;
};

double LaneNameToD(eLaneName laneName);

eLaneName LeftLaneOf(eLaneName laneName);
eLaneName RightLaneOf(eLaneName laneName);


#endif // LANE_INFO_H
