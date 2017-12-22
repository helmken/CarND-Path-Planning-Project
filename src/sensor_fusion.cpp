#include <ctime>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <sstream>
#include <vector>

#include "behavior.h"
#include "ego.h"
#include "sensor_fusion.h"
#include "vehicle.h"


cSensorFusion::cSensorFusion()
    : m_laneLeft(LN_LANE_LEFT)
    , m_laneMiddle(LN_LANE_MIDDLE)
    , m_laneRight(LN_LANE_RIGHT)
{};


void cSensorFusion::Execute(
    const std::vector<sVehicle>& vehicles, 
    const sEgo& ego)
{
    Reset();
    SortVehiclesByLane(vehicles, ego.s);
}

void cSensorFusion::DumpState() const
{
    auto distLeft = 999.0;
    auto vehicleIdLeft = -1;
    if (m_laneLeft.IsLeadingVehicleAhead())
    {
        distLeft = m_laneLeft.LeadingVehicleDistance();
        vehicleIdLeft = m_laneLeft.GetLeadingVehicle().id;
    }

    auto distMiddle = 999.0;
    auto vehicleIdMiddle = -1;
    if (m_laneMiddle.IsLeadingVehicleAhead())
    {
        distMiddle = m_laneMiddle.LeadingVehicleDistance();
        vehicleIdMiddle = m_laneMiddle.GetLeadingVehicle().id;
    }

    auto distRight = 999.0;
    auto vehicleIdRight = -1;
    if (m_laneRight.IsLeadingVehicleAhead())
    {
        distRight = m_laneRight.LeadingVehicleDistance();
        vehicleIdRight = m_laneRight.GetLeadingVehicle().id;
    }

    printf("Sensor Fusion:\n");
    printf("front:  | L: %2i, %8.3f | M: %2i, %8.3f | R: %2i, %8.3f |\n",
        vehicleIdLeft, distLeft,
        vehicleIdMiddle, distMiddle,
        vehicleIdRight, distRight);

    distLeft = -999;
    vehicleIdLeft = -1;
    if (m_laneLeft.GetFollowingVehicle().id != invalidVehicleId)
    {
        distLeft = m_laneLeft.GetFollowingVehicle().sDistanceEgo;
        vehicleIdLeft = m_laneLeft.GetFollowingVehicle().id;
    }

    distMiddle = -999;
    vehicleIdMiddle = -1;
    if (m_laneMiddle.GetFollowingVehicle().id != invalidVehicleId)
    {
        distMiddle = m_laneMiddle.GetFollowingVehicle().sDistanceEgo;
        vehicleIdMiddle = m_laneMiddle.GetFollowingVehicle().id;
    }

    distRight = -999;
    vehicleIdRight = -1;
    if (m_laneRight.GetFollowingVehicle().id != invalidVehicleId)
    {
        distRight = m_laneRight.GetFollowingVehicle().sDistanceEgo;
        vehicleIdRight = m_laneRight.GetFollowingVehicle().id;
    }

    printf("rear:   | L: %2i, %8.3f | M: %2i, %8.3f | R: %2i, %8.3f |\n",
        vehicleIdLeft, distLeft,
        vehicleIdMiddle, distMiddle,
        vehicleIdRight, distRight);

    printf("\n");

    auto vehiclesLeft = m_laneLeft.GetVehicles();
    auto vehiclesMiddle = m_laneMiddle.GetVehicles();
    auto vehiclesRight = m_laneRight.GetVehicles();

    const auto maxDistanceCount =
		std::max(
			std::max(
				vehiclesLeft.size(),
				vehiclesMiddle.size()),
			vehiclesRight.size());

    for (auto vehIdx = 0; vehIdx < maxDistanceCount; ++vehIdx)
    {
    	const char* const empty = "            ";

    	if (vehIdx < vehiclesLeft.size())
    	{
    		printf("%2i: %8.3f | ",
				vehiclesLeft[vehIdx].id,
				vehiclesLeft[vehIdx].sDistanceEgo);
    	}
    	else
    	{
    		printf("%s | ", empty);
    	}

    	if (vehIdx < vehiclesMiddle.size())
    	{
    		printf("%2i: %8.3f | ",
				vehiclesMiddle[vehIdx].id,
				vehiclesMiddle[vehIdx].sDistanceEgo);
    	}
    	else
    	{
    		printf("%s | ", empty);
    	}

    	if (vehIdx < vehiclesRight.size())
    	{
    		printf("%2i: %8.3f\n",
				vehiclesRight[vehIdx].id,
				vehiclesRight[vehIdx].sDistanceEgo);
    	}
    	else
    	{
    		printf("%s\n", empty);
    	}
    }
}

void cSensorFusion::InitBehavior(const eLaneName targetLane, sBehavior& behavior)
{
    behavior.targetLane = targetLane;

    switch (behavior.targetLane)
    {
    case LN_LANE_LEFT:
        m_laneLeft.InitClosestVehicles(
            behavior.leadingVehicle, behavior.followingVehicle);
        m_laneMiddle.InitClosestVehicles(
            behavior.leadingVehicleRight, behavior.followingVehicleRight);
        break;
    case LN_LANE_MIDDLE:
        m_laneLeft.InitClosestVehicles(
            behavior.leadingVehicleLeft, behavior.followingVehicleLeft);
        m_laneMiddle.InitClosestVehicles(
            behavior.leadingVehicle, behavior.followingVehicle);
        m_laneRight.InitClosestVehicles(
            behavior.leadingVehicleRight, behavior.followingVehicleRight);
        break;
    case LN_LANE_RIGHT:
        m_laneMiddle.InitClosestVehicles(
            behavior.leadingVehicleLeft, behavior.followingVehicleLeft);
        m_laneRight.InitClosestVehicles(
            behavior.leadingVehicle, behavior.followingVehicle);
        break;
    }
}

const cLaneInfo& cSensorFusion::GetLaneInfo(eLaneName laneName) const
{
    switch (laneName)
    {
        case LN_LANE_LEFT:
        {
            return m_laneLeft;
        }
        case LN_LANE_MIDDLE:
        {
            return m_laneMiddle;
        }
        case LN_LANE_RIGHT:
        {
            return m_laneRight;
        }
        default:
        {
            throw std::invalid_argument("cSensorFusion::GetLaneInfo: unexpected lane name");
        }
    }
}

bool cSensorFusion::LaneIsBlockedLeft(const sEgo& ego)
{
    return m_laneLeft.IsLaneBlocked(ego);
}

bool cSensorFusion::LaneIsBlockedMiddle(const sEgo& ego)
{
    return m_laneMiddle.IsLaneBlocked(ego);
}

bool cSensorFusion::LaneIsBlockedRight(const sEgo& ego)
{
    return m_laneRight.IsLaneBlocked(ego);
}

void cSensorFusion::Reset()
{
    m_laneLeft.Reset();
    m_laneMiddle.Reset();
    m_laneRight.Reset();
}

void cSensorFusion::SortVehiclesByLane(
    const std::vector<sVehicle>& vehicles,
    const double egoS)
{
    if (vehicles.empty())
    {
        //printf("%s: received empty vehicle list\n", __FUNCTION__);
        return;
    }

    // compare received vehicles with already existing vehicles
    std::set<int> currentIds;

    // copy vehicle and calculate distance to ego
    for (auto vehicle : vehicles)
    {
        auto it = m_vehicles.find(vehicle.id);
        if (it == m_vehicles.end())
        {
            //printf("%s: adding vehicle %i\n",
            //    __FUNCTION__, vehicle.id);
            m_vehicles[vehicle.id] = vehicle;
        }
        else
        {
        	const eLaneName prevLane = DToLaneName(it->second.d);
        	const eLaneName currentLane = DToLaneName(vehicle.d);

        	if (prevLane != currentLane)
        	{
				//printf("%s: vehicle %i changed lane: %s -> %s\n",
				//	__FUNCTION__, vehicle.id,
				//	ToString(prevLane).c_str(),
				//	ToString(currentLane).c_str());
        	}

        	m_vehicles[vehicle.id] = vehicle;
        }

        currentIds.insert(vehicle.id);

        vehicle.sDistanceEgo = vehicle.s - egoS;
        if (m_laneLeft.IsWithinLaneBoundaries(vehicle.d))
        {
            m_laneLeft.AddVehicle(vehicle);
        }
        else if (m_laneMiddle.IsWithinLaneBoundaries(vehicle.d))
        {
            m_laneMiddle.AddVehicle(vehicle);
        }
        else if (m_laneRight.IsWithinLaneBoundaries(vehicle.d))
        {
            m_laneRight.AddVehicle(vehicle);
        }
    }

    SortVehiclesByDistanceToEgo();

    // compare maintained vehicles with received vehicle to find
    // missing vehicles of current update
    for (auto it = m_vehicles.begin(); it != m_vehicles.end(); ++it)
    {
        const int id = it->first;
        if (currentIds.find(id) == currentIds.end())
        {
            //printf("%s: update for id=%i is missing!\n", __FUNCTION__, id);
        }
    }
}

void cSensorFusion::SortVehiclesByDistanceToEgo()
{
	m_laneLeft.SortVehiclesByDistanceToEgo();
	m_laneMiddle.SortVehiclesByDistanceToEgo();
	m_laneRight.SortVehiclesByDistanceToEgo();
}

bool cSensorFusion::CheckNeighborLanes(const sEgo& ego, sBehavior& behavior)
{
    switch (ego.LaneName())
    {
    case LN_LANE_LEFT:
        behavior.laneIsBlockedLeft = false;
        behavior.laneIsBlockedRight = m_laneMiddle.IsLaneBlocked(ego);
        break;
    case LN_LANE_MIDDLE:
        behavior.laneIsBlockedLeft = m_laneLeft.IsLaneBlocked(ego);
        behavior.laneIsBlockedRight = m_laneRight.IsLaneBlocked(ego);
        break;
    case LN_LANE_RIGHT:
        behavior.laneIsBlockedLeft = m_laneMiddle.IsLaneBlocked(ego);
        behavior.laneIsBlockedRight = false;
        break;
    }

    return false;
}

eLaneName cSensorFusion::DToLaneName(const double d) const
{
	if (m_laneLeft.IsWithinLaneBoundaries(d))
	{
		return LN_LANE_LEFT;
	}
	else if (m_laneMiddle.IsWithinLaneBoundaries(d))
	{
		return LN_LANE_MIDDLE;
	}
	else if (m_laneRight.IsWithinLaneBoundaries(d))
	{
		return LN_LANE_RIGHT;
	}

	return LN_UNDEFINED;
}

