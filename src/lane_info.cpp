#include <algorithm>
#include <assert.h>
#include <limits>
#include <stdexcept>

#include "ego.h"
#include "lane_info.h"
#include "vehicle.h"


auto constexpr vehicleLength(5.0);


cLaneInfo::cLaneInfo(const eLaneName laneName)
    : m_laneName(laneName)
{
    switch (laneName)
    {
    case LN_LANE_LEFT:
        m_boundaryLeft = 0.0;
        m_boundaryRight = m_boundaryLeft + laneWidth;
        break;
    case LN_LANE_MIDDLE:
        m_boundaryLeft = laneWidth;
        m_boundaryRight = m_boundaryLeft + laneWidth;
        break;
    case LN_LANE_RIGHT:
        m_boundaryLeft = 2.0 * laneWidth;
        m_boundaryRight = m_boundaryLeft + laneWidth;
        break;
    }

    m_leadingVehicle.id = invalidVehicleId;
    m_leadingVehicle.sDistanceEgo = doubleMax;
    m_followingVehicle.id = invalidVehicleId;
    m_followingVehicle.sDistanceEgo = -doubleMax;
};

void cLaneInfo::InitClosestVehicles(
    sVehicle& leadingVehicle,
    sVehicle& followingVehicle)
{
    leadingVehicle = m_leadingVehicle;
    followingVehicle = m_followingVehicle;
}

bool cLaneInfo::IsWithinLaneBoundaries(const double d) const
{
    if (d >= m_boundaryLeft && d < m_boundaryRight)
    {
        return true;
    }
    return false;
};

void cLaneInfo::AddVehicle(const sVehicle& vehicle)
{
    if (vehicle.sDistanceEgo >= 0.0)
    {
        if (vehicle.sDistanceEgo < m_leadingVehicle.sDistanceEgo)
        {
            m_leadingVehicle = vehicle;
        }
    }
    else
    {
        if (vehicle.sDistanceEgo > m_followingVehicle.sDistanceEgo)
        {
            m_followingVehicle = vehicle;
        }
    }

    m_vehicles.push_back(vehicle);
}

bool cLaneInfo::IsLeadingVehicleAhead() const
{
    return invalidVehicleId != m_leadingVehicle.id;
}

double cLaneInfo::LeadingVehicleDistance() const
{
    assert(invalidVehicleId != m_leadingVehicle.id);
    return m_leadingVehicle.sDistanceEgo;
}

eLaneName cLaneInfo::LaneName() const
{
    return m_laneName;
}

void cLaneInfo::Reset()
{
    m_vehicles.clear();

    m_leadingVehicle.id = invalidVehicleId;
    m_leadingVehicle.sDistanceEgo = doubleMax;
    
    m_followingVehicle.id = invalidVehicleId;
    m_followingVehicle.sDistanceEgo = -doubleMax;
}

bool cLaneInfo::IsLaneBlocked(const sEgo& ego) const
{
    for (const auto& vehicle : m_vehicles)
    {
        // reference point for position is approximately rear axle
        const auto deltaS = vehicle.s - ego.s;
        const auto deltaV = vehicle.v - ego.speed;
        if (	deltaS >= 0.0
        	&& 	deltaS < 2.0 * vehicleLength)
        {
            // vehicle is ahead
    		return true;
        }
        else if (	deltaS < 0.0
        		&& 	deltaS > -1.5 * vehicleLength)
        {
            // vehicle is behind
    		return true;
        }
    }
    return false;
}

const sVehicle& cLaneInfo::GetLeadingVehicle() const
{
    return m_leadingVehicle;
}

const sVehicle& cLaneInfo::GetFollowingVehicle() const
{
    return m_followingVehicle;
}

void cLaneInfo::SortVehiclesByDistanceToEgo()
{
	sort(m_vehicles.begin(), m_vehicles.end(),
		[](const sVehicle& veh0, const sVehicle& veh1) -> bool
		{
			return veh0.sDistanceEgo > veh1.sDistanceEgo;
		});
}

std::vector<double> cLaneInfo::GetVehicleDistances() const
{
	std::vector<double> distances;
	for (const auto& vehicle : m_vehicles)
	{
		distances.emplace_back(vehicle.sDistanceEgo);
	}
	return distances;
}

const std::vector<sVehicle>& cLaneInfo::GetVehicles() const
{
	return m_vehicles;
}

double LaneNameToD(eLaneName laneName)
{
    if (LN_LANE_LEFT == laneName)
    {
        return laneWidth / 2.0;
    }
    else if (LN_LANE_MIDDLE == laneName)
    {
        return laneWidth + laneWidth / 2.0;
    }
    else
    {
        // LN_LANE_RIGHT: keep extra distance to right road boundary
        return 2.0 * laneWidth + laneWidth / 2.0 - 0.2; 
    }
}

eLaneName LeftLaneOf(eLaneName laneName)
{
    if (LN_LANE_MIDDLE == laneName)
    {
        return LN_LANE_LEFT;
    }
    else if (LN_LANE_RIGHT == laneName)
    {
        return LN_LANE_MIDDLE;
    }

    return LN_UNDEFINED;
}

eLaneName RightLaneOf(eLaneName laneName)
{
    if (LN_LANE_MIDDLE == laneName)
    {
        return LN_LANE_RIGHT;
    }
    else if (LN_LANE_LEFT == laneName)
    {
        return LN_LANE_MIDDLE;
    }

    return LN_UNDEFINED;
}
