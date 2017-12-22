#include <chrono>
#include <iostream>
#include <memory>

#include "behavior.h"
#include "behavior_planner.h"
#include "conversion_helpers.h"
#include "path_planner.h"
#include "sensor_fusion.h"
#include "trajectory_planner.h"
#include "waypoint_map.h"


using std::chrono::time_point;
using std::chrono::system_clock;
using std::chrono::milliseconds;
using std::chrono::duration;
using std::chrono::seconds;


cPathPlanner::cPathPlanner()
    : m_lastConsoleUpdate(seconds(0))
	, m_lapCount(0)
	, m_currentLap(0.0)
{
    m_waypointMap = std::make_shared<cWaypointMap>();
    m_waypointMap->ReadMapFile();

	m_sensorFusion = std::make_shared<cSensorFusion>();
    m_behaviorPlanner = std::make_shared<cBehaviorPlanner>();
    m_trajectoryPlanner = std::make_shared<cTrajectoryPlanner>();

    m_behaviorPlanner->Init(m_sensorFusion);
    m_trajectoryPlanner->Init(m_waypointMap.get());

    m_trackLength = m_waypointMap->GetTrackLength();
}

cPathPlanner::~cPathPlanner()
{
}

sPath cPathPlanner::Execute(
    const sEgo& ego,
    const std::vector<sVehicle>& vehicles,
    sPath& previousPath)
{
	static auto lastEgo = ego;
	static auto lastTime = system_clock::now();

	time_point<system_clock> currentTime = system_clock::now();

	auto deltaTrack = Distance(ego.x, ego.y, lastEgo.x, lastEgo.y);
	m_currentLap += deltaTrack;
	if (m_currentLap > m_trackLength)
	{
		m_currentLap -= m_trackLength;
		++m_lapCount;
	}

	duration<double> cycleDuration = currentTime - lastTime;
	const auto deltaT = cycleDuration.count();
	const auto currentV = (deltaT == 0.0) ?
			0.0 : deltaTrack / deltaT;

	// update static variables
	lastEgo = ego;
	lastTime = currentTime;

    sPath newPath;

    try
    {
        m_sensorFusion->Execute(vehicles, ego);
        const sBehavior& plannedBehavior = m_behaviorPlanner->Execute(ego);

        newPath = m_trajectoryPlanner->GenerateTrajectory(
            plannedBehavior, 
            ego, 
            previousPath);

    	duration<double> elapsedSeconds = currentTime - m_lastConsoleUpdate;
    	if (elapsedSeconds.count() > 0.1)
    	{
    		// update console
    		m_lastConsoleUpdate = currentTime;

    		std::cout << CLEAR_SCREEN;

			printf("lap: %i, current: %8.3f, v=%6.3f (m/s) %6.3f (miles/h)\n",
					m_lapCount, m_currentLap, currentV, currentV / mphAsMs);
			printf("===============================================================\n");
    		plannedBehavior.Dump();
			printf("===============================================================\n");
    		m_sensorFusion->DumpState();
    	}
    }
    catch (std::exception& ex)
    {
        std::cerr << "exception: " << ex.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "exception thrown" << std::endl;
    }

    return newPath;
}
