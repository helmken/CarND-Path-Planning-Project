#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H


#include <chrono>
#include <memory>

#include "path.h"
#include "vehicle.h"


class cSensorFusion;
class cBehaviorPlanner;
class cTrajectoryPlanner;
class cWaypointMap;

/**
 * Main module, sets up and maintains the other modules
 * - waypoint map
 * - sensor fusion
 * - behavior planner
 * - trajectory planner
 */
class cPathPlanner
{
public:

    /**
     * Create and initialize modules.
     */
    cPathPlanner();

    /*
    * Cleanup allocated resources.
    */
    ~cPathPlanner();

    /**
     * Delegate inputs to modules.
     */
    sPath Execute(
        const sEgo& ego,
        const std::vector<sVehicle>& vehicles,
        sPath& previousPath);

private:

    // waypoints of track
    std::shared_ptr<cWaypointMap> m_waypointMap;

    // store other traffic participants
    std::shared_ptr<cSensorFusion> m_sensorFusion;

    // plan behavior based on input from sensor fusion
    std::shared_ptr<cBehaviorPlanner> m_behaviorPlanner;

    // generate trajectory based on input from behavior planner
    std::shared_ptr<cTrajectoryPlanner> m_trajectoryPlanner;


	// track time for console output
    std::chrono::time_point<std::chrono::system_clock> m_lastConsoleUpdate;

	// lap counter
    int m_lapCount;

    // length of current lap
	double m_currentLap;

	// length of track from waypoint map
	double m_trackLength;
};

#endif //PATH_PLANNER_H
