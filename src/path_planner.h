#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H


#include "dynamic_object.h"
#include "path.h"
#include "waypoint_map.h"


class cTrajectoryPlanner;
class cBehaviorPlanner;

/**
 * Main module, sets up and maintains the other modules
 * - trajectory planner
 * - behavior planner
 * - prediction (TODO: implement!)
 */
class cPathPlanner
{
public:

    /**
     * Create and initialize modules.
     */
    cPathPlanner(const cWaypointMap& waypointMap);

    /*
    * Cleanup allocated resources.
    */
    ~cPathPlanner();

    /**
     * Delegate inputs to modules.
     */
    sPath Execute(
        const sEgo& ego,
        const std::vector<sDynamicObject>& dynamicObjects,
        const sPath& previousPath);

private:

    /*
    * Responsible for generating trajectories.
    */
    cTrajectoryPlanner* m_trajectoryPlanner;

    /*
    * Responsible for decision to change or keep lane and adjust speed.
    */
    cBehaviorPlanner* m_behaviorPlanner;

    /*
    * Map of track, used for visualization and trajectory generation.
    */
    const cWaypointMap& m_waypointMap;
};

#endif //PATH_PLANNER_H
