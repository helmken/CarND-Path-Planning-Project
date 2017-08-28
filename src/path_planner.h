#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H


#include "path.h"
#include "waypoint_map.h"


class cTrajectoryPlanner;
class cBehaviorPlanner;

/**
 * Main module, sets up and maintains the other modules
 * - trajectory planner
 * - behavior planner
 * - prediction
 */
class cPathPlanner
{
public:

    /**
     * Create and initialize modules.
     */
    cPathPlanner();

    /**
     * Delegate inputs to modules.
     */
    sPath Execute(
        const sEgo& ego,
        const std::vector<sDynamicObject>& dynamicObjects,
        const sPath& previousPath,
        const sMap& waypointMap);

private:
    cTrajectoryPlanner* m_trajectoryPlanner;
    cBehaviorPlanner* m_behaviorPlanner;
};

#endif //PATH_PLANNER_H
