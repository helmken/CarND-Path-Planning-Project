#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H


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
    void Execute(); // TODO: input parameters

private:
    cTrajectoryPlanner* m_trajectoryPlanner;
    cBehaviorPlanner* m_behaviorPlanner;
};

#endif //PATH_PLANNER_H
