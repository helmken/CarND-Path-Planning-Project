#include "behavior_planner.h"
#include "path_planner.h"
#include "trajectory_planner.h"


cPathPlanner::cPathPlanner()
{
    m_behaviorPlanner = new cBehaviorPlanner();
    m_trajectoryPlanner = new cTrajectoryPlanner();

    m_behaviorPlanner->Init(m_trajectoryPlanner);
    m_trajectoryPlanner->Init();
}

sPath cPathPlanner::Execute(
    const sEgo& ego,
    const std::vector<sDynamicObject>& dynamicObjects,
    const sPath& previousPath,
    const sMap& waypointMap)
{
    size_t prevPathSize = previousPath.x.size();
    
    sEgo targetEgoPos = ego;
    if (prevPathSize > 0)
    {
        targetEgoPos.s = previousPath.endS;
    }

    int lane = 1;

    double ref_vel = CalculateReferenceSpeed(
        dynamicObjects,
        lane,
        ego,
        prevPathSize);

    return GeneratePath(ego, waypointMap, previousPath, lane, ref_vel);
}
