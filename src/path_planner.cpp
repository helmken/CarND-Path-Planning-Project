#include "behavior_planner.h"
#include "path_planner.h"
#include "trajectory_planner.h"


cPathPlanner::cPathPlanner(const cWaypointMap& waypointMap)
    : m_waypointMap(waypointMap)
{
    m_behaviorPlanner = new cBehaviorPlanner();
    m_trajectoryPlanner = new cTrajectoryPlanner();

    m_behaviorPlanner->Init(m_trajectoryPlanner);
    m_trajectoryPlanner->Init();
}

cPathPlanner::~cPathPlanner()
{
    delete m_behaviorPlanner;
    delete m_trajectoryPlanner;
}

sPath cPathPlanner::Execute(
    const sEgo& ego,
    const std::vector<sDynamicObject>& dynamicObjects,
    const sPath& previousPath)
{
    int targetLane = 1;

    double ref_vel = CalculateReferenceSpeed(
        dynamicObjects,
        targetLane,
        ego);
    
    sPath newPath = GeneratePath(
        ego, 
        m_waypointMap, 
        previousPath, 
        targetLane, 
        ref_vel);
    
    return newPath;
}
