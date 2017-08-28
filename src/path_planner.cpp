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

cPathPlanner::~cPathPlanner()
{
    delete m_behaviorPlanner;
    delete m_trajectoryPlanner;
}

void cPathPlanner::Init()
{
    m_waypointMap = ReadMapFile();
}

sPath cPathPlanner::Execute(
    const sEgo& ego,
    const std::vector<sDynamicObject>& dynamicObjects,
    const sPath& previousPath)
{
    size_t prevPathSize = previousPath.coordsX.size();
    
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

    return GeneratePath(ego, m_waypointMap, previousPath, lane, ref_vel);
}
