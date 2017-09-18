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
    const std::vector<sDynamicObject>& vehicles,
    const sPath& previousPath)
{
    sPath newPath;

    try
    {
        sBehavior plannedBehavior = m_behaviorPlanner->Execute(
            ego, vehicles);
        printf(ToString(plannedBehavior).c_str());

        int targetLane = LaneNameToLaneIdx(plannedBehavior.targetLane);

        //double ref_vel = 5.0;
        double ref_vel = plannedBehavior.targetSpeed;

        newPath = GeneratePath(
            ego,
            m_waypointMap,
            previousPath,
            targetLane,
            ref_vel);

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
