#include <chrono>

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
    //static system_clock::time_point lastTime = system_clock::now();
    //system_clock::time_point currentTime = system_clock::now();
    //auto diff = currentTime - lastTime;
    //cout << "duration: " << duration<double, milli>(diff).count() << "\n";
    //lastTime = currentTime;
    // -> average duration is approx. 20 milliseconds

    sPath newPath;

    try
    {
        sBehavior plannedBehavior = m_behaviorPlanner->Execute(
            ego, vehicles);
        //printf(ToString(plannedBehavior).c_str());

        int targetLane = LaneNameToLaneIdx(plannedBehavior.targetLane);

        newPath = GeneratePath(
            ego,
            m_waypointMap,
            previousPath,
            targetLane,
            plannedBehavior.speedAtTargetPosition);
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
