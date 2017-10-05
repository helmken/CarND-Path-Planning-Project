#include <chrono>

#include "behavior_planner.h"
#include "path_planner.h"
#include "trajectory_planner.h"


using namespace std;
using namespace std::chrono;

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
    static system_clock::time_point lastTime = system_clock::now();
    static sEgo lastEgo = ego;

    system_clock::time_point currentTime = system_clock::now();
    auto diff = currentTime - lastTime;
    const double deltaT = duration<double, milli>(diff).count() / 1000.0;
    const double deltaS = distance(ego.x, ego.y, lastEgo.x, lastEgo.y);
    double speed = 0.0;
    if (deltaT != 0.0)
    {
        speed = deltaS / deltaT;
    }
    //printf("deltaT=%.3f, deltaS=%.3f, speed=%.3f (m/s), speed=%.3f (MPH)\n", 
    //    deltaT, deltaS, speed, msToMph(speed));
    
    lastTime = currentTime;
    lastEgo = ego;
    // -> average duration is approx. 20 milliseconds

    sPath newPath;

    try
    {
        sBehavior plannedBehavior = m_behaviorPlanner->Execute(
            ego, vehicles);
        //printf(ToString(plannedBehavior).c_str());

        int targetLane = LaneNameToLaneIdx(plannedBehavior.targetLane);

        //newPath = GeneratePath(
        //    ego,
        //    m_waypointMap,
        //    previousPath,
        //    targetLane,
        //    plannedBehavior.speedAtTargetPosition);
        
        newPath = GeneratePath(
            plannedBehavior, 
            ego, 
            m_waypointMap, 
            previousPath);
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
