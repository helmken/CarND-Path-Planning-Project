#include "behavior_planner.h"
#include "path_planner.h"
#include "trajectory_planner.h"


cPathPlanner::cPathPlanner()
{
    m_behaviorPlanner = new cBehaviorPlanner();
    m_trajectoryPlanner = new cTrajectoryPlanner();

    m_behaviorPlanner->Init(m_trajectoryPlanner);
    //m_trajectoryPlanner->Init()
}

void cPathPlanner::Execute() // TODO: input parameters
{

}
