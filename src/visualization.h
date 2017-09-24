#ifndef VISUALIZATION_H
#define VISUALIZATION_H

extern "C"
{
// GLFW library for creating OpenGL content and managing user inputs
#include <GLFW/glfw3.h>
}

#include <mutex>
#include <vector>

#include "dynamic_object.h"
#include "ego.h"
#include "path.h"
#include "waypoint_map.h"


class cVisualization
{
private:
    void SetupGL();
    void ShutdownGL();

    GLFWwindow* m_GLWindow;
    const cWaypointMap& m_waypointMap;

    // threading related members
    std::thread m_visualizationThread;
    std::mutex m_visualizationMutex;
    bool m_shutdownVisualization;

    // local copy of content that shall be visualized
    sEgo m_ego;
    std::vector<sDynamicObject> m_dynamicObjects;
    sPath m_previousPath;
    sPath m_newPath;

public:
    cVisualization(const cWaypointMap& waypointMap);

    // start visualization thread
    void Run();

    // shut down visualization thread 
    void Shutdown();

    // main function for visualization
    void Visualize();

    void Update(
        const sEgo& ego, 
        const std::vector<sDynamicObject>& dynamicObjects,
        const sPath& previousPath,
        const sPath& newPath);

    void BoundingBox(
        double& left, double& right,
        double& bottom, double& top,
        const sEgo& ego,
        const std::vector<sDynamicObject>& dynamicObjects,
        const sPath& previousPath,
        const sPath& newPath);

    void BoundingBoxDynamicObjects(
        double& left, double& right,
        double& bottom, double& top,
        const std::vector<sDynamicObject>& dynamicObjects);

    void BoundingBoxEgo(
        double& left, double& right, 
        double& bottom, double& top,
        const sEgo& ego);

    void BoundingBoxOfPaths(
        double& left, double& right,
        double& bottom, double& top,
        const sPath& previousPath,
        const sPath& newPath);

    void BoundingBoxSinglePath(
        double& left, double& right,
        double& bottom, double& top,
        const sPath& path);

    void DrawEgo(const sEgo& ego);
    
    void DrawDynamicObjects(const std::vector<sDynamicObject>& dynamicObjects);
    
    void DrawPath(const sPath& path, const bool isNewPath);

    void DrawWaypointMap(const double ratio);
};

void DrawTrack(const std::vector<sWaypoint>& waypoints);

void DrawLaneBoundary(
    const std::vector<sWaypoint>& waypoints,
    const double laneWidth);

void DrawWaypoints(const std::vector<sWaypoint>& waypoints);

void SetupProjection(
    double left, double right,
    double bottom, double top,
    double ratio, double padding);

#endif // VISUALIZATION_H
