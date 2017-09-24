#include <stdlib.h>

#include "visualization.h"


using namespace std;


// Window size of initial window
const int WINDOWS_WIDTH = 1024;
const int WINDOWS_HEIGHT = 1024;

cVisualization::cVisualization(const cWaypointMap& waypointMap)
    : m_GLWindow(NULL)
    , m_shutdownVisualization(false)
    , m_waypointMap(waypointMap)
{
}

void cVisualization::Run()
{
    m_visualizationThread = thread{ &cVisualization::Visualize, this };
}

void cVisualization::Shutdown()
{
    // indicate thread to shutdown
    m_shutdownVisualization = true;

    // wait until visualization thread has finished
    m_visualizationThread.join();
}

void cVisualization::SetupGL()
{
    if (!glfwInit())
    {
        exit(EXIT_FAILURE);
    }

    m_GLWindow = glfwCreateWindow(
        WINDOWS_WIDTH, WINDOWS_HEIGHT,
        "Visualization Tools",
        NULL, NULL);

    if (!m_GLWindow)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(m_GLWindow);

    //enable anti-aliasing
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void cVisualization::ShutdownGL()
{
    glfwDestroyWindow(m_GLWindow);
    glfwTerminate();
    
    // not necessary?!?
    //exit(EXIT_SUCCESS);
}

void cVisualization::Visualize()
{
    SetupGL();

    while (!glfwWindowShouldClose(m_GLWindow)
        && !m_shutdownVisualization)
    {
        int frameBuffWidth, frameBuffHeight;
        glfwGetFramebufferSize(m_GLWindow, &frameBuffWidth, &frameBuffHeight);
        const float ratio = (float)frameBuffWidth / (float)frameBuffHeight;

        // define which part of the window is used to display OpenGL content
        glViewport(
            0, 0,                               // x, y - lower left
            frameBuffWidth, frameBuffHeight);   // width, height

        glClear(GL_COLOR_BUFFER_BIT);

        {
            // lock visualization mutex to read content for visualization
            unique_lock<mutex> lock(m_visualizationMutex);
            double left, right, bottom, top;
            //BoundingBoxOfPaths(
            //    left, right, bottom, top,
            //    previousPath, newPath);
            //BoundingBox(
            //    left, right, bottom, top,
            //    ego, dynamicObjects,
            //    previousPath, newPath);
            //m_waypointMap.GetMapBoundaries(left, right, bottom, top);
            //const double padding = 50;
            //left -= padding;
            //right += padding;
            //bottom -= padding;
            //top += padding;
            BoundingBoxEgo(left, right, bottom, top, m_ego);

            SetupProjection(
                left, right, bottom, top,
                ratio, 5.0);

            DrawEgo(m_ego);
            DrawDynamicObjects(m_dynamicObjects);
            DrawPath(m_previousPath, false);
            DrawPath(m_newPath, true);
        }

        const vector<sWaypoint>& waypoints = m_waypointMap.GetWaypoints();
        DrawTrack(waypoints);
        DrawWaypoints(waypoints);

        glfwSwapBuffers(m_GLWindow);
        glfwPollEvents();
    }

    ShutdownGL();
}

void cVisualization::Update(
    const sEgo& ego,
    const std::vector<sDynamicObject>& dynamicObjects,
    const sPath& previousPath,
    const sPath& newPath)
{
    // lock visualization mutex to write content for visualization
    unique_lock<mutex> lock(m_visualizationMutex);
    
    m_ego = ego;
    m_dynamicObjects = dynamicObjects;
    m_previousPath = previousPath;
    m_newPath = newPath;
}

void cVisualization::BoundingBox(
    double& left, double& right,
    double& bottom, double& top,
    const sEgo& ego,
    const std::vector<sDynamicObject>& dynamicObjects,
    const sPath& previousPath,
    const sPath& newPath)
{
    left = ego.x;
    right = ego.x;
    bottom = ego.y;
    top = ego.y;

    BoundingBoxDynamicObjects(
        left, right, bottom, top, dynamicObjects);

    BoundingBoxSinglePath(
        left, right, bottom, top, previousPath);
    BoundingBoxSinglePath(
        left, right, bottom, top, newPath);
}

void cVisualization::BoundingBoxDynamicObjects(
    double& left, double& right,
    double& bottom, double& top,
    const std::vector<sDynamicObject>& dynamicObjects)
{
    for (const sDynamicObject& dynObj : dynamicObjects)
    {
        if (dynObj.x < left)
        {
            left = dynObj.x;
        }
        if (dynObj.x > right)
        {
            right = dynObj.x;
        }

        if (dynObj.y < bottom)
        {
            bottom = dynObj.y;
        }
        if (dynObj.y > top)
        {
            top = dynObj.y;
        }
    }
}

void cVisualization::BoundingBoxEgo(
    double& left, double& right,
    double& bottom, double& top,
    const sEgo& ego)
{
    const double surrounding(50);
    left = ego.x - surrounding;
    right = ego.x + surrounding;
    bottom = ego.y - surrounding;
    top = ego.y + surrounding;
}

void cVisualization::BoundingBoxOfPaths(
    double& left, double& right,
    double& bottom, double& top,
    const sPath& previousPath,
    const sPath& newPath)
{
    left = std::numeric_limits<double>::max();
    right = std::numeric_limits<double>::min();
    bottom = std::numeric_limits<double>::max();
    top = std::numeric_limits<double>::min();

    BoundingBoxSinglePath(
        left, right, bottom, top, previousPath);
    BoundingBoxSinglePath(
        left, right, bottom, top, newPath);
}

void cVisualization::BoundingBoxSinglePath(
    double& left, double& right,
    double& bottom, double& top,
    const sPath& path)
{
    for (size_t i(0); i < path.coordsX.size(); ++i)
    {
        if (path.coordsX[i] < left)
        {
            left = path.coordsX[i];
        }
        if (path.coordsX[i] > right)
        {
            right = path.coordsX[i];
        }

        if (path.coordsY[i] < bottom)
        {
            bottom = path.coordsY[i];
        }
        if (path.coordsY[i] > top)
        {
            top = path.coordsY[i];
        }
    }
}

void cVisualization::DrawEgo(const sEgo& ego)
{
    glColor3f(0.0, 1.0, 0.0);

    glPointSize(5.0);
    glBegin(GL_POINTS);
    glVertex3d(ego.x, ego.y, 0.0);
    glEnd();

    glLineWidth(1.0);
    glBegin(GL_LINES);
    glVertex3d(ego.x, ego.y, 0.0);
    glVertex3d(
        ego.x + ego.speed * cos(ego.yaw * M_PI / 180.0), 
        ego.y + ego.speed * sin(ego.yaw * M_PI / 180.0),
        0.0);
    glEnd();
}

void cVisualization::DrawDynamicObjects(const std::vector<sDynamicObject>& dynamicObjects)
{
    glColor3f(1.0, 0.0, 0.0);

    for (const sDynamicObject& dynObj : dynamicObjects)
    {
        glPointSize(5.0);
        glBegin(GL_POINTS);
        glVertex3d(dynObj.x, dynObj.y, 0.0);
        glEnd();

        glLineWidth(1.0);
        glBegin(GL_LINES);
        glVertex3d(dynObj.x, dynObj.y, 0.0);
        glVertex3d(
            dynObj.x + dynObj.vx,
            dynObj.y + dynObj.vy,
            0.0);
        glEnd();
    }
}

void cVisualization::DrawPath(const sPath& path, const bool isNewPath)
{
    if (isNewPath)
    {
        glPointSize(5.0);
        glColor3f(0.0, 1.0, 0.0); // new path in green
    }
    else
    {
        glPointSize(10.0);
        glColor3f(1.0, 0.0, 0.0); // previous path in red
    }

    glBegin(GL_POINTS);
    for (size_t i(0); i < path.coordsX.size(); ++i)
    {
        glVertex3d(path.coordsX[i], path.coordsY[i], 0.0);
    }
    glEnd();
}

void cVisualization::DrawWaypointMap(const double ratio)
{
    const vector<sWaypoint>& waypoints = m_waypointMap.GetWaypoints();
    if (waypoints.empty())
    {
        return;
    }

    double left, right, bottom, top;
    m_waypointMap.GetMapBoundaries(left, right, bottom, top);
    const double padding(5.0);
    SetupProjection(left, right, bottom, top, ratio, padding);

    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    DrawTrack(waypoints);
    DrawWaypoints(waypoints);
}

void DrawTrack(const vector<sWaypoint>& waypoints)
{
    // left road boundary
    glColor3f(1.0, 1.0, 1.0);
    glLineWidth(2.0);
    DrawLaneBoundary(waypoints, 0.0);

    // right boundary of left lane
    glColor3f(0.0, 1.0, 1.0);
    glLineWidth(1.0);
    DrawLaneBoundary(waypoints, 4.0);

    // right boundary of middle lane
    DrawLaneBoundary(waypoints, 8.0);

    // right road boundary
    glColor3f(1.0, 1.0, 1.0);
    glLineWidth(2.0);
    DrawLaneBoundary(waypoints, 12.0);
}

void DrawLaneBoundary(
    const vector<sWaypoint>& waypoints,
    const double laneWidth)
{
    //glBegin(GL_LINE_STRIP);

    //for (const sWaypoint& wp : waypoints)
    //{
    //    glVertex3d(wp.x, wp.y, 0.0);
    //}

    //// connect last waypoint with first
    //const sWaypoint& firstWp = waypoints.front();
    //glVertex3d(firstWp.x, firstWp.y, 0.0);

    //glEnd();

    //// assuming lane width is 4 meters
    //const double laneWidth(4.0);

    glBegin(GL_LINE_STRIP);

    for (const sWaypoint& wp : waypoints)
    {
        glVertex3d(
            wp.x + wp.dx * laneWidth, 
            wp.y + wp.dy * laneWidth, 
            0.0);
    }

    // connect last waypoint with first
    const sWaypoint& firstWp = waypoints.front();
    glVertex3d(
        firstWp.x + firstWp.dx * laneWidth, 
        firstWp.y + firstWp.dy * laneWidth, 
        0.0);

    glEnd();
}

void DrawWaypoints(const std::vector<sWaypoint>& waypoints)
{
    //draw a point and define the size, color, and location
    glPointSize(3.0);

    glBegin(GL_POINTS);

    // first draw all waypoints with white color
    glColor3f(1.0, 1.0, 1.0);
    for (const sWaypoint& wp : waypoints)
    {
        glVertex3d(wp.x, wp.y, 0.0);
    }

    // draw first waypoint enlarged in green
    glPointSize(9.0);
    glColor3f(0.0, 1.0, 0.0);

    const sWaypoint& firstWp = waypoints.front();
    glVertex3d(firstWp.x, firstWp.y, 0.0);

    // draw last waypoint enlarged in red
    glPointSize(6.0);
    glColor3f(1.0, 0.0, 0.0);

    const sWaypoint& lastWp = waypoints.back();
    glVertex3d(lastWp.x, lastWp.y, 0.0);

    glEnd();
}


void SetupProjection(
    double left, double right,
    double bottom, double top,
    double ratio, double padding)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // Orthographic Projection
    // glOrtho describes a transformation that produces a parallel projection
    // glOrtho schaltet OpenGL praktisch in einen 2D-Modus, wo die
    // Z-Koordiante keine Rolle mehr im Bezug auf die letztendliche Größe
    // eines Objektes hat (weit entfernte Objekte (mit hoher Z-Koordinate)
    // werden genau so groß gezeichnet, wie nahe.) 
    // Damit dient die Z - Koordiante nur noch zur "Anordnung" von Vorder-
    // und Hintergründen auf der 2D-Zeichenfläche.

    //move camera to the right - grid is displayed more to the left
    //move camera upwards - grid is displayed more to the bottom

    // add some padding
    const double width = right - left;
    const double horPadd = width * (padding / 100);

    const double height = top - bottom;
    const double verPadd = height * (padding / 100);

    glOrtho(
        ratio * left - horPadd,     // left
        ratio * right + horPadd,    // right 
        bottom - verPadd,           // bottom
        top + verPadd,              // top
        1.f, -1.f);                 // near, far
}
