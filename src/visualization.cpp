#include <stdlib.h>

#include "visualization.h"

// Window size of initial window
const int WINDOWS_WIDTH = 1024;
const int WINDOWS_HEIGHT = 1024;

cVisualization::cVisualization()
    : m_GLWindow(NULL)
{
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

void cVisualization::Draw(
    const sEgo& ego,
    const std::vector<sDynamicObject>& dynamicObjects,
    const sPath& previousPath,
    const sPath& newPath)
{
    //while (!glfwWindowShouldClose(m_GLWindow))
    {
        int frameBuffWidth, frameBuffHeight;
        glfwGetFramebufferSize(m_GLWindow, &frameBuffWidth, &frameBuffHeight);
        const float ratio = (float)frameBuffWidth / (float)frameBuffHeight;

        // define which part of the window is used to display OpenGL content
        glViewport(
            0, 0,                               // x, y - lower left
            frameBuffWidth, frameBuffHeight);   // width, height

        glClear(GL_COLOR_BUFFER_BIT);

        const double surrounding(100);
        SetupProjection(
            ego.x - surrounding, ego.x + surrounding, 
            ego.y - surrounding, ego.y + surrounding, 
            ratio, 5.0);

        DrawEgo(ego);
        DrawDynamicObjects(dynamicObjects);
        DrawPath(previousPath, false);
        DrawPath(newPath, true);

        glfwSwapBuffers(m_GLWindow);
        glfwPollEvents();
    }
}

void cVisualization::GetBoundingBox(
    double& left, double& right,
    double& bottom, double& top,
    const sEgo& ego,
    const std::vector<sDynamicObject>& dynamicObjects,
    const sPath& previousPath,
    const sPath& newPath)
{
    const double areaSize(100);
    left = ego.x - areaSize;
    right = ego.x + areaSize;
    bottom = ego.y - areaSize;
    top = ego.y + areaSize;

    //for (const sDynamicObject& obj : dynamicObjects)
    //{
    //    if (obj.x < left)
    //    {
    //        left = obj.x;
    //    }
    //    if (obj.x > right)
    //    {
    //        right = obj.x;
    //    }

    //    if (obj.y < bottom)
    //    {
    //        bottom = obj.y;
    //    }
    //    if (obj.y > top)
    //    {
    //        top = obj.y;
    //    }
    //}
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
        glPointSize(1.0);
        glColor3f(1.0, 1.0, 1.0);
    }
    else
    {
        glPointSize(2.0);
        glColor3f(0.5, 0.0, 1.0); // previous path
    }

    glBegin(GL_POINTS);
    for (size_t i(0); i < path.coordsX.size(); ++i)
    {
        glVertex3d(path.coordsX[i], path.coordsY[i], 0.0);
    }
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
