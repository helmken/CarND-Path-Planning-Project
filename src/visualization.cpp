#include <stdlib.h>

#include "visualization.h"

// Window size of initial window
const int WINDOWS_WIDTH = 640;
const int WINDOWS_HEIGHT = 640;

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

void cVisualization::Draw()
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

        //mapDrawer.DrawWaypointMap(ratio);

        glfwSwapBuffers(m_GLWindow);
        glfwPollEvents();
    }
}
