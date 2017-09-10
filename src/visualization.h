#ifndef VISUALIZATION_H
#define VISUALIZATION_H

extern "C"
{
    // GLFW library for creating OpenGL content and managing user inputs
#include <GLFW/glfw3.h>
}

class cVisualization
{
private:
    GLFWwindow* m_GLWindow;

public:
    cVisualization();

    void SetupGL();

    void ShutdownGL();

    void Draw();
};

#endif // VISUALIZATION_H
