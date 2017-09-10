#ifndef VISUALIZATION_H
#define VISUALIZATION_H

extern "C"
{
    // GLFW library for creating OpenGL content and managing user inputs
#include <GLFW/glfw3.h>
}

#include <vector>

#include "dynamic_object.h"
#include "ego.h"
#include "path.h"


class cVisualization
{
private:
    GLFWwindow* m_GLWindow;

public:
    cVisualization();

    void SetupGL();

    void ShutdownGL();

    void Draw(
        const sEgo& ego, 
        const std::vector<sDynamicObject>& dynamicObjects,
        const sPath& previousPath,
        const sPath& newPath);

    void GetBoundingBox(
        double& left, double& right, 
        double& bottom, double& top,
        const sEgo& ego,
        const std::vector<sDynamicObject>& dynamicObjects,
        const sPath& previousPath,
        const sPath& newPath);

    void DrawEgo(const sEgo& ego);
    
    void DrawDynamicObjects(const std::vector<sDynamicObject>& dynamicObjects);
    
    void DrawPath(const sPath& path, const bool isNewPath);
};

void SetupProjection(
    double left, double right,
    double bottom, double top,
    double ratio, double padding);

#endif // VISUALIZATION_H
