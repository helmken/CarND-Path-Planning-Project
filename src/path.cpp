#include "path.h"

sPath::sPath()
{}

sPath::sPath(
    const std::vector<sPoint2D>& points)
    : points(points)
{
}

sPath::sPath(
    const std::vector<double>& coordsX,
    const std::vector<double>& coordsY)
{
    for (auto i(0); i < coordsX.size(); ++i)
    {
        points.push_back(sPoint2D(coordsX[i], coordsY[i]));
    }
}

std::vector<double> sPath::CoordsX() const
{
    std::vector<double> coordsX;
    for (auto i(0); i < points.size(); ++i)
    {
        coordsX.push_back(points[i].x);
    }
    return coordsX;
}

std::vector<double> sPath::CoordsY() const
{
    std::vector<double> coordsY;
    for (auto i(0); i < points.size(); ++i)
    {
        coordsY.push_back(points[i].y);
    }
    return coordsY;
}
