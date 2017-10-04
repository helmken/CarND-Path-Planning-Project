#include "conversion_helpers.h"
#include "path.h"

sPath::sPath()
{}

sPath::sPath(
    const std::vector<sPoint2D>& points,
    const double endS,
    const double endD)
    : points(points),
    endS(endS), endD(endD)
{
}

sPath::sPath(
    const std::vector<double>& coordsX,
    const std::vector<double>& coordsY,
    const double endS,
    const double endD)
    : endS(endS), endD(endD)
{
    for (size_t i(0); i < coordsX.size(); ++i)
    {
        points.push_back(sPoint2D(coordsX[i], coordsY[i]));
    }
}

double sPath::Length() const
{
    if (points.empty())
    {
        return 0;
    }

    sPoint2D lastPt = points[0];

    double distSum(0);
    for (size_t i(1); i < points.size(); ++i)
    {
        distSum += distance(
            lastPt.x, lastPt.y,
            points[i].x, points[i].y);
        lastPt = points[i];
    }

    return distSum;
}

std::vector<sPoint2D> sPath::PathPortion(
    const double maxLength,
    double& portionLength) const
{
    std::vector<sPoint2D> pathPortion;
    if (points.empty())
    {
        return pathPortion;
    }

    sPoint2D lastPt = points[0];
    
    pathPortion.push_back(lastPt);

    double distSum(0);
    for (size_t i(1); i < points.size(); ++i)
    {
        const double curDist = distance(
            lastPt.x, lastPt.y, 
            points[i].x, points[i].y);
        if (distSum + curDist <= maxLength)
        {
            distSum += curDist;
            lastPt = points[i];
            pathPortion.push_back(lastPt);
        }
        else
        {
            portionLength = distSum;
            return pathPortion;
        }
    }

    return pathPortion;
}

std::vector<double> sPath::CoordsX() const
{
    std::vector<double> coordsX;
    for (size_t i(0); i < points.size(); ++i)
    {
        coordsX.push_back(points[i].x);
    }
    return coordsX;
}

std::vector<double> sPath::CoordsY() const
{
    std::vector<double> coordsY;
    for (size_t i(0); i < points.size(); ++i)
    {
        coordsY.push_back(points[i].y);
    }
    return coordsY;
}
