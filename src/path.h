#ifndef PATH_H
#define PATH_H

#include <vector>

#include "point.h"


struct sPath
{
    std::vector<sPoint2D> points;
    double endS; // frenet s end coordinate // TODO: unused?!?
    double endD; // frenet d end coordinate // TODO: unused?!?

    sPath();

    sPath(
        const std::vector<sPoint2D>& points,
        const double endS,
        const double endD);

    sPath(
        const std::vector<double>& coordsX,
        const std::vector<double>& coordsY,
        const double endS,
        const double endD);

    double Length() const;

    // return path points up to given max length 
    std::vector<sPoint2D> PathPortion(
        const double maxLength,
        double& portionLength) const;

    std::vector<double> CoordsX() const;
    std::vector<double> CoordsY() const;
};

#endif // PATH_H