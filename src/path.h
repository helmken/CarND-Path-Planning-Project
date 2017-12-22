#ifndef PATH_H
#define PATH_H

#include <vector>

#include "point.h"


/*
* Container for previous path points provided by simulator.
*/
struct sPath
{
    std::vector<sPoint2D> points;

    sPath();

    sPath(const std::vector<sPoint2D>& points);

    sPath(const std::vector<double>& coordsX,
    	  const std::vector<double>& coordsY);

    std::vector<double> CoordsX() const;
    std::vector<double> CoordsY() const;
};

#endif // PATH_H
