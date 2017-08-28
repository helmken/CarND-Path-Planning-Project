#ifndef PATH_H
#define PATH_H

struct sPath
{
    std::vector<double> coordsX; // cartesian x coordinates
    std::vector<double> coordsY; // cartesian y coordinates
    double endS; // frenet s end coordinate
    double endD; // frenet d end coordinate

    sPath()
    {};

    sPath(
        const std::vector<double>& coordsX,
        const std::vector<double>& coordsY,
        const double endS,
        const double endD)
        : coordsX(coordsX), coordsY(coordsY)
        , endS(endS), endD(endD)
    {
    };
};

#endif // PATH_H