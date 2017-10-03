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

    double Length() const
    {
        if (coordsX.empty())
        {
            return 0;
        }

        return sqrt(  pow(coordsX[coordsX.size() - 1] - coordsX[0], 2)
                    + pow(coordsY[coordsY.size() - 1] - coordsY[0], 2));
    }
};

#endif // PATH_H