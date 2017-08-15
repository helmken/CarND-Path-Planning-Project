#ifndef PATH_H
#define PATH_H

struct sPath
{
    std::vector<double> x; // cartesian x coordinates
    std::vector<double> y; // cartesian y coordinates
    double endS; // frenet s end coordinate
    double endD; // frenet d end coordinate

    sPath(
        const std::vector<double>& x,
        const std::vector<double>& y,
        const double endS,
        const double endD)
        : x(x), y(y), endS(endS), endD(endD)
    {
    };
};

#endif // PATH_H