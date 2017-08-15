#ifndef CONVERSION_HELPERS_H
#define CONVERSION_HELPERS_H

// For converting back and forth between radians and degrees.
constexpr double pi()
{
    return M_PI;
}

double deg2rad(double x)
{
    return x * pi() / 180;
}

double rad2deg(double x)
{
    return x * 180 / pi();
}

#endif // CONVERSION_HELPERS_H
