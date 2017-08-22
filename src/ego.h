#ifndef EGO_H
#define EGO_H


// jwdunn1 [00:34]
// The "collider" on the car measures 4.47x2.43

struct sEgo
{
    double x; // cartesian x coordinate
    double y; // cartesian y coordinate
    double s; // frenet s coordinate
    double d; // frenet d coordinate
    double yaw; // yaw of ego vehicle
    double speed; // speed of ego vehicle

    sEgo(double x, double y, double s, double d, double yaw, double speed)
        : x(x), y(y), s(s), d(d), yaw(yaw), speed(speed)
    {
    }
};

#endif // EGO_H