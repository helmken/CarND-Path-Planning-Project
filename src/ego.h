#ifndef EGO_H
#define EGO_H


// jwdunn1 [00:34]
// The "collider" on the car measures 4.47x2.43

enum eLaneName
{
    LN_UNDEFINED = -1,
    LN_LANE_LEFT,
    LN_LANE_MIDDLE,
    LN_LANE_RIGHT
};

const double laneWidth(4.0);

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

    eLaneName GetCurrentLaneName() const
    {
        if (d >= 0.0 && d < laneWidth)
        {
            return LN_LANE_LEFT;
        }
        else if (d >= laneWidth && d < 2 * laneWidth)
        {
            return LN_LANE_MIDDLE;
        }
        else if (d >= 2 * laneWidth && d <= 3 * laneWidth)
        {
            return LN_LANE_RIGHT;
        }

        return LN_UNDEFINED;
    }
};

#endif // EGO_H