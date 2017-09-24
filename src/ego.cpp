#include "ego.h"


// jwdunn1 [00:34]
// The "collider" on the car measures 4.47x2.43

sEgo::sEgo()
    : x(0.0), y(0.0)
    , s(0.0), d(0.0)
    , yaw(0.0), speed(0.0)
{
}

sEgo::sEgo(double x, double y, double s, double d, double yaw, double speed)
    : x(x), y(y), s(s), d(d), yaw(yaw), speed(speed)
{
}

eLaneName sEgo::GetLaneName() const
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
