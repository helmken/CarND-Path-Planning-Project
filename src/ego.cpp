#include "ego.h"
#include "lane_info.h"

// jwdunn1 [00:34]
// The "collider" on the car measures 4.47x2.43


sEgo::sEgo(double x, double y, double s, double d, double yaw, double speed)
    : x(x), y(y), s(s), d(d), yaw(yaw), speed(speed)
{
}

eLaneName sEgo::GetCurrentLaneName() const
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
