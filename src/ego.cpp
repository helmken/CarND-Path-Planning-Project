#include <sstream>

#include "conversion_helpers.h"
#include "ego.h"


sEgo::sEgo()
    : x(0.0), y(0.0)
    , s(0.0), d(0.0)
    , yaw(0.0), speed(0.0)
{}

sEgo::sEgo(
      double x, double y
    , double s, double d
    , double yaw, double v)
    : x(x), y(y)
    , s(s), d(d)
    , yaw(yaw), speed(mphToMs(v))
{}

eLaneName sEgo::LaneName() const
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

std::string ToString(const sEgo& ego)
{
    std::stringstream strstream;
    strstream 
        << "(x,y)=(" << ego.x << "," << ego.y 
        << "), (s,d)=(" << ego.s << "," << ego.d 
        << "), yaw=" << ego.yaw << ", speed=" << ego.speed;

    return strstream.str();
}
