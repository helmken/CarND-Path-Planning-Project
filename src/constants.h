#ifndef CONSTANTS_H
#define CONSTANTS_H


#include <string>
#include <limits>


const char* const CLEAR_SCREEN = "\x1B[2J\x1B[H";

constexpr double doubleMin = std::numeric_limits<double>::min();
constexpr double doubleMax = std::numeric_limits<double>::max();

// 50 MPH = 22.352 m/s
//        = 80.467 km/h

constexpr double mphAsMs = 0.44704;

constexpr double maxSpeed(49.8 * mphAsMs); // meters per second

// requirement from rubric: max 10 m/s^2 total acceleration, max 10 m/s^3 jerk 
constexpr double maxAccel(10.0);

constexpr double cycleTime = 0.02; // 20 milliseconds as seconds

constexpr int invalidVehicleId(-999);

constexpr double laneWidth(4.0);

enum eLaneName
{
    LN_UNDEFINED = -1,
    LN_LANE_LEFT,
    LN_LANE_MIDDLE,
    LN_LANE_RIGHT
};

std::string ToString(eLaneName laneName);

enum eEgoState
{
    ES_LANE_KEEP, // stay close to lane center, drive at target speed
    ES_LANE_CHANGE_LEFT, // move to left lane
    ES_LANE_CHANGE_RIGHT // move to right lane
};

std::string ToString(const eEgoState egoState);

#endif // CONSTANTS_H
