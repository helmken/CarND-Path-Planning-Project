#include "constants.h"


std::string ToString(const eLaneName laneName)
{
    switch (laneName)
    {
    case LN_UNDEFINED:
        return "UNDEFINED  ";
    case LN_LANE_LEFT:
        return "LANE_LEFT  ";
    case LN_LANE_MIDDLE:
        return "LANE_MIDDLE";
    case LN_LANE_RIGHT:
        return "LANE_RIGHT ";
    }

    return "*** unexpected lane name ***";
}

std::string ToString(const eEgoState egoState)
{
    switch (egoState)
    {
    case ES_LANE_KEEP:
        return "KEEP_LANE   ";
    case ES_LANE_CHANGE_LEFT:
        return "CHANGE_LEFT ";
    case ES_LANE_CHANGE_RIGHT:
        return "CHANGE_RIGHT";
    }

    return "*** unexpected ego state ***";
}
