#ifndef PATH_GENERATOR_H
#define PATH_GENERATOR_H

#include "path.h"
#include "ego.h"
#include "waypoint_map.h"

sPath GeneratePath(
    const sEgo& ego, 
    const sMap& waypointMap, 
    const sPath& previousPath, 
    const int lane,
    const double referenceVelocity);


#endif // PATH_GENERATOR_H