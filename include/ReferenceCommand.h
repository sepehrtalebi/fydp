#pragma once

#include "Vector.h"

struct ReferenceCommand {
    double airspeed;
    double altitude;
    Vector<double, 2> waypoint;
};
