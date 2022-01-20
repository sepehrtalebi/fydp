#pragma once

#include "../bmb_math/include/bmb_math/Vector.h"

struct ReferenceCommand {
    double airspeed;
    double altitude;
    Vector<double, 2> waypoint;
};
