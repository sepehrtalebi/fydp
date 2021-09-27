#pragma once

#include "Vector3.h"
#include "Quaternion.h"

struct AircraftState {
    Vector3<double> Position;
    Quaternion<double> Orientation;
    Vector3<double> BodyVelocity;
    Vector3<double> BodyAngularVelocity;
    Vector3<double> BodyAngularAcceleration;
    Vector3<double> BodyAcceleration;

    Vector<double, 19> getX() const;
};
