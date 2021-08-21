#pragma once

#include "Vector3.h"
#include "Matrix.h"

struct AircraftState {
    Vector3 Position;
    Vector3 EulerAngles;
    Matrix<3, 3> DCM;
    Vector3 BodyVelocity;
    Vector3 BodyAngularVelocity;
    Vector3 BodyAngularAcceleration;
    Vector3 BodyAcceleration;
};
