#pragma once

#include "Vector3.h"
#include "Quaternion.h"

struct AircraftState {
    // represents the AircraftState as stored in Simulink

    Vector3<double> position;
    Quaternion<double> orientation;
    Vector3<double> body_velocity;
    Vector3<double> body_angular_velocity;
    Vector3<double> body_angular_acceleration;
    Vector3<double> body_acceleration;

    [[nodiscard]] Vector<double, 19> getX() const;
};
