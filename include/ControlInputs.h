#pragma once

#include "Vector.h"

struct ControlInputs {
    double PropellerVoltage;
    double RightAileronAngle;
    double LeftAileronAngle;
    double ElevatorAngle;

    static ControlInputs parseU(const Vector<double, 4> &inputs);

    // may be unnecessary and replaced with more specific functions in the future
    [[nodiscard]] Vector<double, 4> getU() const;
};
