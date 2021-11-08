#pragma once

#include "Vector.h"

struct ControlInputs {
    double propeller_voltage;
    double right_aileron_angle;
    double left_aileron_angle;
    double elevator_angle;

    static ControlInputs parseU(const Vector<double, 4> &inputs);

    // may be unnecessary and replaced with more specific functions in the future
    [[nodiscard]] Vector<double, 4> getU() const;
};
