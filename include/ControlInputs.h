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

    ControlInputs& operator=(const ControlInputs &other) {
        this->propeller_voltage = other.propeller_voltage;
        this->right_aileron_angle = other.right_aileron_angle;
        this->left_aileron_angle = other.left_aileron_angle;
        this->elevator_angle = other.elevator_angle;
        return (*this);
    }
};
