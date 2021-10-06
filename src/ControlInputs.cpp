#include "ControlInputs.h"

ControlInputs ControlInputs::parseU(const Vector<double, 4> &inputs) {
    return {inputs[0], inputs[1], inputs[2], inputs[3]};
}

Vector<double, 4> ControlInputs::getU() const {
    return Vector<double, 4>{PropellerVoltage, RightAileronAngle, LeftAileronAngle, ElevatorAngle};
}
