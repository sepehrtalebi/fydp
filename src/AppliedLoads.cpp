#include "AppliedLoads.h"

static Wrench<double> getPropellerLoads(const Vector<double, KF::n> &state, const ControlInputs &control_inputs) {
    auto propeller_voltage = control_inputs.PropellerVoltage;
    if (propeller_voltage > 12) propeller_voltage = 12; //saturation
    else if (propeller_voltage < 0) propeller_voltage = 0; //saturation

    return {};
}

Wrench<double> getAppliedLoads(const Vector<double, KF::n> &state, const ControlInputs &control_inputs) {
    return getPropellerLoads(state, control_inputs);
}
