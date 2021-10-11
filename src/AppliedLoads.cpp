#include "AppliedLoads.h"

static Wrench<double> getPropellerLoads(double propeller_voltage) {
    if (propeller_voltage > 12) propeller_voltage = 12; //saturation
    else if (propeller_voltage < 0) propeller_voltage = 0; //saturation

    return {{}, {}};
}

static Wrench<double> getRightAileronLoads(const double &angle, const double &velocity) {
    return {{}, {}};
}

static Wrench<double> getLeftAileronLoads(const double &angle, const double &velocity) {
    return {{}, {}};
}

static Wrench<double> getElevatorLoads(const double &angle, const double &velocity) {
    return {{}, {}};
}

static Wrench<double> getEnvironmentalLoads(const Vector<double, KF::n> &state) {
    return {{}, {}};
}

Wrench<double> getAppliedLoads(const Vector<double, KF::n> &state, const ControlInputs &control_inputs) {
    double velocity = state[KF::vx];
    return getPropellerLoads(control_inputs.PropellerVoltage) +
           getRightAileronLoads(control_inputs.RightAileronAngle, velocity) +
           getLeftAileronLoads(control_inputs.LeftAileronAngle, velocity) +
           getElevatorLoads(control_inputs.ElevatorAngle, velocity) +
           getEnvironmentalLoads(state);
}
