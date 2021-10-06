#include "AppliedLoads.h"

Wrench<double> getPropellerLoads(const Vector<double, KF::n> &state, const ControlInputs &control_inputs) {
    return {};
}

Wrench<double> getAppliedLoads(const Vector<double, KF::n> &state, const ControlInputs &control_inputs) {
    return getPropellerLoads(state, control_inputs);
}
