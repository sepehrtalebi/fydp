#pragma once

#include "Wrench.h"
#include "Vector.h"
#include "ControlInputs.h"
#include "KF.h"

void getAppliedLoadsWrapper(const double *control_inputs, const double *aircraft_state, double *forces, double *torques);

Wrench<double> getAppliedLoads(const Vector<double, KF::n> &state, const ControlInputs &control_inputs);
