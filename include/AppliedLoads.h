#pragma once

#include "Wrench.h"
#include "Vector.h"
#include "ControlInputs.h"
#include "KF.h"

Wrench<double> getAppliedLoads(const Vector<double, KF::n> &state, const ControlInputs &control_inputs);
