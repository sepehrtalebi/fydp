#pragma once

#include <bmb_math/Accel.h>
#include <bmb_math/Wrench.h>

Accel<double> toAccel(const Wrench<double>& wrench);
