#pragma once

#include <bmb_math/Accel.h>
#include <bmb_math/Wrench.h>

namespace bmb_world_model {

Accel<double> toAccel(const Wrench<double>& wrench);

} // namespace bmb_world_model
