#include "../include/bmb_math/Wrench.h"
#include "../../bmb_world_model/include/world_model/Constants.h"

Accel<double> toAccel(const Wrench<double> &wrench) {
    return {wrench.force / MASS, INERTIA_TENSOR_INV * wrench.torque};
}
