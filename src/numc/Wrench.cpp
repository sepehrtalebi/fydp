#include "Wrench.h"
#include "Constants.h"


Accel<double> toAccel(const Wrench<double> &wrench) {
    return {wrench.force / MASS, INERTIA_TENSOR_INV * wrench.torque};
}
