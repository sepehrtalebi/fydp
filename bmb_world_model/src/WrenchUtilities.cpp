#include "bmb_utilities/WrenchUtilities.h"

#include <bmb_math/Accel.h>
#include <bmb_math/Wrench.h>
#include <bmb_world_model/Constants.h>

namespace bmb_world_model {

Accel<double> toAccel(const Wrench<double>& wrench) {
  return {wrench.force / MASS, INERTIA_TENSOR_INV * wrench.torque};
}

}
