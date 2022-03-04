#include "bmb_world_model/AppliedLoads.h"
#include <bmb_differentiation/runtime/Constant.h>
#include <bmb_differentiation/runtime/Variable.h>
#include <bmb_math/Quaternion.h>
#include <bmb_math/Vector3.h>
#include <bmb_math/Wrench.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_utilities/MathUtils.h>
#include <bmb_world_model/Constants.h>
#include <cmath>

using bmb_utilities::saturation;

static Matrix<ExprPtr, 3, 4> getQuatToWeightJacExpr() {
  Matrix<ExprPtr, 3, 4> expr;

  Quaternion<ExprPtr> quat{Variable::make("q0"), Variable::make("q1"),
                           Variable::make("q2"), Variable::make("q3")};
  Vector3<ExprPtr> weight =
      quat.rotate(WEIGHT.applyFunc<ExprPtr>(&Constant::make));
  for (size_t i = 0; i < 3; i++)
    for (size_t j = 0; j < 4; j++)
      expr[i][j] =
          weight[i]
              ->diff(
                  std::static_pointer_cast<Variable>(quat[j])->getIdentifier())
              ->simplify();

  return expr;
}

static const Matrix<ExprPtr, 3, 4> QUAT_TO_WEIGHT_JAC_EXPR =
    getQuatToWeightJacExpr();  // NOLINT(cert-err58-cpp)

static Wrench<double> getPropellerLoads(const double& propeller_force) {
  Vector3<double> thrust{propeller_force, 0, 0};
  Vector3<double> torque{THRUST_TORQUE_RATIO_PROPELLER * propeller_force, 0, 0};
  return {thrust, L_FRONT_PROPELLER.cross(thrust) + torque};
}

static Wrench<double> getGravitationalLoads(const Quaternion<double>& quat) {
  return {quat.rotate(WEIGHT), Vector3<double>{}};
}

Wrench<double> getAppliedLoads(const bmb_msgs::AircraftState& state,
                               const bmb_msgs::ControlInputs& control_inputs) {
  const auto& b_vel = state.twist.linear;
  const Quaternion<double> quat{state.pose.orientation};

  const double vx_squared = b_vel.x * b_vel.x;
  const double speed_xz_squared = vx_squared + b_vel.z + b_vel.z;
  const double speed_xy_squared = vx_squared + b_vel.y + b_vel.y;
  const double sin_aoa_xz = -b_vel.z / b_vel.x;
  const double sin_aoa_xy = b_vel.y / b_vel.x;

  return getPropellerLoads(control_inputs.propeller_force) +
         BODY_AERO_COEFFICIENTS * sin_aoa_xz * speed_xz_squared +
         AILERON_AERO_COEFFICIENTS * sin_aoa_xz * speed_xz_squared +
         ELEVATOR_AERO_COEFFICIENTS * sin_aoa_xz * speed_xz_squared +
         RUDDER_AERO_COEFFICIENTS * sin_aoa_xy * speed_xy_squared +
         getGravitationalLoads(quat);
}

Matrix<double, 6, bmb_msgs::AircraftState::SIZE> getAppliedLoadsJacobian(
    const bmb_msgs::AircraftState& state) {
  auto wrench_jac = Matrix<double, 6, bmb_msgs::AircraftState::SIZE>::zeros();
  // propeller loads does not contribute since it does not depend on state
  // TODO: implement

  return wrench_jac;
}
