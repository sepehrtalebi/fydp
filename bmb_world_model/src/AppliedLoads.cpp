#include "bmb_world_model/AppliedLoads.h"
#include <bmb_differentiation/runtime/Constant.h>
#include <bmb_differentiation/runtime/Variable.h>
#include <bmb_math/Quaternion.h>
#include <bmb_math/Vector3.h>
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

static Wrench<double> getPropellerLoads(const double& propeller_force) const {
  Vector3<double> thrust{control_inputs.propeller_force, 0, 0};
  Vector3<double> torque{
      THRUST_TORQUE_RATIO_PROPELLER * control_inputs.propeller_force, 0, 0};
  return {thrust, L_FRONT_PROPELLER.cross(thrust) + torque};
}

static Wrench<double> getGravitationalLoads(const Quaternion<double>& quat) {
  return {quat.rotate(WEIGHT), Vector3<double>{}};
}

Wrench<double> getAppliedLoads(
    const bmb_msgs::AircraftState& state,
    const bmb_msgs::ControlInputs& control_inputs) const {
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
    const bmb_msgs::AircraftState& state) const {
  // TODO: this is out of date and needs to be updated
  auto wrench_jac = Matrix<double, 6, bmb_msgs::AircraftState::SIZE>::zeros();
  // propeller loads does not contribute since it does not depend on state

  static constexpr size_t VX = bmb_msgs::AircraftState::VX;
  const double& velocity = state.twist.linear.x;

  // right aileron loads
  double vel_to_lift_jac =
      LIFT_GAIN_AILERON *
      saturation(current_control_inputs.right_aileron_angle, M_PI_4) * 2 *
      velocity;
  wrench_jac[2][VX] -= vel_to_lift_jac;
  Vector3<double> vel_to_right_torque_jac =
      L_RIGHT_AILERON.cross({0, 0, -vel_to_lift_jac});
  for (size_t i = 0; i < 3; i++)
    wrench_jac[3 + i][VX] += vel_to_right_torque_jac[i];

  // left aileron loads
  const double left_aileron_angle = -current_control_inputs.right_aileron_angle;
  vel_to_lift_jac =
      LIFT_GAIN_AILERON * saturation(left_aileron_angle, M_PI_4) * 2 * velocity;
  wrench_jac[2][VX] -= vel_to_lift_jac;
  Vector3<double> vel_to_left_torque_jac =
      L_LEFT_AILERON.cross({0, 0, -vel_to_lift_jac});
  for (size_t i = 0; i < 3; i++)
    wrench_jac[3 + i][VX] += vel_to_left_torque_jac[i];

  // elevator loads
  vel_to_lift_jac = LIFT_GAIN_ELEVATOR *
                    saturation(current_control_inputs.elevator_angle, M_PI_4) *
                    2 * velocity;
  wrench_jac[2][VX] += vel_to_lift_jac;
  Vector3<double> vel_to_elevator_torque_jac =
      L_ELEVATOR.cross({0, 0, vel_to_lift_jac});
  for (size_t i = 0; i < 3; i++)
    wrench_jac[3 + i][VX] += vel_to_elevator_torque_jac[i];

  // TODO: environmental loads

  return wrench_jac;
}
