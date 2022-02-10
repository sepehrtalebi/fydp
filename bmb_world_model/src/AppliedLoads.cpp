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

const Matrix<ExprPtr, 3, 4> AppliedLoads::QUAT_TO_WEIGHT_JAC_EXPR =
    getQuatToWeightJacExpr();  // NOLINT(cert-err58-cpp)

void AppliedLoads::update(const bmb_msgs::ControlInputs& control_inputs) {
  last_propeller_ang_vel = getPropellerAngVelocity();
  last_control_inputs = current_control_inputs;
  current_control_inputs = control_inputs;
}

double AppliedLoads::getPropellerAngVelocity() const {
  double propeller_voltage_sat =
      saturation(current_control_inputs.propeller_voltage, 0, 12);
  double last_propeller_voltage_sat =
      saturation(last_control_inputs.propeller_voltage, 0, 12);
  return (1 / (2 * TAU_PROPELLER + T_SAMPLE)) *
         ((2 * TAU_PROPELLER - T_SAMPLE) * last_propeller_ang_vel +
          K_PROPELLER * T_SAMPLE *
              (last_propeller_voltage_sat + propeller_voltage_sat));
}

Wrench<double> AppliedLoads::getPropellerLoads() const {
  double propeller_ang_vel = getPropellerAngVelocity();
  Vector3<double> thrust{
      THRUST_GAIN_PROPELLER * propeller_ang_vel * propeller_ang_vel, 0, 0};
  Vector3<double> torque{
      TORQUE_GAIN_PROPELLER * propeller_ang_vel * propeller_ang_vel, 0, 0};

  return {thrust, L_FRONT_PROPELLER.cross(thrust) + torque};
}

Wrench<double> AppliedLoads::getRightAileronLoads(
    const double& velocity) const {
  double lift = LIFT_GAIN_AILERON *
                saturation(current_control_inputs.right_aileron_angle, M_PI_4) *
                velocity * velocity;
  Vector3<double> force{0, 0, -lift};
  return {force, L_RIGHT_AILERON.cross(force)};
}

Wrench<double> AppliedLoads::getLeftAileronLoads(const double& velocity) const {
  const double left_aileron_angle = -current_control_inputs.right_aileron_angle;
  double lift = LIFT_GAIN_AILERON * saturation(left_aileron_angle, M_PI_4) *
                velocity * velocity;
  Vector3<double> force{0, 0, -lift};
  return {force, L_LEFT_AILERON.cross(force)};
}

Wrench<double> AppliedLoads::getElevatorLoads(const double& velocity) const {
  double lift = LIFT_GAIN_ELEVATOR *
                saturation(current_control_inputs.elevator_angle, M_PI_4) *
                velocity * velocity;
  Vector3<double> force{0, 0, lift};
  return {force, L_ELEVATOR.cross(force)};
}

Wrench<double> AppliedLoads::getRudderLoads(
    const Vector3<double>& b_vel) const {
  double sin_of_rudder_angle_of_attack = b_vel.y / b_vel.x;
  double rudder_lift = -LIFT_GAIN_RUDDER * sin_of_rudder_angle_of_attack *
                       (b_vel.x * b_vel.x + b_vel.y * b_vel.y);
  Vector3<double> rudder_force{0, rudder_lift, 0};
  return {rudder_force, L_RUDDER.cross(rudder_force)};
}

Wrench<double> AppliedLoads::getBodyLoads(const Vector3<double>& b_vel) const {
  double drag = -DRAG_GAIN_BODY * b_vel.x * b_vel.x;
  double sin_of_angle_of_attack = b_vel.z / b_vel.x;
  double lift = -LIFT_GAIN_BODY * sin_of_angle_of_attack *
                (b_vel.x * b_vel.x + b_vel.z + b_vel.z);
  Vector3<double> body_force{drag, 0, lift};
  return {body_force, L_BODY.cross(body_force)};
}

Wrench<double> AppliedLoads::getWingLoads(const Vector3<double>& b_vel) const {
  // TODO: implement
  return {};
}

Wrench<double> AppliedLoads::getGravitationalLoads(
    const Quaternion<double>& quat) const {
  return {quat.rotate(WEIGHT), Vector3<double>{}};
}

Wrench<double> AppliedLoads::getAppliedLoads(
    const bmb_msgs::AircraftState& state) const {
  const Vector3<double> b_vel{state.twist.linear};
  const double& velocity = b_vel.x;
  return getPropellerLoads() + getRightAileronLoads(velocity) +
         getLeftAileronLoads(velocity) + getElevatorLoads(velocity) +
         getRudderLoads(b_vel) + getBodyLoads(b_vel) + getWingLoads(b_vel);
}

Matrix<double, 6, bmb_msgs::AircraftState::SIZE>
AppliedLoads::getAppliedLoadsJacobian(
    const bmb_msgs::AircraftState& state) const {
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
