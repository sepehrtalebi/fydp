#include "AppliedLoads.h"
#include "KF.h"
#include "Constants.h"
#include "Vector3.h"
#include "Quaternion.h"
#include "Variable.h"
#include "Constant.h"
#include <cmath>

static Matrix<ExprPtr, 3, 4> getWeightToQuatJacExpr() {
    Matrix<ExprPtr, 3, 4> expr;

    Quaternion<ExprPtr> quat{std::make_shared<Variable>("q0"),
                             std::make_shared<Variable>("q1"),
                             std::make_shared<Variable>("q2"),
                             std::make_shared<Variable>("q3")};
    // TODO: get lambda working so we can map WEIGHT with &Constant::make
    Vector3<ExprPtr> weight = quat.rotate({Constant::make(WEIGHT[0]), Constant::make(WEIGHT[1]), Constant::make(WEIGHT[2])});
    for (int i = 0; i < 3; i++) for (int j = 0; j < 4; j++)
        expr[i][j] = weight[i]->diff(std::static_pointer_cast<Variable>(quat[j])->getIdentifier())->simplify();

    return expr;
}

const Matrix<ExprPtr, 3, 4> AppliedLoads::WEIGHT_TO_QUAT_JAC_EXPR = getWeightToQuatJacExpr(); // NOLINT(cert-err58-cpp)

void AppliedLoads::update(const ControlInputs &control_inputs) {
    last_propeller_ang_vel = getPropellerAngVelocity();
    last_control_inputs = current_control_inputs;
    current_control_inputs = control_inputs;
}

double AppliedLoads::saturation(const double &value, const double &limit) {
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

double AppliedLoads::saturation(const double &value, const double &min, const double &max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

double AppliedLoads::getPropellerAngVelocity() const {
    double propeller_voltage_sat = saturation(current_control_inputs.propeller_voltage, 0, 12);
    double last_propeller_voltage_sat = saturation(last_control_inputs.propeller_voltage, 0, 12);
    return (1 / (2 * TAU_PROPELLER + T_SAMPLE)) *
           ((2 * TAU_PROPELLER - T_SAMPLE) * last_propeller_ang_vel +
            K_PROPELLER * T_SAMPLE * (last_propeller_voltage_sat + propeller_voltage_sat));
}

Wrench<double> AppliedLoads::getPropellerLoads() const {
    double propeller_ang_vel = getPropellerAngVelocity();
    Vector3<double> thrust{THRUST_GAIN_PROPELLER * propeller_ang_vel * propeller_ang_vel, 0, 0};
    Vector3<double> torque{TORQUE_GAIN_PROPELLER * propeller_ang_vel * propeller_ang_vel, 0, 0};

    return {thrust, L_FRONT_PROPELLER.cross(thrust) + torque};
}

Wrench<double> AppliedLoads::getRightAileronLoads(const double &velocity) const {
    double lift = LIFT_GAIN_AILERON * saturation(current_control_inputs.right_aileron_angle, M_PI_4) * velocity * velocity;
    Vector3<double> force{0, 0, -lift};
    return {force, L_RIGHT_AILERON.cross(force)};
}

Wrench<double> AppliedLoads::getLeftAileronLoads(const double &velocity) const {
    double lift = LIFT_GAIN_AILERON * saturation(current_control_inputs.left_aileron_angle, M_PI_4) * velocity * velocity;
    Vector3<double> force{0, 0, -lift};
    return {force, L_LEFT_AILERON.cross(force)};
}

Wrench<double> AppliedLoads::getElevatorLoads(const double &velocity) const {
    double lift = LIFT_GAIN_ELEVATOR * saturation(current_control_inputs.elevator_angle, M_PI_4) * velocity * velocity;
    Vector3<double> force{0, 0, lift};
    return {force, L_ELEVATOR.cross(force)};
}

Wrench<double> AppliedLoads::getEnvironmentalLoads(const Vector<double, n> &state) {
    Quaternion<double> quat{state[KF::q0], state[KF::q1], state[KF::q2], state[KF::q3]};
    Vector3<double> weight = quat.rotate(WEIGHT);

    Vector3<double> b_vel{state[KF::vx], state[KF::vy], state[KF::vz]}; // body velocity

    double drag = -DRAG_GAIN_BODY * b_vel.x * b_vel.x;
    double sin_of_angle_of_attack = b_vel.z / b_vel.x;
    double lift = -LIFT_GAIN_BODY * sin_of_angle_of_attack * (b_vel.x * b_vel.x + b_vel.z + b_vel.z);
    Vector3<double> body_force{drag, 0, lift};

    double sin_of_rudder_angle_of_attack = b_vel.y / b_vel.x;
    double rudder_lift = -LIFT_GAIN_RUDDER * sin_of_rudder_angle_of_attack * (b_vel.x * b_vel.x + b_vel.y * b_vel.y);
    Vector3<double> rudder_force{0, rudder_lift, 0};

    Vector3<double> torque = L_BODY.cross(body_force) + L_RUDDER.cross(rudder_force);
    return {weight + body_force + rudder_force, torque};
}

Wrench<double> AppliedLoads::getAppliedLoads(const Vector<double, n> &state) const {
    double velocity = state[KF::vx];
    return getPropellerLoads() +
           getRightAileronLoads(velocity) +
           getLeftAileronLoads(velocity) +
           getElevatorLoads(velocity) +
           getEnvironmentalLoads(state);
}

void AppliedLoads::updateWrapper(const double *control_inputs, const double *aircraft_state, double *forces,
                                 double *torques) {
    Vector<double, 4> control_inputsVec{control_inputs};
    Vector<double, n> aircraft_state_vec{aircraft_state};

    update(ControlInputs::parseU(control_inputsVec));
    Wrench<double> wrench = getAppliedLoads(aircraft_state_vec);
    for (int i = 0; i < 3; i++) {
        forces[i] = wrench.force[i];
        torques[i] = wrench.torque[i];
    }
}

Matrix<double, 6, n> AppliedLoads::getAppliedLoadsJacobian(const Vector<double, n> &state) const {
    Matrix<double, 6, n> applied_loads_jac = Matrix<double, 6, n>::zeros();
    // propeller loads does not contribute since it does not depend on state

    // right aileron loads
    double d_lift_d_vel = LIFT_GAIN_AILERON * saturation(current_control_inputs.right_aileron_angle, M_PI_4) * 2 * state[KF::vx];
    applied_loads_jac[2][KF::vx] -= d_lift_d_vel;
    Vector3<double> d_torque_right_d_vel = L_RIGHT_AILERON.cross({0, 0, -d_lift_d_vel});
    for (int i = 0; i < 3; i++) applied_loads_jac[3 + i][KF::vx] += d_torque_right_d_vel[i];

    // left aileron loads
    d_lift_d_vel = LIFT_GAIN_AILERON * saturation(current_control_inputs.left_aileron_angle, M_PI_4) * 2 * state[KF::vx];
    applied_loads_jac[2][KF::vx] -= d_lift_d_vel;
    Vector3<double> d_torque_left_d_vel = L_LEFT_AILERON.cross({0, 0, -d_lift_d_vel});
    for (int i = 0; i < 3; i++) applied_loads_jac[3 + i][KF::vx] += d_torque_left_d_vel[i];

    // elevator loads
    d_lift_d_vel = LIFT_GAIN_ELEVATOR * saturation(current_control_inputs.elevator_angle, M_PI_4) * 2 * state[KF::vx];
    applied_loads_jac[2][KF::vx] += d_lift_d_vel;
    Vector3<double> d_torque_elevator_d_vel = L_ELEVATOR.cross({0, 0, d_lift_d_vel});
    for (int i = 0; i < 3; i++) applied_loads_jac[3 + i][KF::vx] += d_torque_elevator_d_vel[i];

    return applied_loads_jac;
}
