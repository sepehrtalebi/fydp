#include "AppliedLoads.h"
#include "Constants.h"
#include "Vector3.h"
#include "Quaternion.h"
#include <cmath>

static double saturation(const double &value, const double &limit) {
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static double saturation(const double &value, const double &min, const double &max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static Wrench<double> getPropellerLoads(const double propeller_voltage_0, const double propeller_voltage_1,
                                        const double propeller_ang_vel_1) {
    auto propeller_voltage_0_sat = saturation(propeller_voltage_0, 0, 12);
    auto propeller_voltage_1_sat = saturation(propeller_voltage_1, 0, 12);
    double propeller_ang_vel_0 = (1 / (2 * TAU_PROPELLER + T_SAMPLE)) *
                                 ((2 * TAU_PROPELLER - T_SAMPLE) * propeller_ang_vel_1 +
                                  K_PROPELLER * T_SAMPLE * (propeller_voltage_1_sat + propeller_voltage_0_sat) );
    Vector3<double> thrust = Vector3<double>{THRUST_GAIN_PROPELLER * propeller_ang_vel_0 * propeller_ang_vel_0, 0, 0};
    Vector3<double> torque = Vector3<double>{TORQUE_GAIN_PROPELLER * propeller_ang_vel_0 * propeller_ang_vel_0, 0, 0};
    return {thrust, Vector3<double>{L_FRONT_PROPELLER.cross(thrust) + torque}};
}

static Wrench<double> getRightAileronLoads(const double &angle, const double &velocity) {
    double lift = LIFT_GAIN_AILERON * saturation(angle, M_PI_4) * velocity  * velocity;
    Vector3<double> force{0, 0, -lift};
    Vector3<double> torque = L_RIGHT_AILERON.cross(force);
    return {force, torque};
}

static Wrench<double> getLeftAileronLoads(const double &angle, const double &velocity) {
    double lift = LIFT_GAIN_AILERON * saturation(angle, M_PI_4) * velocity  * velocity;
    Vector3<double> force{0, 0, -lift};
    Vector3<double> torque = L_LEFT_AILERON.cross(force);
    return {force, torque};
}

static Wrench<double> getElevatorLoads(const double &angle, const double &velocity) {
    auto angle_ = saturation(angle, M_PI_4);
    auto velocity_ = velocity;
    Vector3<double> force = Vector3<double>{0, 0, LIFT_GAIN_ELEVATOR * velocity_ * velocity_ * angle_};
    return {force, L_ELEVATOR.cross(force)};
}

static Wrench<double> getEnvironmentalLoads(const Vector<double, KF::n> &state) {
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

    Vector3<double> torque = Vector3<double>{L_BODY.cross(body_force) + L_RUDDER.cross(rudder_force)};
    return {Vector3<double>{weight + body_force + rudder_force}, torque};
}

Wrench<double> getAppliedLoads(const Vector<double, KF::n> &state, const ControlInputs &control_inputs) {
    double velocity = state[KF::vx];
    return getPropellerLoads(control_inputs.PropellerVoltage, 0, 0) +
           getRightAileronLoads(control_inputs.RightAileronAngle, velocity) +
           getLeftAileronLoads(control_inputs.LeftAileronAngle, velocity) +
           getElevatorLoads(control_inputs.ElevatorAngle, velocity) +
           getEnvironmentalLoads(state);
}

void getAppliedLoadsWrapper(const double *control_inputs, const double *aircraft_state, double *forces, double *torques) {
    Vector<double, 4> control_inputsVec{control_inputs[0], control_inputs[1], control_inputs[2], control_inputs[3]};
    Vector<double, KF::n> aircraft_state_vec;
    for (int i = 0; i < KF::n; i++) aircraft_state_vec[i] = aircraft_state[i];
    Wrench<double> wrench = getAppliedLoads(aircraft_state_vec, ControlInputs::parseU(control_inputsVec));
    for (int i = 0; i < 3; i++) {
        forces[i] = wrench.force[i];
        torques[i] = wrench.torque[i];
    }
}
