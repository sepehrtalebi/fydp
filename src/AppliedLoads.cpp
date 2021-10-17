#include "AppliedLoads.h"
#include "Constants.h"
#include "Vector3.h"
#include "Quaternion.h"

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

static Wrench<double> getPropellerLoads(double propeller_voltage) {
    if (propeller_voltage > 12) propeller_voltage = 12; //saturation
    else if (propeller_voltage < 0) propeller_voltage = 0; //saturation

    return {{}, {}};
}

static Wrench<double> getRightAileronLoads(const double &angle, const double &velocity) {
    double lift = LIFT_GAIN_AILERON * saturation(angle, PI / 4) * velocity  * velocity;
    Vector3<double> force{0, 0, -lift};
    Vector3<double> torque = L_RIGHT_AILERON.cross(force);
    return {force, torque};
}

static Wrench<double> getLeftAileronLoads(const double &angle, const double &velocity) {
    double lift = LIFT_GAIN_AILERON * saturation(angle, PI / 4) * velocity  * velocity;
    Vector3<double> force{0, 0, -lift};
    Vector3<double> torque = L_LEFT_AILERON.cross(force);
    return {force, torque};
}

static Wrench<double> getElevatorLoads(const double &angle, const double &velocity) {
    return {{}, {}};
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
    return getPropellerLoads(control_inputs.PropellerVoltage) +
           getRightAileronLoads(control_inputs.RightAileronAngle, velocity) +
           getLeftAileronLoads(control_inputs.LeftAileronAngle, velocity) +
           getElevatorLoads(control_inputs.ElevatorAngle, velocity) +
           getEnvironmentalLoads(state);
}
