#include "controller/Controller.h"
#include "Constants.h"
#include "MathUtils.h"

using math_utils::saturation;

double Controller::pitch_controller(const double &altitude_command, const double &height,
                                    const double &body_velocity, double dt) {
    double error = altitude_command - height;
    double u = pid_pitch.update(error, dt);
    double sin_AOA = 1/(body_velocity*body_velocity) * BASELINE_VELOCITY*BASELINE_VELOCITY * std::sin(TRIM);
    sin_AOA += u;
    sin_AOA = std::asin(saturation(sin_AOA, 1));
    sin_AOA = saturation(sin_AOA, 10 * M_PI / 180);
    return sin_AOA;
}

double Controller::heading_controller(const Vector<double, 2> &waypoint_command, const Vector<double, 2> &position)  {
    Vector<double, 2> dxAndDy = waypoint_command - position;
    return atan2(dxAndDy[0], dxAndDy[1]);
}

double Controller::throttle_controller(const double &airspeed_command, const double &body_velocity, double dt) {
    double error = airspeed_command - body_velocity;
    double u;
    u = pid_throttle.update(error, dt);
    return airspeed_command * THROTTLE_GAIN.K_ff + u;
}

double Controller::roll_controller(const double &heading_command, const double &yaw,
                                 double dt)  {
    double error = heading_command - yaw;
    return saturation(pid_roll.update(error, dt), 5);
}

double Controller::propeller_controller(const double &throttle_command)  {
    return throttle_command * PROPELLER_K_P;
}

double Controller::elevator_controller(double const &pitch_command, double const &aircraft_pitch, double dt) {
    double error = pitch_command - aircraft_pitch;
    double u = pid_elevator.update(error, dt);
    return pitch_command * ELEVATOR_GAIN.K_ff + u;
}

std::pair<double, double> Controller::aileron_controller(const double &roll_command, const double &aircraft_roll,
                                                         double dt) {
    double error = roll_command - aircraft_roll;
    double u = pid_aileron.update(error, dt);
    double right_aileron_angle = AILERON_GAIN.K_ff + u;
    return {right_aileron_angle, -right_aileron_angle}; //right aileron angle, left aileron angle
}

ControlInputs Controller::update(AircraftState *aircraft_state, ReferenceCommand &reference_command) {
    double pitch_command = pitch_controller(reference_command.altitude, aircraft_state->position.x,
                                            aircraft_state->body_velocity.x, SAMPLING_TIME);
    double heading_command = heading_controller(reference_command.waypoint,
                                                Vector<double, 2>{aircraft_state->position.x, aircraft_state->position.y});
    double throttle_command = throttle_controller(reference_command.airspeed, aircraft_state->body_velocity.x, SAMPLING_TIME);
    double roll_command = roll_controller(heading_command, aircraft_state->orientation.get_yaw(), SAMPLING_TIME);

    ControlInputs control_input{};
    control_input.propeller_voltage = propeller_controller(throttle_command);
    control_input.elevator_angle = elevator_controller(pitch_command, aircraft_state->orientation.get_pitch(), SAMPLING_TIME);
    std::tie(control_input.right_aileron_angle, control_input.left_aileron_angle) =
            aileron_controller(roll_command, aircraft_state->orientation.get_roll(), SAMPLING_TIME);
    return control_input;
}