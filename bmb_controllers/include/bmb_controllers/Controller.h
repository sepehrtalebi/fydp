#pragma once
#include "../../../bmb_msgs/include/msgs/AircraftState.h"
#include "../../../bmb_msgs/include/msgs/ReferenceCommand.h"
#include "../../../bmb_msgs/include/msgs/ControlInputs.h"
#include "PID.h"
#include "../../../bmb_world_model/include/world_model/Constants.h"
#include "PIDGains.h"

class Controller {
    PID<double> pid_throttle{THROTTLE_GAIN.K_P, THROTTLE_GAIN.K_I, THROTTLE_GAIN.K_D};
    PID<double> pid_roll{ROLL_GAIN.K_P, ROLL_GAIN.K_I, ROLL_GAIN.K_D};
    PID<double> pid_elevator{ELEVATOR_GAIN.K_P, ELEVATOR_GAIN.K_I, ELEVATOR_GAIN.K_D};
    PID<double> pid_aileron{AILERON_GAIN.K_P, AILERON_GAIN.K_I, AILERON_GAIN.K_D};
    PID<double> pid_pitch{PITCH_GAIN.K_P, PITCH_GAIN.K_I, PITCH_GAIN.K_D};

    double heading_controller(Vector<double, 2> const &waypoint_command, Vector<double, 2> const &position);
    double throttle_controller(double const &airspeed_command, double const &body_velocity, double dt);
    double roll_controller(double const &heading_command, double const &yaw, double dt);
    double propeller_controller(const double &throttle_command);
    double elevator_controller(double const &pitch_command, double const &aircraft_pitch, double dt);
    std::pair<double, double> aileron_controller(double const &roll_command, double const &aircraft_roll, double dt);
    double pitch_controller(double const &altitude_command, double const &height, double const &body_velocity, double dt);
public:
    ControlInputs update(AircraftState *aircraft_state, ReferenceCommand &reference_command);
};
