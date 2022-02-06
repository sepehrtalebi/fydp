#pragma once

#include <ros/ros.h>
#include <bmb_controllers/PID.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_msgs/StateCommand.h>
#include <bmb_utilities/PIDGains.h>
#include <bmb_world_model/Constants.h>

class LowLevelControlLoop {
 public:
  LowLevelControlLoop(ros::NodeHandle& nh, const double& update_frequency);

  void spin();
 private:
  bmb_msgs::ControlInputs getControlInputs();

  void aircraftStateCallback(const bmb_msgs::AircraftState& msg);

  void stateCommandCallback(const bmb_msgs::StateCommand& msg);

  ros::Subscriber aircraft_state_sub_;
  ros::Subscriber state_command_sub_;
  ros::Publisher control_inputs_pub_;

  bmb_msgs::AircraftState latest_aircraft_state;
  bmb_msgs::StateCommand latest_state_command;
  double update_frequency;
  PID<double> speed_pid{THROTTLE_GAIN.K_P, THROTTLE_GAIN.K_I, THROTTLE_GAIN.K_D,
                        SAMPLING_TIME};
  PID<double> roll_pid{ROLL_GAIN.K_P, ROLL_GAIN.K_I, ROLL_GAIN.K_D,
                       SAMPLING_TIME};
  PID<double> pitch_pid{PITCH_GAIN.K_P, PITCH_GAIN.K_I, PITCH_GAIN.K_D,
                        SAMPLING_TIME};
};
