#include "bmb_controllers/LowLevelControlLoop.h"
#include <bmb_controllers/PID.h>
#include <bmb_math/Quaternion.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_msgs/StateCommand.h>

LowLevelControlLoop::LowLevelControlLoop(
    ros::NodeHandle& nh, const double& update_frequency) : update_frequency(update_frequency) {
  // initialize subscribers
  aircraft_state_sub_ = nh.subscribe(
      "aircraft_state", 1, &LowLevelControlLoop::aircraftStateCallback, this);
  state_command_sub_ = nh.subscribe(
      "state_command", 1, &LowLevelControlLoop::stateCommandCallback, this);

  // initialize publishers
  control_inputs_pub_ = nh.advertise<bmb_msgs::ControlInputs>("control_inputs", 1);
}

bmb_msgs::ControlInputs LowLevelControlLoop::getControlInputs() {
  const Quaternion<double> orientation{latest_aircraft_state.pose.orientation};
  const double pitch = orientation.getPitch();
  const double roll = orientation.getRoll();

  ControlInputs control_inputs{};
  control_inputs.propeller_voltage =
      speed_pid.update(latest_state_command.speed - latest_aircraft_state.twist.linear.x);
  control_inputs.right_aileron_angle =
      roll_pid.update(latest_state_command.roll - roll);
  control_inputs.elevator_angle = pitch_pid.update(latest_state_command.pitch - pitch);
  return control_inputs;
}

void aircraftStateCallback(const bmb_msgs::AircraftState& msg) {
  latest_aircraft_state = msg;
}

void stateCommandCallback(const bmb_msgs::StateCommand& msg) {
  latest_state_command = msg;
}

void LowLevelControlLoop::spin() {
  ros::Rate rate{update_frequency};
  const double period = 1 / update_frequency;
  while (ros::ok()) {
    ros::spinOnce();
    control_inputs_pub_.publish(getControlInputs());
    rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "bmb_low_level_control_loop");
  ros::NodeHandle nh;
  LowLevelControlLoop control_loop{nh, 100};
  control_loop.spin();
}
