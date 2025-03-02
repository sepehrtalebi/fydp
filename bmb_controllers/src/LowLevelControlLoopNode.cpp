#include "bmb_controllers/LowLevelControlLoopNode.h"
#include <bmb_controllers/PIDFFController.h>
#include <bmb_math/Quaternion.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_msgs/StateCommand.h>
#include <bmb_utilities/ControllerGains.h>
#include <bmb_world_model/Constants.h>

LowLevelControlLoopNode::LowLevelControlLoopNode(ros::NodeHandle& nh,
                                                 const double& update_frequency)
    : update_frequency(update_frequency) {
  const double update_period = 1 / update_frequency;
  speed_pid = PIDFFController<double>{THROTTLE_GAIN, update_period};
  roll_pid = PIDFFController<double>{ROLL_GAIN, update_period};
  pitch_pid = PIDFFController<double>{PITCH_GAIN, update_period};

  // initialize subscribers
  aircraft_state_sub_ =
      nh.subscribe("aircraft_state", 1,
                   &LowLevelControlLoopNode::aircraftStateCallback, this);
  state_command_sub_ = nh.subscribe(
      "state_command", 1, &LowLevelControlLoopNode::stateCommandCallback, this);

  // initialize publishers
  control_inputs_pub_ =
      nh.advertise<bmb_msgs::ControlInputs>("control_inputs", 1);
}

bmb_msgs::ControlInputs LowLevelControlLoopNode::getControlInputs() {
  const Quaternion<double> orientation{latest_aircraft_state.pose.orientation};
  const double pitch = orientation.getPitch();
  const double roll = orientation.getRoll();

  bmb_msgs::ControlInputs control_inputs{};
  control_inputs.propeller_force = speed_pid.update(
      latest_aircraft_state.twist.linear.x, latest_state_command.speed);
  control_inputs.right_aileron_angle =
      roll_pid.update(roll, latest_state_command.roll);
  control_inputs.elevator_angle =
      pitch_pid.update(pitch, latest_state_command.pitch);
  return control_inputs;
}

void LowLevelControlLoopNode::aircraftStateCallback(
    const bmb_msgs::AircraftState& msg) {
  latest_aircraft_state = msg;
}

void LowLevelControlLoopNode::stateCommandCallback(
    const bmb_msgs::StateCommand& msg) {
  latest_state_command = msg;
}

void LowLevelControlLoopNode::spin() {
  ros::Rate rate{update_frequency};
  while (ros::ok()) {
    ros::spinOnce();
    control_inputs_pub_.publish(getControlInputs());
    rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "bmb_low_level_control_loop");
  ros::NodeHandle nh;
  LowLevelControlLoopNode node{nh, 100};
  node.spin();
}
