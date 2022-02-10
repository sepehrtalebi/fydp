#pragma once

#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ReferenceCommand.h>
#include <bmb_msgs/StateCommand.h>
#include <ros/ros.h>

class LocalPathPlannerNode {
 public:
  LocalPathPlannerNode(ros::NodeHandle& nh, const double& update_frequency);

  void spin();

 private:
  void aircraftStateCallback(const bmb_msgs::AircraftState& msg);

  void referenceCommandCallback(const bmb_msgs::ReferenceCommand& msg);

  double update_frequency;
  ros::Subscriber aircraft_state_sub_;
  ros::Subscriber reference_command_sub_;
  ros::Publisher state_command_pub_;
};
