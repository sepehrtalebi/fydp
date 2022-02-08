#pragma once

#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ReferenceCommand.h>
#include <ros/ros.h>
#include <cstddef>
#include <fstream>
#include <optional>
#include <vector>

// TODO: update based on the addition of velocity to ReferenceCommand
class GlobalPathPlannerNode {
 public:
  GlobalPathPlannerNode(ros::NodeHandle& nh);

  void spin();

  // TODO: starting point in global coordinates has to be stored in memory (in
  //  GPS Sensor, accessible by service),
  //  then global path planner needs to convert relative coordinate to
  //  global coordinate
 private:
  void aircraftStateCallback(const bmb_msgs::AircraftState& msg);

  static constexpr double RADIUS_TOL = 10;  // m

  ros::Subscriber aircraft_state_sub_;
  ros::Publisher reference_command_pub_;
  std::vector<bmb_msgs::ReferenceCommand> reference_commands;
  size_t waypoint_index;
};
