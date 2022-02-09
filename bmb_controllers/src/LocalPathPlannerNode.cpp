#include "bmb_controllers/LocalPathPlannerNode.h"
#include <ros/ros.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ReferenceCommand.h>
#include <bmb_msgs/StateCommand.h>

LocalPathPlannerNode::LocalPathPlannerNode(ros::NodeHandle& nh,
                                           const double& update_frequency)
    : update_frequency(update_frequency) {
  // initialize subscribers
  aircraft_state_sub_ = nh.subscribe(
      "aircraft_state", 1, &LocalPathPlannerNode::aircraftStateCallback, this);
  reference_command_sub_ = nh.subscribe(
      "reference_command", 1, &LocalPathPlannerNode::referenceCommandCallback, this);

  // initialize publishers
  state_command_pub_ = nh.advertise<bmb_msgs::StateCommand>("state_command", 1);
}

void LocalPathPlannerNode::aircraftStateCallback(const bmb_msgs::AircraftState& msg) {

}

void LocalPathPlannerNode::referenceCommandCallback(const bmb_msgs::ReferenceCommand& msg) {

}

void LocalPathPlannerNode::spin() {
  ros::Rate rate{update_frequency};
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "bmb_local_path_planner");
  ros::NodeHandle nh;
  LocalPathPlannerNode node{nh, 1};
  node.spin();
}
