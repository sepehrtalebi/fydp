#include "bmb_state_estimation/OmniscientStateEstimationNode.h"
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <bmb_msgs/AircraftState.h>
#include <vector>

OmniscientStateEstimationNode::OmniscientStateEstimationNode(ros::NodeHandle& nh) {
  // initialize subscribers
  model_states_sub_ =
      nh.subscribe("/gazebo/model_states", 1,
                   &OmniscientStateEstimationNode::modelStatesCallback, this);

  // initialize publishers
  aircraft_state_pub_ =
      nh.advertise<bmb_msgs::AircraftState>("/aircraft_state", 1);
}

OmniscientStateEstimationNode::spin() {
  ros::spin();
}

OmniscientStateEstimationNode::modelStatesCallback(const gazebo_msgs::ModelsStates& msg) {
  for (size_t i = 0; i < msg.name.size(); i++) {
    if (msg.name[i] == "aris") {
      bmb_msgs::AircraftState state;
      state.pose = msg.pose[i];
      state.twist = msg.twist[i];
      aircraft_state_pub_.publish(state);
    }
  }
}
