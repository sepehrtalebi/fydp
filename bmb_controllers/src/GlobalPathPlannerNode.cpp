#include "bmb_controllers/GlobalPathPlannerNode.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <bmb_msgs/ReferenceCommand.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_utilities/MathUtils.h>

#include <cassert>
#include <fstream>
#include <vector>
#include <optional>

GlobalPathPlannerNode::GlobalPathPlannerNode() : waypoint_index(0) {
    aircraft_state_sub_ = nh.subscribe(
        "aircraft_state", 1,
        &GlobalPathPlannerNode::aircraftStateCallback, this);

    reference_command_pub_ = nh.advertise<bmb_msgs::ReferenceCommand>("reference_command", 1);

    // TODO: implement hard coded coordinates csv
    std::ifstream hard_coded_coordinates{
      ros::package::getPath("bmb_controllers") + "/config/hard_coded_coordinates.csv"};
    assert(hard_coded_coordinates);
    std::string data;
    while (std::getline(hard_coded_coordinates, data)) {
        std::stringstream sep(data);
        std::string       field;
        bmb_msgs::RefernceCommand command;

        // First column is airspeed
        std::getline(sep, field, ',');
        command.airspeed = std::stod(field);

        // Second column is altitude
        std::getline(sep, field, ',');
        command.altitude = std::stod(field);

        // Third column is x coord latitude, north is positive
        std::getline(sep, field, ',');
        command.latitude = std::stod(field);

        // Fourth column is y coord longitude, east is positive
        std::getline(sep, field, ',');
        command.longitude = std::stod(field);

        reference_commands.push_back(command);
    }
    hard_coded_coordinates.close();
}


void GlobalPathPlannerNode::aircraftStateCallback(const bmb_msgs::AircraftState& msg) {
    static constexpr double RADIUS_TOL_SQUARED = squared(RADIUS_TOL);
    const bmb_msgs::ReferenceCommand& command = reference_commands[waypoint_index];
    if (squared(msg.pose.position.x - command.x_pos) +
        squared(msg.pose.position.y - command.y_pos) +
        squared(msg.pose.position.altitude - command.altitude) <
        RADIUS_TOL_SQUARED) {
        waypoint_index++;
        if (waypoint_index == reference_commands.size()) waypoint_index = 0;

        reference_command_pub_.publish(reference_commands[waypoint_index]);
    }
}

void GlobalPathPlannerNode::spin() {
  ros::spin();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "bmb_global_path_planner");
  ros::NodeHandle nh;
  GlobalPathPlannerNode node{nh};
  node.spin();
}
