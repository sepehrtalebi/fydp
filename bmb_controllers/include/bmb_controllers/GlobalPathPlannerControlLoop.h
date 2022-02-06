#pragma once

#include <bmb_controllers/GlobalPathPlanner.h>
#include <bmb_msgs/AircraftState>
#include <bmb_msgs/ReferenceCommand>
#include <ros/ros.h>

#include <fstream>
#include <vector>

class GlobalPathPlannerControlLoop {
    GlobalPathPlanner global_path_planner{};
    ros::Subscriber aircraft_state_sub_;

    ros::Publisher reference_command_pub_;

    void aircraftStateCallback(const bmb_msgs::AircraftState& msg);
public:
    GlobalPathPlannerControlLoop(ros::NodeHandle nh);

    void spin();
};