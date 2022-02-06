#include "bmb_controllers/GlobalPathPlanner.h"

#include <bmb_controllers/GlobalPathPlanner.h>
#include <bmb_msgs/AircraftState>
#include <bmb_msgs/ReferenceCommand>
#include <ros/ros.h>

GlobalPathPlannerControlLoop::GlobalPathPlannerControlLoop(ros::NodeHandle& nh) {
    aircraft_state_sub_ = nh.subscribe(
        "aircraft_state", 1,
        &GlobalPathPlannerControlLoop::aircraftStateCallback, this);

    reference_command_pub_ = nh.advertise<bmb_msgs::ReferenceCommand>("reference_command", 1);
}

void aircraftStateCallback(const bmb_msgs::AircraftState& msg) {
    bmb_msgs::ReferenceCommand ref_cmd = global_path_planner.update(msg);
    if (ref_cmd) reference_command_pub_.publish(ref_cmd);
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "bmb_global_path_planner");
    ros::NodeHandle nh;
    GlobalPathPlannerControlLoop control_loop{nh};
    control_loop.spin();
}