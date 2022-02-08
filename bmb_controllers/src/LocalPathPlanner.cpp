#include "bmb_controllers/LocalPathPlanner.h"

#include <bmb_msgs/ReferenceCommand.h>
#include <bmb_msgs/StateCommand.h>
#include <bmb_msgs/AircraftState.h>

#include <bmb_controllers/DubinsPath.h>
#include <bmb_controllers/PurePursuit.h>
#include <bmb_controllers/PIDFFController.h>
#include <ros/ros.h>
#include <bmb_world_model/Constants.h>

using State = typename DubinsPath<double>::State;

void LocalPathPlanner::referenceCommandCallBack(bmb_msgs::ReferenceCommand ref_msg) {
    this->ref_cmd = ref_msg;
    this->update_dubins = true;
}

bmb_msgs::StateCommand LocalPathPlanner::update(const bmb_msgs::AircraftState& state_msg) {
    const double& x_vel = state_msg.twist.linear.x;
    goal.pos[0] = ref_cmd.x_pos;
    goal.pos[1] = ref_cmd.y_pos;
    goal.vel[0] = ref_cmd.x_vel;
    goal.vel[1] = ref_cmd.y_vel;
    State current_state;
    current_state.pos[0] = state_msg.pose.position.x;
    current_state.pos[1] = state_msg.pose.position.y;
    current_state.vel[0] = x_vel;
    current_state.vel[1] = state_msg.twist.linear.y;
    if (update_dubins) {
        path = DubinsPath<double>::create(current_state, goal, MIN_RADIUS_CURVATURE);
        this->update_dubins = false;
    }
    auto [should_replan, angular_vel] = pursuer.pursue(path, current_state);
    if (should_replan) {
        path = DubinsPath<double>::create(current_state, goal, MIN_RADIUS_CURVATURE);
        result = pursuer.pursue(path, current_state);
    }
    double vertical_force = altitude_pid.update(state_msg.pose.position.z - ref_cmd.altitude);
    double horizontal_force = MASS * x_vel * angular_vel;
}
