#include "bmb_controllers/LocalPathPlanner.h"

#include <bmb_msgs/ReferenceCommand.h>
#include <bmb_msgs/StateCommand.h>
#include <bmb_msgs/AircraftState.h>

#include <bmb_controllers/DubinsPath.h>
#include <bmb_controllers/PurePursuit.h>
#include <bmb_controllers/PIDFFController.h>
#include <ros/ros.h>
#include <bmb_world_model/Constants.h>


using State = typename DubinsPath<T>::State;

void referenceCommandCallBack(bmb_msgs::ReferenceCommand ref_msg) {
    this->ref_cmd = ref_msg;
    this->update_dubins = true;
}


bmb_msgs::StateCommand update(const bmb_msgs::AircraftState& state_msg) {
    goal.pos[0] = ref_cmd.x_pos;
    goal.pos[1] = ref_cmd.y_pos;
    goal.vel[0] = ref_cmd.x_vel;
    goal.vel[1] = ref_cmd.y_vel;
    State current_state;
    current_state.pos[0] = state_msg.x_pos;
    current.pos[1] = state_msg.y_pos;
    current_state.vel[0] = state_msg.x_vel;
    current_state.vel[1] = state_msg.y_vel;
    if (update_dubins) {
        path = DubinsPath<T>::create(current_state, goal, MIN_RADIUS_CURVATURE);
        this->update_dubins =  false;
    }
    typename PurePursuit<double>::Result result = pursuer.pursue(path, current_state);
    if (result.should_replan) {
        path = DubinsPath<T>::create(current_state, goal, MIN_RADIUS_CURVATURE);
        result = pursuer.pursue(path, current_state);
    }
    double ang_vel = result.angular_vel;
    double vertical_force = altitude_pid.update(state_msg.pose.position.z - ref_cmd.altitude);
    double horizontal_force = MASS * state_msg.x_vel * ang_vel;
}