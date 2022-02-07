#pragma once

#include <bmb_controllers/DubinsPath.h>
#include <bmb_controllers/PurePursuit.h>
#include <bmb_controllers/PID.h>
#include <ros/ros.h>
#include <bmb_world_model/Constants.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ReferenceCommand.h>
#include <bmb_msgs/StateCommand.h>


class LocalPathPlanner {
    using State = DubinsPath<double>::State;
/**
    void spin() {
        State current_state; // TODO: read from KF
        typename PurePursuit<T>::Result result = pursuer.pursue(path, current_state);
        if (result.should_replan) {
            path = DubinsPath<T>::create(current_state, goal, 5);
            result = pursuer.pursue(path, current_state);
        }
        T ang_vel = result.angular_vel;
        T vertical_force = altitude_pid.update(0); // TODO: current height
        T horizontal_force = 0; // TODO: compute from ang_vel and forward velocity

        // compute pitch, roll
        // if not possible aerodynamically, prefer altitude commands

        // send to low level controller
    }
**/

    bmb_msgs::StateCommand update(const bmb_msgs::ReferenceCommand& msg);

    LocalPathPlanner() = default;
private:
    State goal;
    DubinsPath<double> path;
    PurePursuit<double> pursuer;
    PID<double> altitude_pid{ALTITUDE_GAIN.K_P, ALTITUDE_GAIN.K_I, ALTITUDE_GAIN.K_D};
};
