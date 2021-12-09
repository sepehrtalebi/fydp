#pragma once
#include "DubinsPath.h"
#include "PurePursuit.h"
#include "PID.h"

template<typename T>
class LocalPathPlanner {
    using State = typename DubinsPath<T>::State;

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

private:
    State goal;
    DubinsPath<T> path;
    PurePursuit<T> pursuer;
    PID<T> altitude_pid;
};
