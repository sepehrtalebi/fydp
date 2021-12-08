#pragma once

#include "DubinsPath.h"

template<typename T>
class PurePursuit {
public:
    struct Result {
        bool should_replan;
        T angular_vel;
    };

    explicit PurePursuit(const T& lookahead_time) : lookahead_time(lookahead_time) {}

    void updatePath(const DubinsPath<T>& new_path) {
        this->path = new_path;
    }

    /**
     *
     * @param state
     * @return Commanded angular velocity. Positive indicates a turn to the left
     */
    Result pursue(const typename DubinsPath<T>::State& state) {
        Vector2 ideal_target = state.pos + state.vel * lookahead_time;
        Vector2 target = path.closestPoint(ideal_target);
        // TODO: add check to see if we are too far off course

        // readjust to a coordinate system where we are at the origin and moving in the direction of the x-axis
        target -= state.pos;
        target = getRotationMatrix(std::atan2(state.vel[1], state.vel[0])).transpose() * target;

        if (target[0] <= 0) {
            // target is behind us (or to the side of us)
            return {true, 0};
        }

        // now the center of rotation must be at the point of intersection of the y-axis and
        // the perpendicular bisector of the line segment connecting target to the origin
        // simple geometry shows that the y coordinate where this occurs is given by the following
        // note that this will be negative if target[1] < 0, as it should be in that case to indicate a right turn
        T radius = target.magnitudeSquared() / (2 * target[1]);
        return {false, state.vel.magnitude() / radius};
    }

private:
    using Vector2 = Vector<T, 2>;
    T lookahead_time;
    DubinsPath<T> path;
};