#pragma once

#include "DubinsPath.h"
#include <cmath>

template<typename T>
class PurePursuit {
public:
    struct Result {
        bool should_replan;
        T angular_vel;
    };

    explicit PurePursuit(const T &lookahead_time, const T &min_radius) :
            lookahead_time(lookahead_time), min_radius(min_radius) {}

    void updatePath(const DubinsPath<T> &new_path) {
        this->path = new_path;
    }

    /**
     *
     * @param state
     * @return Commanded angular velocity. Positive indicates a turn to the left
     */
    Result pursue(const typename DubinsPath<T>::State &state) {
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

        if (std::fabs(radius) < min_radius) {
            // required turn is too sharp, request replan
            return {true, 0};
        }

        return {false, state.vel.magnitude() / radius};
    }

    /**
     * Outputs the anticipated trajectory that will be followed based on the current applied settings and path.
     * The state will be integrated forwards in time with a step size of STEP_SIZE in time.
     * Integration will be stopped once a replan is requested.
     */
    template<typename OStream>
    void toCSV(OStream &out, const typename DubinsPath<T>::State& start) const {
        State state = start;
        out << state.pos[0] << ", " << state.pos[1] << std::endl;

        while (true) {
            auto [should_replan, ang_vel] = pursue(state);
            if (should_replan) break;

            state.pos += state.vel * STEP_SIZE;
            state.vel = getRotationMatrix(ang_vel * STEP_SIZE) * state.vel;
            out << state.pos[0] << ", " << state.pos[1] << std::endl;
        }
    }

private:
    using Vector2 = Vector<T, 2>;
    using State = typename DubinsPath<T>::State;
    static constexpr T STEP_SIZE = static_cast<T>(0.01);
    T lookahead_time;
    T min_radius;
    DubinsPath<T> path;
};