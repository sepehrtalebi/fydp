#pragma once

#include "Vector.h"
#include "Matrix.h"
#include "MathUtils.h"
#include <array>
#include <vector>
#include <algorithm>
#include <string>
#include <sstream>
#include <utility>
#include <tuple>
#include <fstream>

template<typename T>
class DubinsPath {
public:
    struct State {
        Vector<T, 2> pos;
        Vector<T, 2> vel;
    };

    struct Curve {
        bool is_turning;
        bool turn_right; // only valid if is_turning is true
        T length;

        [[nodiscard]] std::string toStr() const {
            std::stringstream out;
            if (!is_turning) out << "Straight for: ";
            else out << "Turn " << (turn_right ? "right" : "left") << " for: ";
            out << length;
            return out.str();
        }

        /**
         * @param start_state Must have a velocity with a magnitude of 1.
         * @return The resulting State
         */
         template<typename OStream>
        State toCSV(OStream& out, const State& start_state, const T& radius) const {
            if (!is_turning) {
                for (int i = 0; i < 100; i++) {
                    Vector<T, 2> pos = start_state.pos + (length * (i / 100.0)) * start_state.vel;
                    out << pos[0] << "," << pos[1] << std::endl;
                }
                return {start_state.pos + length * start_state.vel, start_state.vel};
            }

            Vector<T, 2> center = getCenter(start_state, radius, turn_right);
            T delta_theta = length / radius;
            Vector<T, 2> displacement_0 = start_state.pos - center;
            for (int i = 0; i < 100; i++) {
                T theta = (turn_right ? -1 : 1) * delta_theta * (i / 100.0);
                Vector<T, 2> pos = center + getRotationMatrix(theta) * displacement_0;
                out << pos[0] << "," << pos[1] << std::endl;
            }
            Matrix<T, 2, 2> rot = getRotationMatrix(delta_theta);
            return {center + rot * displacement_0, rot * start_state.vel};
        }
    };

    static DubinsPath create(const State &start, const State &goal, const T &radius) {
        // Based on: https://gieseanw.wordpress.com/2012/10/21/a-comprehensive-step-by-step-tutorial-to-computing-dubins-paths/

        // start with the shorter between the RSR and LSL path, since they are always possible
        DubinsPath best_path = std::min(getCSCOuterTangent(start, goal, radius, true), // RSR
                                        getCSCOuterTangent(start, goal, radius, false)); // LSL

        // try RSL path
        auto [is_valid, path] = getCSCInnerTangent(start, goal, radius, true);
        if (is_valid) best_path = std::min(best_path, path);

        // try LSR path
        std::tie(is_valid, path) = getCSCInnerTangent(start, goal, radius, false);
        if (is_valid) best_path = std::min(best_path, path);

        // try RLR path
        std::tie(is_valid, path) = getCCC(start, goal, radius, true);
        if (is_valid) best_path = std::min(best_path, path);

        // try LRL path
        std::tie(is_valid, path) = getCCC(start, goal, radius, false);
        if (is_valid) best_path = std::min(best_path, path);

        return best_path;
    }

    const Curve &operator[](const size_t &index) const {
        return path[index];
    }

    Curve &operator[](const size_t &index) {
        return path[index];
    }

    T getLength() const {
        return path[0].length + path[1].length + path[2].length;
    }

    bool operator<(const DubinsPath<T> &other) const {
        return getLength() < other.getLength();
    }

    [[nodiscard]] std::string toStr() const {
        std::stringstream out;
        out << path[0].toStr() << ", " << path[1].toStr() << ", " << path[2].toStr();
        return out.str();
    }

    template<typename OStream>
    void toCSV(OStream& out, const State& start, const T& radius) const {
        State state = path[0].toCSV(out, start, radius);
        state = path[1].toCSV(out, state, radius);
        path[2].toCSV(out, state, radius);
    }

private:
    using Vector2 = Vector<T, 2>;
    using Path = std::array<Curve, 3>;
    static const Matrix<T, 2, 2> ROT_90_CW;
    static const Matrix<T, 2, 2> ROT_90_CCW;

    std::array<Curve, 3> path;

    DubinsPath() = default;

    explicit DubinsPath(const Path &path) : path(path) {}

    /**
     *
     * @param start
     * @param goal
     * @param radius
     * @param right If true, then the RSR path will be returned, otherwise the LSL path will be returned.
     * @return
     */
    static DubinsPath getCSCOuterTangent(const State &start, const State &goal, const T &radius, const bool &right) {
        Vector2 p1 = getCenter(start, radius, right);
        Vector2 p2 = getCenter(goal, radius, right);
        Vector2 v1 = p2 - p1;

        T d = v1.magnitude();
        Vector2 normal = (radius / d) * ((right ? ROT_90_CCW : ROT_90_CW) * v1); // extra brackets to reduce multiplications
        Vector2 p1_tangent = p1 + normal;
        Vector2 p2_tangent = p2 + normal;

        return DubinsPath{{Curve{true, right, radius * getArcAngle(start.pos - p1, normal, right)},
                           Curve{false, false, d},
                           Curve{true, right, radius * getArcAngle(goal.pos - p2, normal, right)}}};
    }

    /**
     *
     * @param start
     * @param goal
     * @param radius
     * @param right If true, then the RSL path will be returned, otherwise the LSR path will be returned.
     * @return
     */
    static std::pair<bool, DubinsPath> getCSCInnerTangent(const State &start, const State &goal, const T &radius, const bool &right) {
        Vector2 p1 = getCenter(start, radius, right);
        Vector2 p2 = getCenter(goal, radius, !right);
        Vector2 v1 = p2 - p1;

        T d = v1.magnitude();
        if (d < 2 * radius) {
            // not possible to create RSL trajectory since the circles intersect, return invalid DubinsPath
            return {false, DubinsPath{}};
        }
        T cos = 2 * radius / d;
        T sin = std::sqrt(1 - cos * cos);
        Matrix<T, 2, 2> rot{cos, -sin,
                            sin, cos};
        if (!right) rot = rot.transpose(); // transposing rotation matrix is the same as taking the inverse
        Vector2 normal = (radius / d) * (rot * v1); // extra brackets to reduce multiplications
        Vector2 p1_tangent = p1 + normal;
        Vector2 p2_tangent = p2 - normal;

        return {true, DubinsPath{{Curve{true, right, radius * getArcAngle(start.pos - p1, normal, right)},
                                  Curve{false, false, d},
                                  Curve{true, !right, radius * getArcAngle(goal.pos - p2, -normal, !right)}}}};
    }

    /**
     *
     * @param start
     * @param goal
     * @param radius
     * @param right If true, then the RLR path will be returned, otherwise the LRL path will be returned.
     * @return
     */
    static std::pair<bool, DubinsPath> getCCC(const State &start, const State &goal, const T &radius, const bool &right) {
        Vector2 p1 = getCenter(start, radius, right);
        Vector2 p2 = getCenter(goal, radius, right);
        Vector2 v1 = p2 - p1;

        T d = v1.magnitude();
        if (d >= 4 * radius) {
            // not possible to create RLR trajectory, return invalid DubinsPath
            // technically, if d >= 4 * radius then a RLR trajectory is possible, however it will always be suboptimal
            return {false, DubinsPath{}};
        }
        T h = std::sqrt(4 * radius * radius - d * d / 4);
        Vector2 normal = (h / d) * ((right ? ROT_90_CCW : ROT_90_CW) * v1);
        Vector2 p3 = p1 + v1 / 2 + normal;

        Vector2 p_first_stop = (p1 + p3) / 2;
        Vector2 p_second_stop = (p2 + p3) / 2;
        return {true, DubinsPath{{Curve{true, right, radius * getArcAngle(start.pos - p1, p_first_stop - p1, right)},
                                  Curve{true, !right, radius * getArcAngle(p_first_stop - p3, p_second_stop - p3, !right)},
                                  Curve{true, right, radius * getArcAngle(p_second_stop - p2, goal.pos - p2, right)}}}};
    }

    static Vector2 getCenter(const State &state, const T &radius, const bool &right) {
        Vector2 normal = (right ? ROT_90_CW : ROT_90_CCW) * state.vel;
        normal *= radius / normal.magnitude();
        return state.pos + normal;
    }

    static T getArcAngle(const Vector2 &start, const Vector2 &end, const bool &right) {
        T theta = std::atan2(end[1], end[0]) - std::atan2(start[1], start[0]);
        if (right) {
            return theta < 0 ? -theta : 2 * M_PI - theta;
        } else {
            return theta > 0 ? theta : theta + 2 * M_PI;
        }
    }
};

// need to define constant matrices here since they cannot be declared constexpr
template<typename T>
const Matrix<T, 2, 2> DubinsPath<T>::ROT_90_CW{0, 1,
                                               -1, 0};
template<typename T>
const Matrix<T, 2, 2> DubinsPath<T>::ROT_90_CCW{0, -1,
                                                1, 0};
