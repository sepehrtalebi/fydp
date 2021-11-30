#pragma once

#include "Vector.h"
#include "Matrix.h"
#include <array>
#include <vector>
#include <algorithm>
#include <string>
#include <sstream>

template<typename T>
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
};

template<typename T>
struct State {
    Vector<T, 2> pos;
    Vector<T, 2> vel;
};

template<typename T>
class DubinsPath {
public:
    static DubinsPath create(const State<T> &start, const State<T> &goal, const T &radius) {
        // start with the shorter between the RSR and LSL path, since they are always possible
        DubinsPath best_path = std::min(getRSR(start, goal, radius), getLSL(start, goal, radius));

        // try RSL path
        DubinsPath rsl = getRSR(start, goal, radius);
        if (rsl) best_path = std::min(best_path, rsl);

        // try LSR path

        // try RLR path

        // try LRL path

        return best_path;
    }

    const Curve<T> &operator[](const size_t &index) const {
        return path[index];
    }

    Curve<T> &operator[](const size_t &index) {
        return path[index];
    }

    T getLength() const {
        return path[0].length + path[1].length + path[2].length;
    }

    bool operator<(const DubinsPath<T> &other) const {
        return getLength() < other.getLength();
    }

    explicit operator bool() const {
        return is_valid;
    }

    [[nodiscard]] std::string toStr() const {
        if (!is_valid) return "Invalid!";
        std::stringstream out;
        out << path[0].toStr() << ", " << path[1].toStr() << ", " << path[2].toStr();
        return out.str();
    }

private:
    using Vector2 = Vector<T, 2>;
    using Path = std::array<Curve<T>, 3>;
    static const Matrix<T, 2, 2> ROT_90_CW;
    static const Matrix<T, 2, 2> ROT_90_CCW;
    static const Matrix<T, 2, 2> ROT_180;

    std::array<Curve<T>, 3> path;
    bool is_valid = true;

    DubinsPath() : is_valid(false) {}

    explicit DubinsPath(const Path &p) : is_valid(true), path(p) {}

    static DubinsPath getRSR(const State<T> &start, const State<T> &goal, const T &radius) {
        Vector2 p1 = getCenter(start, radius, true);
        Vector2 p2 = getCenter(goal, radius, true);
        Vector2 v1 = p2 - p1;

        T d = v1.magnitude();
        Vector2 normal = (radius / d) * (ROT_90_CCW * v1); // extra brackets to reduce multiplications
        Vector2 p1_tangent = p1 + normal;
        Vector2 p2_tangent = p2 + normal;

        return DubinsPath{{Curve<T>{true, true, radius * getArcAngle(start.pos - p1, normal, true)},
                           Curve<T>{false, false, d},
                           Curve<T>{true, true, radius * getArcAngle(goal.pos - p2, normal, true)}}};
    }

    static DubinsPath getLSL(const State<T> &start, const State<T> &goal, const T &radius) {
        Vector2 p1 = getCenter(start, radius, false);
        Vector2 p2 = getCenter(goal, radius, false);
        Vector2 v1 = p2 - p1;

        T d = v1.magnitude();
        Vector2 normal = (radius / d) * (ROT_90_CW * v1); // extra brackets to reduce multiplications
        Vector2 p1_tangent = p1 + normal;
        Vector2 p2_tangent = p2 + normal;

        return DubinsPath{{Curve<T>{true, false, radius * getArcAngle(start.pos - p1, normal, false)},
                           Curve<T>{false, false, d},
                           Curve<T>{true, false, radius * getArcAngle(goal.pos - p2, normal, false)}}};
    }

    static DubinsPath getRSL(const State<T> &start, const State<T> &goal, const T &radius) {
        Vector2 p1 = getCenter(start, radius, true);
        Vector2 p2 = getCenter(goal, radius, false);
        Vector2 v1 = p2 - p1;

        T D = v1.magnitude();
        if (true || D < 2 * radius) {
            // not possible to create RSL trajectory, return invalid DubinsPath
            return DubinsPath{};
        }
        // TODO: implement
        Vector2 normal = (radius / D) * (ROT_90_CW * v1); // extra brackets to reduce multiplications
        Vector2 p1_tangent = p1 + normal;
        Vector2 p2_tangent = p2 + normal;

        return DubinsPath{{Curve<T>{true, true, radius * getArcAngle(start.pos - p1, normal, true)},
                           Curve<T>{false, false, D},
                           Curve<T>{true, false, radius * getArcAngle(goal.pos - p2, normal, false)}}};
    }

    static Vector2 getCenter(const State<T> &state, const T &radius, const bool &right) {
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
template<typename T>
const Matrix<T, 2, 2> DubinsPath<T>::ROT_180{-1, 0,
                                             0, -1};
