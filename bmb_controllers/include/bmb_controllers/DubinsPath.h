#pragma once

#include "../../../bmb_math/include/bmb_math/Vector.h"
#include "../../../bmb_math/include/bmb_math/Matrix.h"
#include "../../../bmb_utilities/include/bmb_utilities/MathUtils.h"
#include <array>
#include <cmath>
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
        union {
            struct { // only valid if is_turning
                Vector<T, 2> center;
                T radius;
                T start_angle;
                T delta_angle; //
            };
            struct { // only valid if !is_turning
                Vector<T, 2> start_pos;
                Vector<T, 2> end_pos;
            };
        };

        Curve(const Vector<T, 2> &center, const T &radius, const T &start_angle, const T &delta_angle) :
                is_turning(true), center(center), radius(radius),
                start_angle(start_angle), delta_angle(delta_angle) {}

        Curve(const Vector<T, 2> &start_pos, const Vector<T, 2> &end_pos) :
                is_turning(false), start_pos(start_pos), end_pos(end_pos) {}

        Curve(const Curve &other) {
            (*this) = other;
        }

        /**
         * This constructor will create this Curve with unspecified data inside of it.
         * It is the caller's responsibility to handle this.
         */
        Curve() {
            // cannot use = default to define this constructor because then it will be implicitly deleted
        };

        Curve &operator=(const Curve &other) {
            is_turning = other.is_turning;
            if (is_turning) {
                center = other.center;
                radius = other.radius;
                start_angle = other.start_angle;
                delta_angle = other.delta_angle;
            } else {
                start_pos = other.start_pos;
                end_pos = other.end_pos;
            }
            return *this;
        }

        [[nodiscard]] T getLength() const {
            return is_turning ? radius * std::fabs(delta_angle) : (start_pos - end_pos).magnitude();
        }

        /**
         * Return value is unspecified if !is_turning
         */
        [[nodiscard]] bool isRightTurn() const {
            return delta_angle < 0;
        }

        /**
         * @return The point on this Curve closest to the specified point
         */
        [[nodiscard]] Vector<T, 2> closestPoint(const Vector<T, 2> &p) const {
            using math_utils::getRotationMatrix;

            if (!is_turning) {
                Vector2 d = end_pos - start_pos;
                T projection_fraction = (p - start_pos).dot(d) / d.dot(d);
                if (projection_fraction < 0) return start_pos;
                if (projection_fraction > 1) return end_pos;
                return start_pos + projection_fraction * d;
            }

            // otherwise, we are turning
            Vector2 disp = p - center;
            T relative_angle = std::atan2(disp[1], disp[0]) - start_angle;
            if (isRightTurn()) {
                // remap angle to [-2 * PI, 0]
                if (relative_angle > 0) relative_angle -= 2 * M_PI;
                else if (relative_angle < -2 * M_PI) relative_angle += 2 * M_PI;

                if (relative_angle > delta_angle) {
                    // closest point on the circle to p is within the delta_angle range
                    return center + disp * (radius / disp.magnitude());
                }
                else if (relative_angle > delta_angle / 2 - M_PI) {
                    // closest point is the end point
                    return center + getRotationMatrix(start_angle + delta_angle) * Vector2{radius, 0};
                } else {
                    // closest point is the start point
                    return center + getRotationMatrix(start_angle) * Vector2{radius, 0};
                }
            } else {
                // remap angle to [0, 2 * PI]
                if (relative_angle > 2 * M_PI) relative_angle -= 2 * M_PI;
                else if (relative_angle < 0) relative_angle += 2 * M_PI;

                if (relative_angle < delta_angle) {
                    // closest point on the circle to p is within the delta_angle range
                    return center + disp * (radius / disp.magnitude());
                }
                else if (relative_angle < delta_angle / 2 + M_PI) {
                    // closest point is the end point
                    return center + getRotationMatrix(start_angle + delta_angle) * Vector2{radius, 0};
                } else {
                    // closest point is the start point
                    return center + getRotationMatrix(start_angle) * Vector2{radius, 0};
                }
            }
        }

        [[nodiscard]] std::string toStr() const {
            std::stringstream out;
            if (!is_turning) out << "Straight for: " << getLength() << "m";
            else
                out << "Turn " << ((delta_angle < 0) ? "right" : "left") << " for: " << std::fabs(delta_angle) << "deg";
            return out.str();
        }

        /**
         * @return The resulting State
         */
        template<typename OStream>
        void toCSV(OStream &out) const {
            if (!is_turning) {
                for (size_t i = 0; i < NUM_SAMPLES; i++) {
                    Vector<T, 2> pos = (static_cast<T>(NUM_SAMPLES - i) * start_pos + static_cast<T>(i) * end_pos) /
                                       static_cast<T>(NUM_SAMPLES);
                    out << pos[0] << "," << pos[1] << std::endl;
                }
            } else {
                for (size_t i = 0; i < NUM_SAMPLES; i++) {
                    T theta = start_angle + delta_angle * (i / static_cast<T>(NUM_SAMPLES));
                    Vector<T, 2> pos = center + math_utils::getRotationMatrix(theta) * Vector2{radius, 0};
                    out << pos[0] << "," << pos[1] << std::endl;
                }
            }
        }

    private:
        static constexpr size_t NUM_SAMPLES = 100;
    };

    DubinsPath(const DubinsPath<T>& other) {
        (*this) = other;
    }

    DubinsPath& operator=(const DubinsPath<T>& other) {
        for (size_t i = 0; i < 3; i++) path[i] = other[i];
        return *this;
    }

    static DubinsPath create(const State &start, const State &goal, const T &radius) {
        // Based on: https://gieseanw.wordpress.com/2012/10/21/a-comprehensive-step-by-step-tutorial-to-computing-dubins-paths/

        // start with the shorter between the RSR and LSL path, since they are always possible
        DubinsPath best_path = std::min(getCSCOuterTangent(start, goal, radius, true), // RSR
                                        getCSCOuterTangent(start, goal, radius, false)); // LSL

        // try RSL path
        auto[is_valid, path] = getCSCInnerTangent(start, goal, radius, true);
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
        return path[0].getLength() + path[1].getLength() + path[2].getLength();
    }

    bool operator<(const DubinsPath<T> &other) const {
        return getLength() < other.getLength();
    }

    Vector<T, 2> closestPoint(const Vector<T, 2> &p) const {
        return std::min({path[0].closestPoint(p), path[1].closestPoint(p), path[2].closestPoint(p)},
                        [&p](const Vector2 &first, const Vector2 &second) {
                            return (first - p).magnitudeSquared() < (second - p).magnitudeSquared();
                        });
    }

    [[nodiscard]] std::string toStr() const {
        std::stringstream out;
        out << path[0].toStr() << ", " << path[1].toStr() << ", " << path[2].toStr();
        return out.str();
    }

    template<typename OStream>
    void toCSV(OStream &out) const {
        for (size_t i = 0; i < 3; i++) path[i].toCSV(out);
    }

private:
    using Vector2 = Vector<T, 2>;
    using Path = std::array<Curve, 3>;
    static const Matrix<T, 2, 2> ROT_90_CW;
    static const Matrix<T, 2, 2> ROT_90_CCW;
    using iterator = typename Path::iterator;
    using const_iterator = typename Path::const_iterator;

    Path path;

    DubinsPath() = default;

    explicit DubinsPath(const Path &path) : path(path) {}

    iterator begin() {
        return path.begin();
    }

    iterator end() {
        return path.end();
    }

    const_iterator begin() const {
        return path.begin();
    }

    const_iterator end() const {
        return path.end;
    }

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
        Vector2 normal =
                (radius / d) * ((right ? ROT_90_CCW : ROT_90_CW) * v1); // extra brackets to reduce multiplications
        Vector2 p1_tangent = p1 + normal;
        Vector2 p2_tangent = p2 + normal;

        auto[start_angle, delta_angle] = getArcAngle(start.pos - p1, normal, right);
        Curve c1{p1, radius, start_angle, delta_angle};
        Curve c2{p1_tangent, p2_tangent};
        std::tie(start_angle, delta_angle) = getArcAngle(normal, goal.pos - p2, right);
        Curve c3{p2, radius, start_angle, delta_angle};
        return DubinsPath{{c1, c2, c3}};
    }

    /**
     *
     * @param start
     * @param goal
     * @param radius
     * @param right If true, then the RSL path will be returned, otherwise the LSR path will be returned.
     * @return
     */
    static std::pair<bool, DubinsPath>
    getCSCInnerTangent(const State &start, const State &goal, const T &radius, const bool &right) {
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

        auto[start_angle, delta_angle] = getArcAngle(start.pos - p1, normal, right);
        Curve c1{p1, radius, start_angle, delta_angle};
        Curve c2{p1_tangent, p2_tangent};
        std::tie(start_angle, delta_angle) = getArcAngle(-normal, goal.pos - p2, !right);
        Curve c3{p2, radius, start_angle, delta_angle};
        return {true, DubinsPath{{c1, c2, c3}}};
    }

    /**
     *
     * @param start
     * @param goal
     * @param radius
     * @param right If true, then the RLR path will be returned, otherwise the LRL path will be returned.
     * @return
     */
    static std::pair<bool, DubinsPath>
    getCCC(const State &start, const State &goal, const T &radius, const bool &right) {
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

        auto[start_angle, delta_angle] = getArcAngle(start.pos - p1, p_first_stop - p1, right);
        Curve c1{p1, radius, start_angle, delta_angle};
        std::tie(start_angle, delta_angle) = getArcAngle(p_first_stop - p3, p_second_stop - p3, !right);
        Curve c2{p3, radius, start_angle, delta_angle};
        std::tie(start_angle, delta_angle) = getArcAngle(p_second_stop - p2, goal.pos - p2, right);
        Curve c3{p2, radius, start_angle, delta_angle};
        return {true, DubinsPath{{c1, c2, c3}}};
    }

    static Vector2 getCenter(const State &state, const T &radius, const bool &right) {
        Vector2 normal = (right ? ROT_90_CW : ROT_90_CCW) * state.vel;
        normal *= radius / normal.magnitude();
        return state.pos + normal;
    }

    /**
     * The start angle will be in the range [-pi, pi]
     * delta_angle will be in the range [-2 * pi, 2 * pi], where positive indicates counterclockwise (turning left)
     * The returned total angle traversed will be positive if it is a left turn, and negative if it is a right turn.
     * @return A std::pair consisting of the starting angle and the total angle traversed
     */
    static std::pair<T, T> getArcAngle(const Vector2 &start, const Vector2 &end, const bool &right) {
        T start_angle = std::atan2(start[1], start[0]);
        T delta_angle = std::atan2(end[1], end[0]) - start_angle;
        if (right) {
            return {start_angle, delta_angle < 0 ? delta_angle : delta_angle - 2 * M_PI};
        } else {
            return {start_angle, delta_angle > 0 ? delta_angle : delta_angle + 2 * M_PI};
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
