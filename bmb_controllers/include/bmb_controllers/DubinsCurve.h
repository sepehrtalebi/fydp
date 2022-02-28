#pragma once

#include <bmb_math/Utility.h>
#include <bmb_math/Vector.h>
#include <cmath>
#include <sstream>
#include <string>

template <typename T>
struct DubinsCurve {
  bool is_turning;
  union {
    struct {  // only valid if is_turning
      Vector<T, 2> center;
      T radius;
      T start_angle;
      T delta_angle;  //
    };
    struct {  // only valid if !is_turning
      Vector<T, 2> start_pos;
      Vector<T, 2> end_pos;
    };
  };

  DubinsCurve(const Vector<T, 2>& center, const T& radius, const T& start_angle,
              const T& delta_angle)
      : is_turning(true),
        center(center),
        radius(radius),
        start_angle(start_angle),
        delta_angle(delta_angle) {}

  DubinsCurve(const Vector<T, 2>& start_pos, const Vector<T, 2>& end_pos)
      : is_turning(false), start_pos(start_pos), end_pos(end_pos) {}

  DubinsCurve(const DubinsCurve<T>& other) { (*this) = other; }

  /**
   * This constructor will create this DubinsCurve with unspecified data inside
   * of it. It is the caller's responsibility to handle this.
   */
  DubinsCurve(){
      // cannot use = default to define this constructor because then it will
      // be implicitly deleted
  };

  DubinsCurve& operator=(const DubinsCurve<T>& other) {
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
    return is_turning ? radius * std::fabs(delta_angle)
                      : (start_pos - end_pos).magnitude();
  }

  /**
   * Return value is unspecified if !is_turning
   */
  [[nodiscard]] bool isRightTurn() const { return delta_angle < 0; }

  /**
   * @return The point on this DubinsCurve closest to the specified point
   */
  [[nodiscard]] Vector<T, 2> closestPoint(const Vector<T, 2>& p) const {
    using bmb_math::getRotationMatrix;

    if (!is_turning) {
      const Vector2 d = end_pos - start_pos;
      const T projection_fraction = (p - start_pos).dot(d) / d.dot(d);
      if (projection_fraction < 0) return start_pos;
      if (projection_fraction > 1) return end_pos;
      return start_pos + projection_fraction * d;
    }

    // otherwise, we are turning
    const Vector2 disp = p - center;
    T relative_angle = std::atan2(disp[1], disp[0]) - start_angle;
    if (isRightTurn()) {
      // remap angle to [-2 * PI, 0]
      if (relative_angle > 0)
        relative_angle -= 2 * M_PI;
      else if (relative_angle < -2 * M_PI)
        relative_angle += 2 * M_PI;

      if (relative_angle > delta_angle) {
        // closest point on the circle to p is within the delta_angle range
        return center + disp * (radius / disp.magnitude());
      } else if (relative_angle > delta_angle / 2 - M_PI) {
        // closest point is the end point
        return center + getRotationMatrix(start_angle + delta_angle) *
                            Vector2{radius, 0};
      } else {
        // closest point is the start point
        return center + getRotationMatrix(start_angle) * Vector2{radius, 0};
      }
    } else {
      // remap angle to [0, 2 * PI]
      if (relative_angle > 2 * M_PI)
        relative_angle -= 2 * M_PI;
      else if (relative_angle < 0)
        relative_angle += 2 * M_PI;

      if (relative_angle < delta_angle) {
        // closest point on the circle to p is within the delta_angle range
        return center + disp * (radius / disp.magnitude());
      } else if (relative_angle < delta_angle / 2 + M_PI) {
        // closest point is the end point
        return center + getRotationMatrix(start_angle + delta_angle) *
                            Vector2{radius, 0};
      } else {
        // closest point is the start point
        return center + getRotationMatrix(start_angle) * Vector2{radius, 0};
      }
    }
  }

  [[nodiscard]] std::string toStr() const {
    std::stringstream out;
    if (!is_turning)
      out << "Straight for: " << getLength() << "m";
    else
      out << "Turn " << ((delta_angle < 0) ? "right" : "left")
          << " for: " << std::fabs(delta_angle) << "deg";
    return out.str();
  }

  template <typename OStream>
  void toCSV(OStream& out) const {
    if (!is_turning) {
      for (size_t i = 0; i < NUM_SAMPLES; i++) {
        const Vector<T, 2> pos = (static_cast<T>(NUM_SAMPLES - i) * start_pos +
                                  static_cast<T>(i) * end_pos) /
                                 static_cast<T>(NUM_SAMPLES);
        out << pos[0] << "," << pos[1] << std::endl;
      }
    } else {
      for (size_t i = 0; i < NUM_SAMPLES; i++) {
        const T theta =
            start_angle + delta_angle * (i / static_cast<T>(NUM_SAMPLES));
        const Vector<T, 2> pos =
            center + bmb_math::getRotationMatrix(theta) * Vector2{radius, 0};
        out << pos[0] << "," << pos[1] << std::endl;
      }
    }
  }

 private:
  using Vector2 = Vector<T, 2>;
  static constexpr size_t NUM_SAMPLES = 100;
};