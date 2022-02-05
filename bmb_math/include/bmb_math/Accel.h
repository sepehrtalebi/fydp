#pragma once

#include <bmb_math/Vector3.h>
#include <geometry_msgs/Accel.h>

template<typename T>
struct Accel {
    Vector3<T> linear{};
    Vector3<T> angular{};

    Accel(const Vector3<T>& linear, const Vector3<T>& angular) : linear(linear), angular(angular) {}

    Accel(const geometry_msgs::Accel& msg) : linear(msg.linear), angular(msg.angular) {}

    void copy_to(geometry_msgs::Accel& msg) {
      linear.copy_to(msg.linear);
      angular.copy_to(msg.angular);
    }

    Accel<T> operator+(const Accel<T> &other) {
        return {Vector3<T>{linear + other.linear}, Vector3<T>{angular + other.angular}};
    }

    Accel<T> operator-(const Accel<T> &other) {
        return {Vector3<T>{linear - other.linear}, Vector3<T>{angular - other.angular}};
    }

    Accel<T> operator*(const T &scalar) {
        return {Vector3<T>{linear * scalar}, Vector3<T>{angular * scalar}};
    }

    Accel<T> operator/(const double &scalar) {
        return {Vector3<T>{linear / scalar}, Vector3<T>{angular / scalar}};
    }

    void operator+=(const Accel<T> &other) {
        linear += other.linear;
        angular += other.angular;
    }

    void operator-=(const Accel<T> &other) {
        linear -= other.linear;
        angular -= other.angular;
    }

    void operator*=(const T &scalar) {
        linear *= scalar;
        angular *= scalar;
    }

    void operator/=(const double &scalar) {
        linear /= scalar;
        angular /= scalar;
    }

    T &operator[](const size_t &index) {
        return index < 3 ? linear[index] : angular[index - 3];
    }

    const T &operator[](const size_t& index) const {
        return index < 3 ? linear[index] : angular[index - 3];
    }
};

template<typename T>
Accel<T> operator+(const T &scalar, const Accel<T> &accel) {
    // respect operator order in case underlying type is non-commutative
    return {scalar + accel.linear, scalar + accel.angular};
}

template<typename T>
Accel<T> operator-(const T &scalar, const Accel<T> &accel) {
    // respect operator order in case underlying type is non-commutative
    return {scalar - accel.linear, scalar - accel.angular};
}

template<typename T>
Accel<T> operator*(const T &scalar, const Accel<T> &accel) {
    // respect operator order in case underlying type is non-commutative
    return {scalar * accel.linear, scalar * accel.angular};
}

template<typename T>
Accel<T> operator/(const T &scalar, const Accel<T> &accel) {
    // respect operator order in case underlying type is non-commutative
    return {scalar / accel.linear, scalar / accel.angular};
}
