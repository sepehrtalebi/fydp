#pragma once

#include "Vector3.h"

template<typename T>
struct Wrench {
    Vector3<T> force;
    Vector3<T> torque;

    Wrench<T> operator+(const Wrench<T> &other) {
        return {Vector3<T>{force + other.force}, Vector3<T>{torque + other.torque}};
    }

    Wrench<T> operator-(const Wrench<T> &other) {
        return {Vector3<T>{force - other.force}, Vector3<T>{torque - other.torque}};
    }

    Wrench<T> operator*(const T &scalar) {
        return {Vector3<T>{force * scalar}, Vector3<T>{torque * scalar}};
    }

    Wrench<T> operator/(const double &scalar) {
        return {Vector3<T>{force / scalar}, Vector3<T>{torque / scalar}};
    }

    void operator+=(const Wrench<T> &other) {
        force += other.force;
        torque += other.torque;
    }

    void operator-=(const Wrench<T> &other) {
        force -= other.force;
        torque -= other.torque;
    }

    void operator*=(const T &scalar) {
        force *= scalar;
        torque *= scalar;
    }

    void operator/=(const double &scalar) {
        force /= scalar;
        torque /= scalar;
    }
};
