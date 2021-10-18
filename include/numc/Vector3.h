#pragma once

#include "Vector.h"

template<typename T>
class Vector3 : public Vector<T, 3> {
public:
    T &x = this->data[0];
    T &y = this->data[1];
    T &z = this->data[2];

public:
    Vector3() = default;

    Vector3(T x, T y, T z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    Vector3(const Vector3<T> &other) : Vector3(other.x, other.y, other.z) {}

    // allow implicit conversions
    Vector3(Vector<T, 3> vec) : Vector3(vec[0], vec[1], vec[2]) {} // NOLINT(google-explicit-constructor)

    Vector3<T> cross(const Vector3<T> &other) const {
        return Vector3{y * other.z - z * other.y,
                       z * other.x - x * other.z,
                       x * other.y - y * other.x};
    }
};
