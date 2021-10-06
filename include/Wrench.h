#pragma once

#include "Vector3.h"

template<typename T>
struct Wrench {
    Vector3<T> force;
    Vector3<T> torque;

    Wrench<T> operator+(const Wrench<T> &other) {
        return Wrench<T>{force + other.force, torque + other.torque};
    }

    // TODO: add other operators
};
