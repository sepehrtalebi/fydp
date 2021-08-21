#pragma once

#include "Vector.h"

class Vector3 : public Vector<3> {
public:
    double &x = data[0];
    double &y = data[1];
    double &z = data[2];
public:
    Vector3() = default;

    Vector3(double x, double y, double z);

    Vector3(const Vector3 &other);

    explicit Vector3(Vector<3> vec);

    Vector3 cross(const Vector3 &other) const;
};
