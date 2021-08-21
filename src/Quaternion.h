#pragma once

#include "Vector3.h"
#include "Matrix.h"

class Quaternion : public Vector<4> {
public:
    double &q0 = data[0];
    double &q1 = data[1];
    double &q2 = data[2];
    double &q3 = data[3];
public:
    explicit Quaternion(double q0 = 1, double q1 = 0, double q2 = 0, double q3 = 0);

    explicit Quaternion(Vector<4> vec);

    Quaternion cong() const;

    Vector3 rotate(const Vector3 &vec) const;

    Vector3 unrotate(const Vector3 &vec) const;

    Matrix<3, 4> E() const;

    Matrix<3, 4> G() const;

    Vector3 toEulerAngles() const;

    Matrix<3, 3> toDCM() const;

    // OPERATOR OVERLOADING:

    Quaternion operator*(const Quaternion &other) const;

    void operator*=(const Quaternion &other);

    Quaternion operator/(const Quaternion &other) const;

    void operator/=(const Quaternion &other);
};
