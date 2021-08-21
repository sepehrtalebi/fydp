#include "Quaternion.h"

Quaternion::Quaternion(double q0, double q1, double q2, double q3) {
    this->q0 = q0;
    this->q1 = q1;
    this->q2 = q2;
    this->q3 = q3;
}

Quaternion::Quaternion(Vector<4> vec) : Quaternion(vec[0], vec[1], vec[2], vec[3]) {}

Quaternion Quaternion::cong() const {
    return Quaternion{this->q0, this->q1, this->q2, this->q3};
}

Vector3 Quaternion::rotate(const Vector3 &vec) const {
    Quaternion x_quat = Quaternion{0, vec.x, vec.y, vec.z};
    Quaternion x_rot_quat = this->cong() * x_quat * (*this);
    return Vector3{x_rot_quat.q1,
                   x_rot_quat.q2,
                   x_rot_quat.q3};
}

Vector3 Quaternion::unrotate(const Vector3 &vec) const {
    return this->cong().rotate(vec);
}

Matrix<3, 4> Quaternion::E() const {
    Matrix<3, 4>matrix{};

    matrix[0][0] = -q1;
    matrix[0][1] = q0;
    matrix[0][2] = -q3;
    matrix[0][3] = q2;

    matrix[1][0] = -q2;
    matrix[1][1] = q3;
    matrix[1][2] = q0;
    matrix[1][3] = -q1;

    matrix[2][0] = -q3;
    matrix[2][1] = -q2;
    matrix[2][2] = q1;
    matrix[2][3] = q0;

    return matrix;
}

Matrix<3, 4> Quaternion::G() const {
    Matrix<3, 4> matrix{};

    matrix[0][0] = -q1;
    matrix[0][1] = q0;
    matrix[0][2] = q3;
    matrix[0][3] = -q2;

    matrix[1][0] = -q2;
    matrix[1][1] = -q3;
    matrix[1][2] = q0;
    matrix[1][3] = q1;

    matrix[2][0] = -q3;
    matrix[2][1] = q2;
    matrix[2][2] = -q1;
    matrix[2][3] = q0;

    return matrix;
}

Vector3 Quaternion::toEulerAngles() const {
    // TODO
    return Vector3{};
}

Matrix<3, 3> Quaternion::toDCM() const {
    // TODO
    return Matrix<3, 3>::identity();
}

Quaternion Quaternion::operator*(const Quaternion &other) const {
    return Quaternion{q0 * other.q0 - q1 * other.q1 - q2 * other.q2 - q3 * other.q3,
                      q0 * other.q1 + other.q0 * q1 + q2 * other.q3 - q3 * other.q2,
                      q0 * other.q2 + other.q0 * q2 + q3 * other.q1 - q1 * other.q3,
                      q0 * other.q3 + other.q0 * q3 + q1 * other.q2 - q2 * other.q1};
}

void Quaternion::operator*=(const Quaternion &other) {
    Quaternion product = (*this) * other;
    q0 = product.q0;
    q1 = product.q1;
    q2 = product.q2;
    q3 = product.q3;
}

Quaternion Quaternion::operator/(const Quaternion &other) const {
    return (*this) * other.cong();
}

void Quaternion::operator/=(const Quaternion &other) {
    Quaternion quotient = (*this) / other;
    q0 = quotient.q0;
    q1 = quotient.q1;
    q2 = quotient.q2;
    q3 = quotient.q3;
}
