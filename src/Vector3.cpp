#include "Vector3.h"

Vector3::Vector3(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

Vector3::Vector3(const Vector3 &other) : Vector3(other.x, other.y, other.z) {}

Vector3::Vector3(Vector<3> vec) : Vector3(vec[0], vec[1], vec[2]) {}

Vector3 Vector3::cross(const Vector3 &other) const {
    return Vector3{this->y * other.z - this->z * other.y,
                   this->z * other.x - this->x * other.z,
                   this->x * other.y - this->y * other.x
    };
}
