#pragma once

#include "Matrix.h"
#include <cmath>

template<typename T>
Matrix<T, 2, 2> getRotationMatrix(const T& theta) {
    T sin = std::sin(theta);
    T cos = std::cos(theta);
    return {cos, -sin,
            sin, cos}
}
