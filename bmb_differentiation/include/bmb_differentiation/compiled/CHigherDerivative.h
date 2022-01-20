#pragma once

#include "CDerivative.h"

#include <cstddef>

namespace compiled {

template <size_t id, size_t order, typename T>
struct higher_derivative
    : higher_derivative<id, order - 1, derivative_t<id, T>> {};

template <size_t id, typename T>
struct higher_derivative<id, 0, T> {
  using type = T;
};

template <size_t id, size_t order, typename T>
using higher_derivative_t = typename higher_derivative<id, order, T>::type;

}  // namespace compiled
