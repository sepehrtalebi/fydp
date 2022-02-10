#pragma once

#include <cstddef>
#include <type_traits>

namespace bmb_utilities {

/**
 * Unrolls a loop over the specified values.
 * The passed in function should be a lambda with a single argument "auto i"
 * where i is the loop variable which is constexpr.
 */
template <int start, int stop, int step = 1, class F>
constexpr void constexprFor(F&& f) {
  static_assert(step != 0);

  if constexpr (step > 0 && start < stop || step < 0 && start > stop) {
    f(std::integral_constant<size_t, start>());
    constexprFor<start + step, stop, step>(f);
  }
}

}  // namespace bmb_utilities
