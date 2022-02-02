#pragma once

#include <cstddef>

namespace bmb_math {

static constexpr size_t slice_count(const size_t& start, const size_t& stop,
                                    const size_t& step = 1) {
  return (stop - start + step - 1) / step;
}

}  // namespace bmb_math
