#include <bmb_utilities/ConstexprUtils.h>
#include <gtest/gtest.h>
#include <cstddef>

template <size_t n>
size_t getValue() {
  return n;
}

TEST(TestConstexprUtils, testConstexprFor) {
  using namespace bmb_utilities;
  static constexpr size_t SIZE = 10;
  size_t sum = 0;
  constexprFor<0, SIZE>([&](auto i) { sum += getValue<i>(); });
  ASSERT_EQ(sum, SIZE * (SIZE - 1) / 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
