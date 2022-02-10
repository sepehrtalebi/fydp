#include <bmb_math/Vector3.h>
#include <gtest/gtest.h>

TEST(TestVector3, testVector3) {
  Vector3<int> first{1, 2, 3};
  Vector3<int> second;
  second = first;
  second.x = 4;
  ASSERT_EQ(first.x, 1);
  ASSERT_EQ(second.x, 4);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
