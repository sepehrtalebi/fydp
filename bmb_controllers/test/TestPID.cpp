#include <bmb_math/TransferFunction.h>

#include <gtest/gtest.h>

TEST(TestPID, testPID) {
    TransferFunction<double, 1, 3> {1, 1, 2, 3};
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
