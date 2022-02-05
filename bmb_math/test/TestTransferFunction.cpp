#include <bmb_math/TransferFunction.h>
#include <bmb_math/RationalFunction.h>

#include <gtest/gtest.h>
#include <fstream>

const double EPSILON = 1E-4;

template<typename T>
constexpr bool AreSame(T a, T b)
{
    return abs_difference(a, b) < EPSILON;
}

TEST(TestTransferFunction, testTransferFunction) {
    TransferFunction<double, 1, 3> P{4, 1, 2, 3};
    TransferFunction<double, 4, 2> C{3, 4, 5, 1, 4, 1};
    TransferFunction<double, 4, 4> CP = C*P;
    CP.print();
    TransferFunction<double, 4, 4> truth{12, 16, 20, 4, 4, 9, 14, 3};
    for (int i = 0; i < 4; i++)
        ASSERT_EQ(CP.numerator_data(i), truth.numerator_data(i));
    for (int i = 0; i < 4; i++)
        ASSERT_EQ(CP.denominator_data(i), truth.denominator_data(i));
    TransferFunction<double, 2, 2> C2{2.6354*69.5, 2.6354, 38, 1};
    auto fbl = C2.feedbackLoop();
    fbl.print();
    TransferFunction<double, 2, 2> truth1{2.6354*69.5, 2.6354, 2.6354*69.5+38, 3.6354};
    for (int i = 0; i < 2; i++)
        ASSERT_EQ(fbl.numerator_data(i), truth1.numerator_data(i));
    for (int i = 0; i < 2; i++)
        ASSERT_EQ(fbl.denominator_data(i), truth1.denominator_data(i));
    auto D = fbl.discretize();
    D.print();
    TransferFunction<double, 2, 2> truth2{-0.7202, 0.7252, -0.9939, 1};
    for (int i = 0; i < 2; i++)
      ASSERT_TRUE(AreSame(D.numerator_data(i), truth2.numerator_data(i)));
    for (int i = 0; i < 2; i++)
      ASSERT_TRUE(AreSame(D.denominator_data(i), truth2.denominator_data(i)));
    Vector<double, 100000> output = D.step<100000>();
    std::ofstream out("test/output/step_response.csv");
    ASSERT_TRUE(out);
    output.toCSV(out);
    out.close();
};

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}