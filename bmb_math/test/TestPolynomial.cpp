#include <bmb_math/Polynomial.h>

#include <gtest/gtest.h>
#include <array>

//test constructors and operators *= += *
TEST(TestPolynomial, testPolynomial) {
    Polynomial<double, 3> poly{4, 3, 2};
    for (int i = 0; i < 3; i++)
        ASSERT_EQ(poly[i], double(4 - i));
    auto test = poly * poly;
    std::array<double, 5> truth{16, 24, 25, 12, 4};
    for (int i = 0; i < 5; i++)
        ASSERT_EQ(test[i], truth[i]);
    test *= 2;
    for (int i = 0; i < 5; i++)
        ASSERT_EQ(test[i], 2 * truth[i]);
    test += Polynomial<double, 5> {truth};
    test.print();
    for (int i = 0; i < 5; i++)
        ASSERT_EQ(test[i], 3 * truth[i]);
}

//test * edge cases
TEST(TestPolynomial, testEdgeCases) {
    Polynomial<double, 3> poly{4, 0, 3};
    Polynomial<double, 3> other{2, 0, 1};
    Polynomial<double, 5> test = poly * other;
    std::array<double, 5> truth{8, 0, 10, 0, 3};
    for (int i = 0; i < 5; i++)
        ASSERT_EQ(test[i], truth[i]);
    test += other;
    truth = {10, 0, 11, 0, 3};
    for (int i = 0; i < 5; i++)
        ASSERT_EQ(test[i], truth[i]);
    test.print();
    test *= 0;
    for (int i = 0; i < 5; i++)
        ASSERT_EQ(test[i], 0);
}

//test composite method _of_
TEST(TestPolynomial, testComposition) {
    Polynomial<double, 4> f = {1, 3, 0, 5};
    Polynomial<double, 3> g{5, 0, 6};
    auto f_of_g = f._of_(g);
    Polynomial<double, 7> truth{641, 0, 2268, 0, 2700, 0, 1080};
    for (int i = 0; i < 7; i++)
        ASSERT_EQ(f_of_g[i], truth[i]);
    f_of_g.print();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
