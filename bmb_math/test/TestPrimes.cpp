#include <bmb_math/Primes.h>
#include <gtest/gtest.h>
#include <vector>

TEST(TestPrimes, testPrimeFactorization) {
    std::vector<size_t> factors69 = primeFactorization(69); // 3 * 23
    ASSERT_EQ(factors69.size(), 2);
    ASSERT_EQ(factors69[0], 3);
    ASSERT_EQ(factors69[1], 23);

    std::vector<size_t> factors140 = primeFactorization(140); // 2 * 2 * 5 * 7
    ASSERT_EQ(factors140.size(), 4);
    ASSERT_EQ(factors140[0], 2);
    ASSERT_EQ(factors140[1], 2);
    ASSERT_EQ(factors140[2], 5);
    ASSERT_EQ(factors140[3], 7);

    std::vector<size_t> factors2 = primeFactorization(2); // 2
    ASSERT_EQ(factors2.size(), 1);
    ASSERT_EQ(factors2[0], 2);
}

TEST(TestPrimes, testLowestPrimitiveRoot) {
    std::vector<size_t> test_cases{2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31};
    std::vector<size_t> outputs{1, 2, 2, 3, 2, 2, 3, 2, 5, 2, 3};
    for (size_t i = 0; i < test_cases.size(); i++) {
        ASSERT_EQ(lowestPrimitiveRootOfPrime(test_cases[i]), outputs[i]);
    }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
