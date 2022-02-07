#include <bmb_differentiation/compiled/TypeArray.h>
#include <bmb_differentiation/compiled/CDeepSimplify.h>
#include <bmb_differentiation/compiled/CConstant.h>

#include <gtest/gtest.h>

template<typename T1, typename T2>
auto f(T1 state, T2 dt) {
  using namespace compiled;

  auto state2 = state.set(0_i, state[3_i] * dt);
  auto state3 = state.set(1_i, state[4_i] * dt);
  auto state4 = state.set(2_i, state[5_i] * dt);
  return state4;
}

TEST(TestTypeArray, testTypeArray) {
  using namespace compiled;

  auto state = get_variables_type_array_t<6>{};
  auto dt = 6_v;
  auto f_func = f(state, dt);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
