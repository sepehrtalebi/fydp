#include "TestTypeArray.h"
#include <bmb_utilities/TypeArray.h>
#include <bmb_differentiation/compiled/CDeepSimplify.h>

#include <iostream>

template<typename T1, typename T2>
auto f(T1 state, T2 dt) {
  using namespace compiled;

  auto state2 = state.set(0_i, state[3_i] * dt);
  auto state3 = state.set(1_i, state[4_i] * dt);
  auto state4 = state.set(2_i, state[5_i] * dt);
  return state4;
}

void testTypeArray() {
  using namespace compiled;

  auto state = get_variables_type_array_t<6>{};
  auto dt = 6_v;
  auto f_func = f(state, dt);

  std::cout << "Passed All Tests for Type Array!\n";
}
