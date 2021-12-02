#include "TestDubinsPath.h"
#include "DubinsPath.h"
#include <iostream>

void testDubinsPath() {
    using Vector2 = Vector<double, 2>;
    DubinsPath<double>::State start = {Vector2{0, 0}, Vector2{0, 1}};
    DubinsPath<double>::State goal = {Vector2{10, 0}, Vector2{0, -1}};
    double radius = 1;
    DubinsPath<double> path = DubinsPath<double>::create(start, goal, radius);
    std::cout << path.toStr() << std::endl;

    std::cout << "Passed all tests for DubinsPath!" << std::endl;
}
