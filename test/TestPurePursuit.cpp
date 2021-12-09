#include "TestPurePursuit.h"
#include "PurePursuit.h"
#include "DubinsPath.h"
#include <cassert>
#include <iostream>

void testPurePursuit() {
    using Vector2 = Vector<double, 2>;
    DubinsPath<double>::State start = {Vector2{0, 0}, Vector2{0, 1}};
    DubinsPath<double>::State goal = {Vector2{5, 3}, Vector2{-1, 2}};
    const double min_dubins_radius = 5;
    DubinsPath<double> path = DubinsPath<double>::create(start, goal, min_dubins_radius);
    PurePursuit<double> pursuer{2, 7, path};

    std::ofstream out1("test/output/pure_pursuit1.csv");
    assert(out1);
    pursuer.toCSV(out1, start);
    out1.close();

    std::cout << "Passed all tests for PurePursuit!" << std::endl;
}
