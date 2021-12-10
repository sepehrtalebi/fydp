#include "TestDubinsPath.h"
#include "DubinsPath.h"
#include <cassert>
#include <fstream>
#include <iostream>

void testDubinsPath() {
    using Vector2 = Vector<double, 2>;
    DubinsPath<double>::State start = {Vector2{0, 0}, Vector2{0, 1}};
    DubinsPath<double>::State goal = {Vector2{10, 0}, Vector2{0, -1}};
    double radius = 1;
    DubinsPath<double> path = DubinsPath<double>::create(start, goal, radius);
    assert(path[0].is_turning && path[0].turn_right && !path[1].is_turning && path[2].is_turning && path[2].turn_right);

    std::ofstream out1("test/output/dubins_path1.csv");
    assert(out1);
    path.toCSV(out1, start, radius);
    out1.close();

    goal = {Vector2{5, 3}, Vector2{-1, 2}};
    path = DubinsPath<double>::create(start, goal, radius);
    std::ofstream out2("test/output/dubins_path2.csv");
    assert(out2);
    path.toCSV(out2, start, radius);
    out2.close();

    goal = {Vector2{0.2, 1.1}, Vector2{-0.2, -3}};
    path = DubinsPath<double>::create(start, goal, radius);
    std::ofstream out3("test/output/dubins_path3.csv");
    assert(out3);
    path.toCSV(out3, start, radius);
    out3.close();

    std::cout << "Passed all tests for DubinsPath!" << std::endl;
}
