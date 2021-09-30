#include "Variable.h"
#include <iostream>

double Variable::evaluate(const std::map<std::string, double> &variables) const {
    return variables.at(identifier);
}

Expression *Variable::diff(const std::string &id) const {
    return new Constant{identifier == id ? 1 : 0};
}

std::string Variable::toStr() const {
    return identifier;
}

Expression *Variable::copy() const {
    std::cout << "copying" << std::endl;
    return new Variable(identifier);
}
